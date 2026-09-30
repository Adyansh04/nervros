//! canopy's world editor, from the agent and the app: a client of its HTTP API
//! (`editor/canopy_editor.py`, which serves one saved world), and the tools that review, inspect
//! and edit that world through it, as the operator does by hand.

use std::borrow::Cow;
use std::sync::Arc;
use std::time::Duration;

use async_trait::async_trait;
use nervros_ros::RobotPort;
use secrecy::{ExposeSecret as _, SecretString};
use serde_json::{Value, json};

use crate::llm::{ImageFormat, ImageInput};
use crate::look::{Eyes, Snapshot, SnapshotStore};
use crate::profile::EditorConfig;
use crate::tools::{ImageArtifact, Risk, Tool, ToolOutcome, ToolSpec};

/// The ops `edit_world` passes on, as the editor names them.
pub const OPS: [&str; 13] = [
    "label",
    "name",
    "check",
    "remove",
    "restore",
    "merge",
    "box",
    "split",
    "add",
    "delete",
    "room_type",
    "room_name",
    "room_check",
];

/// How long a call may take; the editor answers from memory, a save writes a few files.
const TIMEOUT: Duration = Duration::from_secs(20);

/// Why the editor did not do what it was asked.
#[derive(Debug, Clone, thiserror::Error)]
#[non_exhaustive]
pub enum EditorError {
    /// Nothing answers at the address.
    #[error(
        "the world editor at {0} does not answer; start it on the world with canopy's \
         editor/canopy_editor.py <world_dir>"
    )]
    Unreachable(String),
    /// The editor refused, and says why.
    #[error("{0}")]
    Refused(String),
    /// canopy saved the world after the editor loaded it.
    #[error("canopy saved the world since the editor loaded it: {0}")]
    Conflict(String),
}

/// A client of one world editor.
#[derive(Debug)]
pub struct EditorClient {
    http: reqwest::Client,
    base: String,
    token: Option<SecretString>,
    /// The Trigger that makes the running world model read the saved world again.
    pub reload: Option<String>,
}

/// An edit's answer: the ids it created and the world after it.
#[derive(Debug, Clone)]
pub struct Edited {
    /// New object ids, from `split` and `add`.
    pub created: Vec<u64>,
    /// The whole world, as `/api/world` gives it.
    pub world: Value,
}

impl EditorClient {
    /// A client for the configured editor; its token, if any, is read now.
    ///
    /// # Errors
    ///
    /// The token cannot be read, or the HTTP client not built.
    pub fn new(config: &EditorConfig) -> Result<Self, String> {
        let token = config
            .token
            .as_ref()
            .map(crate::secret::KeySource::load)
            .transpose()
            .map_err(|e| format!("the world editor's token: {e}"))?;
        let http = reqwest::Client::builder()
            .timeout(TIMEOUT)
            .build()
            .map_err(|e| e.to_string())?;
        Ok(Self {
            http,
            base: config.url.trim_end_matches('/').to_owned(),
            token,
            reload: config.reload.clone(),
        })
    }

    async fn send(
        &self,
        request: reqwest::RequestBuilder,
    ) -> Result<reqwest::Response, EditorError> {
        let request = match &self.token {
            Some(t) => request.header("Cookie", format!("canopy_token={}", t.expose_secret())),
            None => request,
        };
        let response = request
            .send()
            .await
            .map_err(|_| EditorError::Unreachable(self.base.clone()))?;
        let status = response.status();
        if status.is_success() {
            return Ok(response);
        }
        let body: Value = response.json().await.unwrap_or(Value::Null);
        let why = body["error"]
            .as_str()
            .map_or_else(|| status.to_string(), str::to_owned);
        Err(if status == reqwest::StatusCode::CONFLICT {
            EditorError::Conflict(why)
        } else {
            EditorError::Refused(why)
        })
    }

    async fn post(&self, path: &str, body: &Value) -> Result<Value, EditorError> {
        let request = self.http.post(format!("{}{path}", self.base)).json(body);
        let response = self.send(request).await?;
        response
            .json()
            .await
            .map_err(|e| EditorError::Refused(e.to_string()))
    }

    /// The whole world, as the editor's page draws it.
    ///
    /// # Errors
    ///
    /// Any [`EditorError`].
    pub async fn world(&self) -> Result<Value, EditorError> {
        let response = self
            .send(self.http.get(format!("{}/api/world", self.base)))
            .await?;
        response
            .json()
            .await
            .map_err(|e| EditorError::Refused(e.to_string()))
    }

    /// Applies one op; a failed one leaves the world as it was.
    ///
    /// # Errors
    ///
    /// Any [`EditorError`]; [`EditorError::Refused`] says what was wrong with the op.
    pub async fn edit(&self, op: &Value) -> Result<Edited, EditorError> {
        let answer = self.post("/api/edit", op).await?;
        let created = answer["created"]
            .as_array()
            .map_or_else(Vec::new, |c| c.iter().filter_map(Value::as_u64).collect());
        Ok(Edited {
            created,
            world: answer["world"].clone(),
        })
    }

    /// `undo`, `redo`, `save`, `reload` or `rebase`: what the editor says, and the world after.
    ///
    /// # Errors
    ///
    /// Any [`EditorError`]; a save canopy got in before is [`EditorError::Conflict`].
    pub async fn command(&self, name: &str) -> Result<(String, Value), EditorError> {
        let answer = self.post(&format!("/api/{name}"), &json!({})).await?;
        let message = answer["message"].as_str().unwrap_or(name).to_owned();
        Ok((message, answer["world"].clone()))
    }

    /// Saves, replaying this session's edits over canopy's newer save first when it made one.
    ///
    /// # Errors
    ///
    /// Any [`EditorError`] but a first conflict.
    pub async fn save(&self) -> Result<(String, Value), EditorError> {
        match self.command("save").await {
            Err(EditorError::Conflict(_)) => {
                self.command("rebase").await?;
                self.command("save").await
            }
            other => other,
        }
    }

    /// What the camera saw of an object, if canopy kept a view of it.
    ///
    /// # Errors
    ///
    /// The editor does not answer.
    pub async fn crop(&self, id: u64) -> Result<Option<Vec<u8>>, EditorError> {
        let request = self.http.get(format!("{}/api/crops/O{id}.jpg", self.base));
        match self.send(request).await {
            Ok(response) => Ok(response.bytes().await.ok().map(|b| b.to_vec())),
            Err(EditorError::Refused(_)) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// The saved floor plan, as a greyscale PNG.
    ///
    /// # Errors
    ///
    /// Any [`EditorError`].
    pub async fn map_png(&self) -> Result<Vec<u8>, EditorError> {
        let response = self
            .send(self.http.get(format!("{}/api/map.png", self.base)))
            .await?;
        response
            .bytes()
            .await
            .map(|b| b.to_vec())
            .map_err(|e| EditorError::Refused(e.to_string()))
    }
}

/// An object id as the editor takes it: `12`, `"12"` or `"O12"`.
fn object_id(value: &Value) -> Option<u64> {
    value.as_u64().or_else(|| {
        value
            .as_str()?
            .trim()
            .trim_start_matches(['O', 'o'])
            .parse()
            .ok()
    })
}

/// What the review list says about one thing, and why.
fn review_line(world: &Value, item: &Value) -> Value {
    let what = if item["kind"] == "object" {
        let object = world["objects"]
            .as_array()
            .and_then(|all| all.iter().find(|o| o["id"] == item["id"]));
        let label = object.and_then(|o| o["label"].as_str()).unwrap_or("?");
        format!("O{} {label}", item["id"])
    } else {
        let room = world["rooms"]
            .as_array()
            .and_then(|all| all.iter().find(|r| r["id"] == item["id"]));
        let kind = room.and_then(|r| r["type"].as_str()).unwrap_or("");
        format!("{} {kind}", item["id"].as_str().unwrap_or("?"))
    };
    json!({"what": what.trim(), "reasons": item["reasons"], "phantom": item["phantom"]})
}

/// The `review_world` tool: what the world holds and what deserves a second look.
pub struct ReviewWorld {
    spec: ToolSpec,
    editor: Arc<EditorClient>,
}

impl ReviewWorld {
    /// The tool over an editor.
    #[must_use]
    pub fn new(editor: Arc<EditorClient>) -> Self {
        let spec = ToolSpec::new(
            "review_world",
            "Reads the saved world through the world editor: its rooms, how many objects, the \
             edits not saved yet, and what deserves a second look, most doubtful first, with why \
             (a likely phantom, a split vote, an untyped room). Look at one with inspect_object; \
             fix it with edit_world.",
            json!({"type": "object", "properties": {
                "limit": {"type": "integer", "minimum": 1, "maximum": 30, "description": "At most this many things to review (8)."}
            }, "additionalProperties": false}),
            Risk::Observe,
        );
        Self { spec, editor }
    }
}

#[async_trait]
impl Tool for ReviewWorld {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        let world = match self.editor.world().await {
            Ok(w) => w,
            Err(e) => return ToolOutcome::failed(e.to_string()),
        };
        let limit = args["limit"]
            .as_u64()
            .and_then(|l| usize::try_from(l).ok())
            .unwrap_or(8);
        let objects = world["objects"].as_array().map_or(&[][..], Vec::as_slice);
        let shown = objects.iter().filter(|o| o["shown"] == true).count();
        let removed = objects.iter().filter(|o| o["state"] == "removed").count();
        let rooms: Vec<Value> = world["rooms"]
            .as_array()
            .map_or(&[][..], Vec::as_slice)
            .iter()
            .map(|r| json!({"id": r["id"], "type": r["type"], "name": r["name"], "checked": r["checked"]}))
            .collect();
        let suggestions = world["suggestions"]
            .as_array()
            .map_or(&[][..], Vec::as_slice);
        let review: Vec<Value> = suggestions
            .iter()
            .take(limit)
            .map(|s| review_line(&world, s))
            .collect();
        let mut out = ToolOutcome::ok(json!({
            "rooms": rooms, "objects": shown, "removed": removed,
            "unsaved_edits": world["unsaved"], "to_review": review,
            "more_to_review": suggestions.len().saturating_sub(limit),
        }));
        out.message = format!(
            "{shown} objects in {} rooms; {} to review",
            rooms.len(),
            suggestions.len()
        );
        out
    }
}

/// The `inspect_object` tool: one object's facts and the camera's view of it.
pub struct InspectObject {
    spec: ToolSpec,
    editor: Arc<EditorClient>,
    snapshots: Arc<SnapshotStore>,
    eyes: Option<Arc<dyn Eyes>>,
}

impl InspectObject {
    /// The tool over an editor; with eyes it can answer questions about the object's view.
    #[must_use]
    pub fn new(
        editor: Arc<EditorClient>,
        snapshots: Arc<SnapshotStore>,
        eyes: Option<Arc<dyn Eyes>>,
    ) -> Self {
        let spec = ToolSpec::new(
            "inspect_object",
            "One object of the saved world: its label, name, the detector's votes, sightings, \
             size, height, state and why it is up for review, and the camera's best view of it, \
             which the operator sees. Ask `question` about the view, such as \"is this a chair or \
             a stool?\", and a vision model answers.",
            json!({"type": "object", "properties": {
                "id": {"type": "string", "description": "The object, such as \"O12\"."},
                "question": {"type": "string", "description": "What to ask about the camera's view of it."}
            }, "required": ["id"], "additionalProperties": false}),
            Risk::Observe,
        );
        Self {
            spec,
            editor,
            snapshots,
            eyes,
        }
    }

    async fn run(&self, args: &Value) -> Result<ToolOutcome, String> {
        let id = object_id(&args["id"]).ok_or("`id` is an object such as \"O12\"")?;
        let world = self.editor.world().await.map_err(|e| e.to_string())?;
        let object = world["objects"]
            .as_array()
            .and_then(|all| all.iter().find(|o| o["id"].as_u64() == Some(id)))
            .ok_or_else(|| format!("no object O{id} in the saved world"))?;
        let reasons = world["suggestions"]
            .as_array()
            .and_then(|all| {
                all.iter()
                    .find(|s| s["kind"] == "object" && s["id"].as_u64() == Some(id))
            })
            .map_or(json!([]), |s| s["reasons"].clone());
        let votes: Vec<Value> = object["votes"]
            .as_object()
            .map(|v| v.iter().take(4).map(|(k, n)| json!([k, n])).collect())
            .unwrap_or_default();
        let mut data = json!({
            "id": format!("O{id}"), "label": object["label"], "name": object["name"],
            "caption": object["caption"], "votes": votes, "sightings": object["observations"],
            "size_m": object["size"], "height_m": [object["z_min"], object["z_max"]],
            "centre": object["centre"], "yaw": object["yaw"], "state": object["state"],
            "removed_by": object["removed_by"], "checked": object["checked"],
            "box_set_by_hand": object["box_pinned"], "review": reasons,
        });
        let mut images = Vec::new();
        match self.editor.crop(id).await.map_err(|e| e.to_string())? {
            Some(jpeg) => {
                let (width, height) =
                    image::load_from_memory(&jpeg).map_or((0, 0), |i| (i.width(), i.height()));
                let artifact = ImageArtifact {
                    snapshot: self.snapshots.next_id(),
                    jpeg: Arc::new(jpeg.clone()),
                    width,
                    height,
                };
                self.snapshots.put(Snapshot {
                    id: artifact.snapshot.clone(),
                    stamp_s: 0.0,
                    marks: Vec::new(),
                    image: artifact.clone(),
                });
                data["snapshot"] = json!(artifact.snapshot);
                if let (Some(eyes), Some(q)) = (&self.eyes, args["question"].as_str()) {
                    let prompt = format!(
                        "{q}\n\nThis is the camera's best view of O{id}, which the world model \
                         calls a {}. The grey around it is masked out.",
                        object["label"].as_str().unwrap_or("thing")
                    );
                    let image = ImageInput {
                        bytes: jpeg,
                        format: ImageFormat::Jpeg,
                    };
                    match eyes.see(&prompt, image).await {
                        Ok((answer, model)) => {
                            data["answer"] = json!(answer);
                            data["seen_by"] = json!(model);
                        }
                        Err(why) => data["not_seen"] = json!(why),
                    }
                }
                images.push(artifact);
            }
            None => data["view"] = json!("canopy kept no view of it"),
        }
        let mut out = ToolOutcome::ok(data);
        out.message = if images.is_empty() {
            format!("O{id}: no view of it")
        } else {
            format!("O{id}; the user sees the camera's view of it")
        };
        out.images = images;
        Ok(out)
    }
}

#[async_trait]
impl Tool for InspectObject {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.run(&args).await.unwrap_or_else(ToolOutcome::failed)
    }
}

/// The `edit_world` tool: the editor's ops, in order, all or none.
pub struct EditWorld {
    spec: ToolSpec,
    editor: Arc<EditorClient>,
}

impl EditWorld {
    /// The tool over an editor.
    #[must_use]
    pub fn new(editor: Arc<EditorClient>) -> Self {
        let spec = ToolSpec::new(
            "edit_world",
            "Edits the saved world as the operator does in the world editor: the edits run in \
             order, and if one fails the earlier ones are undone. Ops: label (id, label; \"\" \
             hands it back to the detector's votes), name (id, name), check (id, checked), \
             remove or restore (ids, reason), merge (into, ids), box (id, centre, size, yaw), \
             split (id, polygon, label, name), add (label, centre, size, yaw, z_min, z_max, \
             name), delete (id; prefer remove, which can be undone later), room_type (room, \
             type), room_name (room, name), room_check (room, checked). Positions are map \
             metres, yaw radians. Nothing is kept until world_edits saves.",
            json!({"type": "object", "properties": {
                "edits": {"type": "array", "minItems": 1, "maxItems": 20, "items": {"type": "object", "properties": {
                    "op": {"type": "string", "enum": OPS},
                    "id": {"type": "string", "description": "The object, such as \"O12\"."},
                    "ids": {"type": "array", "items": {"type": "string"}, "description": "Objects, for remove, restore and merge."},
                    "into": {"type": "string", "description": "For merge: the object the others become part of."},
                    "label": {"type": "string"},
                    "name": {"type": "string"},
                    "checked": {"type": "boolean"},
                    "reason": {"type": "string"},
                    "centre": {"type": "array", "items": {"type": "number"}, "description": "[x, y]."},
                    "size": {"type": "array", "items": {"type": "number"}, "description": "[length, width] in metres."},
                    "yaw": {"type": "number"},
                    "polygon": {"type": "array", "items": {"type": "array", "items": {"type": "number"}}, "description": "For split: the part's outline, [[x, y], ...]."},
                    "z_min": {"type": "number"},
                    "z_max": {"type": "number"},
                    "room": {"type": "string", "description": "The room, such as \"R3\"."},
                    "type": {"type": "string", "description": "For room_type: kitchen, bedroom, ..."}
                }, "required": ["op"]}}
            }, "required": ["edits"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self { spec, editor }
    }

    async fn run(&self, args: &Value) -> Result<ToolOutcome, String> {
        let edits = args["edits"]
            .as_array()
            .filter(|e| !e.is_empty())
            .ok_or("`edits` is a list of edits")?;
        if let Some(bad) = edits
            .iter()
            .find(|e| !e["op"].as_str().is_some_and(|op| OPS.contains(&op)))
        {
            return Err(format!(
                "`{}` is no edit; the ops are {}",
                bad["op"],
                OPS.join(", ")
            ));
        }
        let (mut created, mut world) = (Vec::new(), Value::Null);
        for (done, op) in edits.iter().enumerate() {
            match self.editor.edit(op).await {
                Ok(e) => {
                    created.extend(e.created.iter().map(|id| format!("O{id}")));
                    world = e.world;
                }
                Err(why) => {
                    let mut undone = 0;
                    for _ in 0..done {
                        if self.editor.command("undo").await.is_ok() {
                            undone += 1;
                        }
                    }
                    return Err(format!(
                        "edit {} ({}) failed: {why}; the {undone} before it were undone",
                        done + 1,
                        op["op"].as_str().unwrap_or("?")
                    ));
                }
            }
        }
        let mut out = ToolOutcome::ok(json!({
            "applied": edits.len(), "created": created, "unsaved_edits": world["unsaved"],
        }));
        out.message = format!(
            "{} edit(s) applied, not saved: world_edits saves them",
            edits.len()
        );
        Ok(out)
    }
}

#[async_trait]
impl Tool for EditWorld {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.run(&args).await.unwrap_or_else(ToolOutcome::failed)
    }
}

/// The `world_edits` tool: save, undo, redo or discard the edits made so far.
pub struct WorldEdits {
    spec: ToolSpec,
    editor: Arc<EditorClient>,
    robot: Arc<dyn RobotPort>,
}

impl WorldEdits {
    /// The tool over an editor; the robot is for the world model's reload after a save.
    #[must_use]
    pub fn new(editor: Arc<EditorClient>, robot: Arc<dyn RobotPort>) -> Self {
        let spec = ToolSpec::new(
            "world_edits",
            "Saves the world edits made so far, by the operator or by edit_world, and has the \
             running world model read them; or undoes or redoes the last one, or discards every \
             edit not saved.",
            json!({"type": "object", "properties": {
                "action": {"type": "string", "enum": ["save", "undo", "redo", "discard"]}
            }, "required": ["action"], "additionalProperties": false}),
            Risk::Annotate,
        );
        Self {
            spec,
            editor,
            robot,
        }
    }

    async fn run(&self, args: &Value) -> Result<ToolOutcome, String> {
        let action = args["action"].as_str().unwrap_or_default();
        let (message, world) = match action {
            "save" => self.editor.save().await,
            "undo" | "redo" => self.editor.command(action).await,
            "discard" => self.editor.command("reload").await,
            other => {
                return Err(format!(
                    "`action` is save, undo, redo or discard, not `{other}`"
                ));
            }
        }
        .map_err(|e| e.to_string())?;
        let mut data = json!({"said": message, "unsaved_edits": world["unsaved"]});
        if action == "save"
            && let Some(reload) = &self.editor.reload
        {
            let answer = self
                .robot
                .call(reload, "std_srvs/srv/Trigger", json!({}), TIMEOUT)
                .await;
            data["world_model"] = match answer {
                Ok(a) if a["success"] == true => json!("read the saved world"),
                Ok(a) => json!(format!(
                    "did not read it: {}",
                    a["message"].as_str().unwrap_or("no reason")
                )),
                Err(e) => json!(format!("{reload} did not answer: {e}")),
            };
        }
        let mut out = ToolOutcome::ok(data);
        out.message = message;
        Ok(out)
    }
}

#[async_trait]
impl Tool for WorldEdits {
    fn spec(&self) -> Cow<'_, ToolSpec> {
        Cow::Borrowed(&self.spec)
    }

    async fn call(&self, args: Value) -> ToolOutcome {
        self.run(&args).await.unwrap_or_else(ToolOutcome::failed)
    }
}

/// The four editor tools.
#[must_use]
pub fn tools(
    editor: &Arc<EditorClient>,
    robot: &Arc<dyn RobotPort>,
    snapshots: &Arc<SnapshotStore>,
    eyes: Option<Arc<dyn Eyes>>,
) -> Vec<Arc<dyn Tool>> {
    vec![
        Arc::new(ReviewWorld::new(Arc::clone(editor))),
        Arc::new(InspectObject::new(
            Arc::clone(editor),
            Arc::clone(snapshots),
            eyes,
        )),
        Arc::new(EditWorld::new(Arc::clone(editor))),
        Arc::new(WorldEdits::new(Arc::clone(editor), Arc::clone(robot))),
    ]
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;
    use std::sync::atomic::{AtomicUsize, Ordering};

    use tokio::io::{AsyncReadExt as _, AsyncWriteExt as _};

    use super::*;

    type Answer = Arc<dyn Fn(&str, &Value) -> (u16, Value) + Send + Sync>;

    /// A stand-in editor on a free port: answers each request by path, and logs the paths and ops.
    async fn stub(answer: Answer) -> (Arc<EditorClient>, Arc<Mutex<Vec<String>>>) {
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let url = format!("http://{}", listener.local_addr().unwrap());
        let log = Arc::new(Mutex::new(Vec::new()));
        let seen = Arc::clone(&log);
        tokio::spawn(async move {
            while let Ok((mut socket, _)) = listener.accept().await {
                let (seen, answer) = (Arc::clone(&seen), Arc::clone(&answer));
                tokio::spawn(async move {
                    let mut buf = Vec::new();
                    let mut chunk = [0_u8; 4096];
                    let (head, body) = loop {
                        let n = socket.read(&mut chunk).await.unwrap_or(0);
                        buf.extend_from_slice(&chunk[..n]);
                        let text = String::from_utf8_lossy(&buf).to_string();
                        if let Some((head, body)) = text.split_once("\r\n\r\n") {
                            let want = head
                                .lines()
                                .find_map(|l| {
                                    l.to_ascii_lowercase()
                                        .strip_prefix("content-length:")
                                        .map(|v| v.trim().parse().unwrap_or(0))
                                })
                                .unwrap_or(0_usize);
                            if body.len() >= want || n == 0 {
                                break (head.to_owned(), body.to_owned());
                            }
                        }
                        if n == 0 {
                            return;
                        }
                    };
                    let path = head.split_whitespace().nth(1).unwrap_or("/").to_owned();
                    let request: Value = serde_json::from_str(&body).unwrap_or(Value::Null);
                    let op = request["op"]
                        .as_str()
                        .map_or(String::new(), |o| format!(" {o}"));
                    seen.lock().unwrap().push(format!("{path}{op}"));
                    let (status, reply) = answer(&path, &request);
                    let text = reply.to_string();
                    let out = format!(
                        "HTTP/1.1 {status} X\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{text}",
                        text.len()
                    );
                    let _ = socket.write_all(out.as_bytes()).await;
                });
            }
        });
        let config: EditorConfig = toml::from_str(&format!("url = \"{url}\"")).unwrap();
        (Arc::new(EditorClient::new(&config).unwrap()), log)
    }

    #[tokio::test]
    async fn a_failed_edit_undoes_the_ones_before_it() {
        let answer: Answer = Arc::new(|path, request| match (path, request["op"].as_str()) {
            ("/api/edit", Some("name")) => (400, json!({"error": "no object O99"})),
            ("/api/edit", _) => (200, json!({"created": [], "world": {"unsaved": 1}})),
            _ => (200, json!({"message": "undone", "world": {"unsaved": 0}})),
        });
        let (editor, log) = stub(answer).await;
        let tool = EditWorld::new(editor);
        let out = tool
            .call(json!({"edits": [
                {"op": "label", "id": "O3", "label": "stool"},
                {"op": "name", "id": "O99", "name": "x"},
                {"op": "check", "id": "O3"}
            ]}))
            .await;
        assert_eq!(out.status, crate::tools::Status::Failed);
        assert!(
            out.message
                .contains("edit 2 (name) failed: no object O99; the 1 before it were undone"),
            "{}",
            out.message
        );
        assert_eq!(
            *log.lock().unwrap(),
            ["/api/edit label", "/api/edit name", "/api/undo"]
        );
        let refused = tool.call(json!({"edits": [{"op": "teleport"}]})).await;
        assert!(
            refused.message.contains("the ops are label"),
            "{}",
            refused.message
        );
    }

    #[tokio::test]
    async fn a_save_canopy_got_in_before_is_rebased_and_saved_again() {
        let saves = Arc::new(AtomicUsize::new(0));
        let answer: Answer = Arc::new(move |path, _| match path {
            "/api/save" if saves.fetch_add(1, Ordering::SeqCst) == 0 => {
                (409, json!({"error": "world.yaml changed on disk"}))
            }
            _ => (
                200,
                json!({"message": "saved 2 edits", "world": {"unsaved": 0}}),
            ),
        });
        let (editor, log) = stub(answer).await;
        let robot: Arc<dyn RobotPort> = Arc::new(nervros_ros::fake::FakeRobot::new());
        let out = WorldEdits::new(editor, robot)
            .call(json!({"action": "save"}))
            .await;
        assert_eq!(
            out.status,
            crate::tools::Status::Succeeded,
            "{}",
            out.message
        );
        assert_eq!(
            *log.lock().unwrap(),
            ["/api/save", "/api/rebase", "/api/save"]
        );
    }

    #[test]
    fn object_ids_are_read_as_the_editor_reads_them() {
        assert_eq!(object_id(&json!(12)), Some(12));
        assert_eq!(object_id(&json!("12")), Some(12));
        assert_eq!(object_id(&json!("O12")), Some(12));
        assert_eq!(object_id(&json!("chair")), None);
    }

    #[test]
    fn a_review_line_names_the_thing_and_says_why() {
        let world = json!({"objects": [{"id": 3, "label": "stool"}], "rooms": [{"id": "R2", "type": "kitchen"}]});
        let line = review_line(
            &world,
            &json!({"kind": "object", "id": 3, "reasons": ["split vote"], "phantom": false}),
        );
        assert_eq!(line["what"], "O3 stool");
        let line = review_line(
            &world,
            &json!({"kind": "room", "id": "R2", "reasons": ["no type"], "phantom": false}),
        );
        assert_eq!(line["what"], "R2 kitchen");
    }
}
