# Using NervROS

Every way to use NervROS, each with a short clip and the exact words typed. The clips are real
runs: a simulated Unitree G1 in a six-room apartment ([grove-g1](https://github.com/Adyansh04/grove-g1)),
answered by the local Qwen3.5-9B model. Missions play three or four times faster, and the
23-minute mapping run keeps only its start and end; the rest is real time. Setup is in the [README](../README.md); every setting is in the [profile](profile.md),
[models](models.md) and [executor](executor.md) pages.

- [The window](#the-window)
- [Asking what it sees](#asking-what-it-sees)
- [Giving it a task](#giving-it-a-task)
- [Clicking in the 3D view](#clicking-in-the-3d-view)
- [Commands without the model](#commands-without-the-model)
- [Driving by hand](#driving-by-hand)
- [Keeping an eye on it](#keeping-an-eye-on-it)
- [Remembering](#remembering)
- [Mapping a new building](#mapping-a-new-building)
- [Finishing the map](#finishing-the-map)
- [Sessions and models](#sessions-and-models)
- [The command line](#the-command-line)
- [Keys](#keys)

## The window

```bash
source scripts/ros-env.sh
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

![The window: chat, 3D world and cameras, dock](https://github.com/user-attachments/assets/d0e52596-0a56-49af-a0d3-42b28c34d7ef)

- **Top bar:** the robot, then the state of ROS, the mission executor and the model. On the
  right: the arming switch (until it is on, the agent can look and answer but not act),
  **Reset layout**, **Follow robot**, **Edit world**, **Dock** and **Stop mission**, which is
  always there.
- **Chat, left:** Enter sends, Shift+Enter adds a line, Esc stops the reply.
- **Viewer, middle:** the 3D world (map, rooms, objects, the robot, where it walked and is about
  to walk), the cameras with what the detector marks, and below them the mission's steps over
  time, the agent's log and plots. It is one timeline you can scrub back.
- **Dock, right:** Mission, World, Layers, Approvals, Events, Agent, Doctor and Robot
  (Ctrl+1 to Ctrl+8). A waiting approval turns its tab amber.
- **Status bar:** whether the agent is working, how full the model's context is, and how much of
  it came from the model server's cache.

## Asking what it sees

https://github.com/user-attachments/assets/84720295-ae66-47d6-a6be-060d07b3e0eb

> **You:** What can you see right now?
>
> **NervROS:** I can see a red wardrobe (mark 1) with two doors, positioned against a plain wall
> on a wooden floor.

`look` takes the newest camera frame and draws each detection as a numbered mark, which you and
the model both refer to by number. Under the image, **Show in 3D** pauses the viewer at the
moment the snapshot was taken, and **Ask about it** starts a message about it.

> **You:** Where is the small white mug?
>
> **NervROS:** I found the small white mug (O783) in room R1. It's located at position x=-1.91,
> y=2.67 meters with a height of 0.78 meters. Would you like me to pick it up?

`find_objects` searches the world model, which remembers everything the cameras have seen,
room by room.

> **You:** Which rooms do you know, and how much of each have you seen?

`list_places` answers with each room's type, how much of its floor and walls the camera has
seen, and how many objects it holds, then the named places.

### Pointing and outlining

https://github.com/user-attachments/assets/f32f4b9b-bd5e-4961-a186-ebc9150aff91

> **You:** Point at the tray.
>
> **NervROS:** I've pointed at the tray. You can see it marked as mark 4 in the image.
>
> **You:** Outline the desk.
>
> **NervROS:** I've outlined the desk in the image. The desk is marked as region 1 and covers
> about 11.5% of the frame.

`point` asks a vision model, so it finds things the detector never marked, and draws a ring where
it points. `segment` outlines what you name, from a vision model or the robot's own segmentation
service.

## Giving it a task

Anything that moves the robot is a mission. The agent writes a plan from the robot's own
skills, NervROS compiles it to a behaviour tree, the executor checks it, and only then does it
reach you, as a card to approve.

### Walking somewhere

https://github.com/user-attachments/assets/34cf6674-b1e5-4dec-81d7-1bfdb72a0d9f

> **You:** Walk to the bedroom, room R3.

The plan card lists each step with its track record on this robot ("13 of 15 · 24 s"), and the
viewer draws the planned path. While it runs, each step shows its live state, the Mission tab
shows the behaviour tree node by node, and the timeline strip shows it over time. When it ends,
the agent gets a report with its goal checks:

```text
report> Mission 01a0feba (Walk to the bedroom, room R3) ended: success after 24 s.
        Goal checks: at(R3) holds (the robot is in R3).
```

### A short one, and the plan check

https://github.com/user-attachments/assets/a0839b5c-ef27-43ef-b483-2c50629f3e90

> **You:** Turn left 90 degrees.

Plans are checked against your words: left or right, forward or back, how far, how much, which
hand. Here the model got the sign right. In an earlier run its first plan was
`TurnInPlace(degrees=-90)`, a right turn, and it never reached the card:

```text
run_mission failed s1: the operator said turn left, but degrees=-90 turns the other way
(positive degrees turn left). The plan may not do what the operator asked: fix it, or if it is
right, send it again unchanged and the operator sees these concerns when approving
```

When the model insists, the concern stays on the card with a one-click fix. **Edit** changes,
drops or moves a step before you approve.

### Fetching something

https://github.com/user-attachments/assets/dc4712e6-81cc-4f7a-b986-baae0cda8bf9

> **You:** Bring the small white mug from the dining table to the tray on the office desk.

The plan has four steps, and the card tracks each live while the arms work (the clip plays
four times faster while the mission runs):

```text
plan: GoToPlace(place=dining_table_side) -> PickObject(object_id=mug_4, arm=left)
      -> GoToPlace(place=office_desk_tray) -> PlaceInto(container_id=tray_1, arm=left)
```

Under three minutes later the report comes back with its checks. The world model could not judge
"inside" for these two objects, which only the arm's detector knows, so the camera did: the
detector marks a mug inside the tray's box.

```text
report> Mission 01a0ff3c (Bring the small white mug from the dining table to the tray on the
        office desk) ended: success after 170 s. Camera check: inside(mug_4, tray_1): the
        camera agrees: the detector marks a mug (mark 1) in or on the tray (mark 5).
```

> **NervROS:** The mission was successful! The small white mug has been brought from the dining
> table to the tray on the office desk.

The Mission tab shows the behaviour tree as it runs, node by node, beside the track record of
each step and the missions before it.

A failed mission comes back with the step that failed, why, and what the world model saw of the
object it was about; the agent then proposes a changed plan for you to approve. A mission keeps
running while you talk.

### Stopping

https://github.com/user-attachments/assets/8a2d0caa-46ec-472e-8664-ebf82297b63b

> **You:** Walk to the office, room R4.
>
> **You:** Stop.

A plain "stop" (or "halt", "freeze", "stop!") never waits on a model: it stops the reply and the
robot at once, through the executor's `StopAll`. In the clip the walk had run for seven seconds:

```text
[halted: stopped by the operator: reply and robot]
[the robot stopped: holding posture; hands empty]
report> Mission 01a0febc (Walk to the office, room R4) ended: canceled after 9 s.
        It was stopped during s1 GoToPlace(place=R4) (stopped: operator).
```

Esc stops only the reply;
**Stop mission** or Ctrl+Shift+S stops the robot. A stop never opens a hand: a robot holding a
mug keeps it. While a mission runs, the window sends the executor a heartbeat, so a mission
whose window closed or hung stops by itself.

### Doing it again

- "Save that as the evening check" keeps a plan that worked; "Run the evening check" runs it
  again. The Mission tab lists saved plans.
- A schedule runs a plan on a clock or when something happens, approved once for every run
  (see [Keeping an eye on it](#keeping-an-eye-on-it)). Stopping the robot cancels every schedule.
- Requests no skill can do are logged as skill gaps, for whoever writes the next skill.

## Clicking in the 3D view

https://github.com/user-attachments/assets/a6f03d25-e631-441d-940b-dda37a833435

Click a room, an object or a point on the floor in the 3D view: it lights up, and a bar above the
message box offers what the agent can do with it. **Go there** plans the walk at once, for you to approve;
the other buttons fill in a message for you to send. In the clip: the office, **Go there**; then
the desk, **What is it?**, which sends "Tell me about O47 (desk).":

> **NervROS:** O47 is a large grey metal desk with multiple drawers on both sides. It measures 2m
> x 0.95m and is about 0.88m tall.

**Follow robot** in the top bar keeps the 3D view on the robot as it walks; off, the view frames
the whole map.

## Commands without the model

https://github.com/user-attachments/assets/4fdabfda-5e5e-4c1d-a830-6af4db0a406b

Ctrl+K opens the command palette: type a few letters and Enter. Every command also runs as a
`/word` typed in the chat. Most need no model: they open a tab, change the view or stop the
robot; `/look`, `/places` and `/explore` send their request to the agent. In the clip: Ctrl+K,
"follow", Enter; then `/layers` and `/doctor`.

| Command | |
|---|---|
| `/look` | Ask the agent to look through the camera |
| `/places` | Ask which places it can go to |
| `/stop` | Stop the robot |
| `/arm`, `/disarm` | Let the agent act, or only observe |
| `/explore` | Ask it to explore the building |
| `/compact` | Condense the conversation |
| `/follow` | Follow the robot in 3D, or see the whole map |
| `/layout` | Put the viewer's panes back |
| `/edit` | The world editor |
| `/live` | Back to the live view after **Show in 3D** |
| `/doctor`, `/robot`, `/models`, `/missions`, `/world`, `/approvals`, `/events`, `/layers`, `/dock` | Open that tab of the dock |

## Driving by hand

https://github.com/user-attachments/assets/5b176fdf-aa97-401b-9e44-2cf031865379

The Robot tab shows whether the robot is upright, where it is and in which room, what each hand
holds, and its motors and battery when the robot reports them. **Drive** asks the executor for
the base: W and S walk, A and D turn, Q and E step sideways, the pad lights the key held, and
**Slow** halves the speed. The
executor refuses while a mission runs and stops the base when the keys stop, so a closed window
stops the robot too. Driving needs the robot armed.

## Keeping an eye on it

https://github.com/user-attachments/assets/891d3213-9841-4bc5-9f79-a09c3a4be631

> **You:** Tell me if the chest camera's images drop below 5 Hz.
>
> **NervROS:** I'm now watching the chest camera's image rate. I'll let you know if it drops below
> 5 Hz within the next 30 minutes.

A watch runs in the background and posts to the chat when its condition holds: a rate dropping,
a value crossing a line, or a log line matching.

> **You:** Plot the robot's forward speed for the next two minutes.

The viewer brings its **Plots** tab forward and draws the robot's speed live: still, then the
scheduled walk below.

> **You:** Every ten minutes, walk to the bedroom, room R3, three times.

A schedule is approved once, runs now and then on its clock, and reports after each run:

```text
report> Mission 01a0ff18 (Walk to bedroom R3 every 10 minutes, 3 times, run 1 of 3 of schedule
        p1) ended: success after 18 s. Goal checks: at(R3) holds (the robot is in R3).
```

The
Mission tab lists it with a **Cancel** button. A schedule can also wait for something: an object
of a kind appearing, or a condition on a topic.

"Check the robot's health" runs the same connection check as the Doctor tab, asked even while a
mission runs.

### Debugging the ROS side

With a `[ros_tools]` table the agent reads the graph as `ros2` does: "what is publishing the
map?", "is the chest camera dropping frames?", "show me the controller's parameters", "any errors
in the logs?". Acting through ROS (a service call, an action goal, a parameter, a publish) needs
the profile to allow it, and your approval each time.

## Remembering

https://github.com/user-attachments/assets/60e89e67-063b-417f-920d-92c33ad09105

> **You:** Remember this spot as the charging spot.
>
> **You:** Remember that the living room window sticks.
>
> **You:** What do you remember?
>
> **NervROS:** I remember two things: 1. The living room window sticks (note 1) 2. This spot is
> the charging spot ...

A named place works from then on, in this session and the next: "go to the charging spot". Notes
join what the agent knows on every turn, and the Agent tab lists them with **Forget**. "What have
you done so far?" and "what happened to the mug?" read the mission ledger and the world model's
history.

## Mapping a new building

https://github.com/user-attachments/assets/90340180-66da-4bc3-a7d9-b2e6a1f24c7b

> **You:** Explore the building to fill in the map.

In a building it has never seen, the robot maps as it explores. SLAM draws the floor plan, the
world model splits it into rooms and types each one from the objects it finds there, and the
viewer shades in orange what the camera has yet to see. The plan is one step,
`ExploreBuilding()`, approved once. The clip shows the first four minutes and the last one, four
times faster:

```text
report> Mission 01a0fe14 (Explore the building to fill in the map) ended: success after 1380 s.
```

In 23 minutes the robot found six rooms and 45 objects. The World tab shows how much of each
room's floor and walls the camera has seen; its **Explore** button, or `/explore`, asks for the
same mission. In grove-g1, `./scripts/demos/nervros.sh map name:=my-building` starts the
simulator with no map and saves the new world as `data/worlds/my-building`, and
`nervros.sh app saved:=my-building` runs on it afterwards.

## Finishing the map

A new map needs a person's eye. The world model types each room from the objects it finds there
and labels each object from what the detector saw, so a study with a daybed and an armchair can
come out as a living room. Fix it by hand in the world editor, or in the chat. Nothing is kept
until it is saved, and a save reaches the running world model at once.

### In the editor

https://github.com/user-attachments/assets/c1385c2b-85c4-4dec-961d-4a33d173f93c

**Edit world** in the top bar (or `/edit`) puts the editor in place of the 3D view: the floor plan
with every room and object, and in the dock the selection and a list to review, most doubtful
first. Here the list has one entry, R7, typed "living room" from its objects at 0.30. It is the
study, so: pick it, type "study" in **Type**, **Set type**, **Mark checked**, then **Save**.

```text
saved 2 edits to data/worlds/media-mapped; the world model read them
```

On the plan, click an object to select it, drag it to move it, drag a corner to resize it or the
handle to turn it. **Add object** draws a box round something the detector missed, **Split off…**
and **Merge with…** fix one object seen as two or two seen as one, and **Delete** asks whether to
delete it for good or only remove it, which **Restore** undoes. N goes to the next thing to
review, C checks the selection, Del deletes it, and **Undo** and **Redo** reach back to the last
save.

### In the chat

https://github.com/user-attachments/assets/4606a6ec-f39d-44b3-a4ec-c19bbd7f3b64

> **You:** Review the new map. What needs a second look?
>
> **NervROS:** The world model shows 45 objects across 6 rooms. There are no items flagged for
> review right now. However, I notice several rooms are marked as unchecked (R6, R3, R1, R8, R4).
>
> **You:** R1 is the living room, with the kitchen and the dining table in it. Change its type.
>
> **You:** O59 in the study is a daybed, not a bed.
>
> **You:** Save the world edits.
>
> **NervROS:** The world edits have been saved. The two changes (R1 as living room and O59 as
> daybed) are now committed to the world model.

Before relabelling O59 the agent looked at it: `inspect_object` shows the camera's best view of an
object, a daybed here. Each edit is approved on its card and waits, unsaved, until the save. The
agent has the editor's tools: `review_world` lists what deserves a second look, `inspect_object`
shows an object, `edit_world` relabels, renames, checks, removes, merges, splits, adds and moves
objects and types rooms, and `world_edits` saves, undoes, redoes or discards. The editor and the
chat share one history, so an edit made in one shows in the other.

## Sessions and models

Every conversation is saved. The Agent tab lists earlier sessions to resume, what the agent
remembers, and each model with its quota for the day. The context is condensed before it fills;
`/compact` condenses it at once, and right-clicking one of your messages condenses everything
before it. A model that hits its rate limit is set aside, and one whose server is not running is
tried last, so the next one in its role's chain answers; a busy one is asked once more first, and
when every model is at its limit for the minute the turn waits for the first to free up. **Save this one
as a test** turns the session into an eval case.

## The command line

`nervros-cli` runs the same agent without a window, from the same profile:

```bash
nervros-cli --profile my.toml chat                     # /arm, /yes N, /no N, /stop, /compact, /quit
nervros-cli --profile my.toml chat --resume last
nervros-cli --profile my.toml ask "What can you see?" --image frame.jpg
```

| Command | |
|---|---|
| `chat` | Chat with the robot; `--say` sends a script of messages and exits |
| `ask` | One prompt, optionally with an image, to one role's models |
| `look`, `segment` | The camera's marks, or the regions a prompt names, saved as an image |
| `ros` | One read-only ROS tool, no model: graph, topic sample, interface, TF, parameters, logs |
| `doctor` | The profile's tools, topics and mission services against the live graph |
| `missions`, `gaps` | The mission ledger, and the requests no skill could do |
| `replay` | A session as it happened |
| `eval`, `case` | Test suites against the live robot, and a session kept as a new case |
| `models`, `skills`, `mcp` | The model chains with today's counts, the skills, and the MCP tools to approve |

`doctor` checks the profile against the live graph, and `ros` runs one read tool with no model:

```text
$ nervros-cli --profile nervros.toml doctor
ok      657 interface definitions loaded
ok      tool find_objects -> /canopy/find_objects (canopy_msgs/srv/FindObjects)
ok      look image /chest_camera/color/image_raw
ok      look detections /detector_chest/instance_masks
ok      mission validate /nervros_executor/validate_mission
ok      mission catalog /nervros_executor/get_catalog
ok      mission stop /nervros_executor/stop_all

$ nervros-cli --profile nervros.toml ros topic_sample \
    '{"topic": "/chest_camera/color/image_raw", "mode": "hz", "seconds": 3}'
{
  "bytes_per_s": 12212000.0,
  "gap_max_s": 0.14,
  "gap_min_s": 0.064,
  "messages": 30,
  "rate_hz": 10.019,
  "type": "sensor_msgs/msg/Image",
  "window_s": 3.0
}
```

`missions` reads the ledger, each mission with what was asked and the step that failed:

```text
$ nervros-cli --profile nervros.toml missions --limit 3
01a0fe14-7585 30 min ago: Explore the building to fill in the map (success, 1380 s)
    asked: Explore the building to fill in the map.
01a0fe11-aac0 33 min ago: walk to bedroom R3 every 10 minutes, 3 times, run 1 of 3 of schedule p1 (success, 15 s)
    asked: Every ten minutes, walk to the bedroom, room R3, three times.
01a0fe10-ae04 34 min ago: Walk to the bedroom, room R3 (canceled, 9 s)
    asked: Walk to the bedroom, room R3.
    s1 GoToPlace R3 failure: stopped: operator
```

### Evals

`eval` runs a suite of requests against the live robot, each in a fresh session with every
approval granted, and judges what happened: the tools and skills used, the missions' outcomes,
the approvals asked for, the reply, and how far and which way the robot moved on the simulator's
own pose. The apartment suite in grove-g1 has 44 cases:

```text
$ nervros-cli --profile nervros.toml eval eval/apartment.toml --out eval-r3
== look #1
   PASS in 3 s, 2 calls
== turn-left #1
   PASS in 4 s, 3 calls
== pick-place #1
   FAIL in 236 s, 12 calls: 3 approvals, at most 1 wanted
...
42 of 44 trials passed; report eval-r3/report.md
```

`--repeat k` runs each case k times for pass^k, `--models a,b` compares models, and
`nervros-cli case last --id my-case --append suite.toml` keeps a session as a new case.

## Keys

| Key | |
|---|---|
| Enter, Shift+Enter | Send; a new line |
| Esc | Stop the reply |
| Ctrl+Shift+S | Stop the mission |
| Ctrl+K | The command palette |
| Ctrl+1 to Ctrl+8 | The dock's tabs |
| W S, A D, Q E | Drive by hand: walk, turn, step sideways (Robot tab, Drive on) |
