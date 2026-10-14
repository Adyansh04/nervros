# Using NervROS

Every way to use NervROS, most with a short clip, and the exact words typed. The clips are real
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
- [Keeping it out of places](#keeping-it-out-of-places)
- [The robot's self-description](#the-robots-self-description)
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
  **Reset layout**, **Follow robot**, **Edit world** (with a world editor in the profile),
  **Dock**, **Pause** or **Resume** while a mission runs (when the executor can pause one) and
  **Stop mission**, which is always there. A **Mission paused** chip shows while one is.
- **Chat, left:** Enter sends, Shift+Enter adds a line, Esc stops the reply.
- **Viewer, middle:** the 3D world (map, rooms, objects, the robot, where it walked and is about
  to walk), the cameras with what the detector marks, and below them the mission's steps over
  time, the agent's log and plots. It is one timeline you can scrub back.
- **Dock, right:** Mission, World, Layers, Approvals, Events, Agent, Doctor and Robot
  (Ctrl+1 to Ctrl+8). A waiting approval turns its tab amber. Doctor also drafts the robot's
  self-description.
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

### Pausing

https://github.com/user-attachments/assets/d6e4dc59-a2c3-4b54-87cb-25c1abc4c190

> **You:** Walk to the living room, room R1.
>
> **You:** Pause.
>
> **You:** Resume.

While a mission runs, a plain "pause" (or "wait", "hold on") pauses it without asking a model: the
robot stops where it is, the arms hold and the hands keep their grip, and a pick or a place
finishes first. "resume" (or "continue", "carry on", "go on") goes on: a walk to a place sets out
again, and a straight walk or a turn does only what was left of it. The window's **Pause** button
and the plan card's do the same, as in the clip: the card reads "Paused · started 12 s ago" and
the top bar shows **Mission paused**. Time spent paused is not counted in a step's time, and the
executor ends a mission paused for too long (ten minutes on grove-g1's). The words count only
while a mission runs and the profile names the executor's pause service (`pause` under
[`[mission]`](profile.md#mission)); otherwise they go to the model as any others.

### One step's answer in the next

https://github.com/user-attachments/assets/9b4cfc4f-5355-43bb-a4c8-de9aff755d4a

> **You:** Walk to the red mug, look for a mug there, and pick up the one you find with your left
> hand.

One mission walks, looks and picks. In the clip the plan check sent the first plan back: on this
demo robot a look is only for `mug` or `tray`, the names its mock detector knows, and the model
had planned it for "red mug".

Some skills hand on what they find. `FindInView` hands on `object_id`, the name the detector
reports an object under. A walk to what several objects are (`GoToPlace(place="red mug")`) hands
on `target_id`, the id of the nearest, and is allowed only when a later step takes it. The model
gives the step an id and takes its answer in a later step:

```json
{"steps": [
  {"id": "look", "skill": "FindInView", "args": {"phrase": "mug"}},
  {"skill": "PickObject", "args": {"object_id": "{look.object_id}", "arm": "right"}}]}
```

The plan is checked before it runs: the id (a letter, then letters, digits or `_`, at most 24)
must be an earlier step's that is not optional, the answer one that step gives, and of a kind the
argument takes. The plan card shows the id before its step: `look: FindInView(phrase=mug)`.

### Doing it again

- "Save that as the evening check" keeps a plan that worked; "Run the evening check" runs it
  again. The Mission tab lists saved plans.
- A schedule runs a plan on a clock or when something happens, approved once for every run
  (see [Keeping an eye on it](#keeping-an-eye-on-it)). Stopping the robot cancels every schedule.
- Requests no skill can do are logged as skill gaps, for whoever writes the next skill.

### Counting, looping and measuring

> **You:** Count the chairs in every room.

What takes a loop, a count or a measurement the agent writes as a short program in
[Rhai](https://rhai.rs), a small scripting language with no files, network or `eval`, and runs
as a job. Counting what the world model knows needs no walking:

```rhai
let n = #{};
for r in rooms() { n[r.name] = objects("chair", r.id).len(); }
n
```

The program's functions are the robot's: `objects(words, room)`, `rooms()`, `places()`,
`pose()`, `walk(place)`, `find(words)`, any skill by its name (`PickObject(#{object_id: id, arm:
"right"})`) or `run("Skill", #{...})`, `read(topic)`, `rate(topic, seconds)`, `tf(parent, child)`,
`wait(seconds)` and `say(text)`. One that only reads runs at once. One that moves is approved once,
with what it may run and for how long: "may move the base (GoToPlace) and look (FindInView), for
at most 10 min". It then runs on its own: the chat shows a card with its latest lines and a Stop
button, and its result comes back as a report the agent answers. Every mission a program sends is
checked like any plan, keep-out zones hold, and a skill the approval did not name is refused. A
program stops at its time limit, after a few seconds of computing between two robot calls, and
when the robot is stopped. "Keep that program as count chairs" keeps one by name; the Mission tab
lists the kept programs to run again, each approved again.

### Finding something

> **You:** Find my keys.

`search` ranks where to look: where the world model saw the thing and how long ago, where it went
since, then the rooms such things are usually in, the less seen of them first, nearer before
farther. The approval names the first places and why ("O7 (a mug was seen there 2 min ago), then
R2 (such things are often in the office; the camera has seen 30% of it)"). The search then walks
to each place and looks with the detector, up to four places in ten minutes, until it sees the
thing, and reports where. "Where could my keys be? Don't move" lists the places without moving,
and "where did you last see the red mug?" is a recall, answered at once.

### Teaching it a skill

> **You:** Point at the red mug with your right hand.

No skill points, so the agent teaches the robot one: a small tree of the leaves the executor lets
a taught skill use, such as `MoveHand` and `Wait`, with its own arguments:

```json
{"name": "PointAt", "args": [{"name": "object", "type": "world_id"}, {"name": "arm"}],
 "body": {"sequence": [{"leaf": "MoveHand", "arm": "{arm}", "target": "{object}", "mode": "point"},
                       {"leaf": "Wait", "seconds": 2}]}}
```

The executor checks it: only its learnable leaves and the plain control nodes, each value a
literal or one of the skill's arguments, within a time and size limit, with the authority its
leaves take. It is then a draft, and the plan that uses it comes to you as any plan does, its new
step marked as a taught skill on its first try. Once that mission succeeds the skill is kept, on
the robot, and the next "point at the tray" uses it. A taught skill that fails more often than
not over four runs is forgotten again. The Mission tab lists the taught skills, each with what it
was taught for and a Forget button.

## Clicking in the 3D view

https://github.com/user-attachments/assets/a6f03d25-e631-441d-940b-dda37a833435

Click a room, an object or a point on the floor in the 3D view: it lights up, and a bar above the
message box offers what the agent can do with it. **Go there** plans the walk at once, for you to approve;
the other buttons fill in a message for you to send. In the clip: the office, **Go there**; then
the desk, **What is it?**, which fills in "Tell me about O47 (desk)." to send:

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
| `/pause`, `/resume` | Pause the running mission, or let it go on |
| `/arm`, `/disarm` | Let the agent act, or only observe |
| `/explore` | Ask it to explore the building |
| `/compact` | Condense the conversation |
| `/follow` | Follow the robot in 3D, or see the whole map |
| `/layout` | Put the viewer's panes back |
| `/edit` | The world editor |
| `/live` | Back to the live view after **Show in 3D** |
| `/robot`, `/world`, `/approvals`, `/events`, `/layers` | Open that tab of the dock |
| `/doctor` | Check the setup again, and open Doctor |
| `/models`, `/missions` | Open Agent, or Mission |
| `/dock` | Show or hide the dock |

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
in the logs?". Nodes composed into a container are listed under it, `params` gives each
parameter's value with its description, and the read tools take `odom` for `/odom`. Acting through
ROS (a service call, an action goal, a parameter, a publish) needs the profile to allow it, an
absolute name, and your approval each time.

## When something goes wrong

> **You:** Why did that fail?
>
> **NervROS:** The robot is lost on the map: it stands 1.2 m and 91 degrees from where it thinks it
> is, so the walk ended in the wrong place. Set its pose again (2D Pose Estimate) or restart the
> stack before another walk; the same plan would fail the same way.

NervROS keeps a flight recorder of the robot's last five minutes: warnings and errors from
`/rosout`, the nodes that started and stopped, the missions, and the topics the profile watches.
When a mission fails, a watched topic goes quiet, a watched node stops or a node logs errors in a
burst, it opens an incident: the recording is frozen, and every playbook that may explain it runs
on the robot as it is. A playbook is what someone who debugged the robot wrote down: a few
read-only checks (a topic's rate or a field of its newest message, a line in the recording, a node
missing or running twice, a transform missing or two poses that disagree, the failure's own
words), and when they hold, the verdict and the fix. NervROS carries some that fit any robot, and
the profile adds its robot's own ([`[incidents]`](profile.md#incidents)).

An incident the robot opened by itself, with a likely cause, reaches the agent with your next
message, so "why can't it find anything?" starts from it. `why` tells the newest incident, or one
by its id or its mission's: the likely causes and what each
rests on, what was ruled out, and the minutes before it. Asked when nothing happened, it checks the
robot now, and "check the robot's health" runs the playbooks too. After a debugging session,
"remember this" keeps what was found as a playbook (`remember_finding`, approved as an edit), so
the next incident like it is named at once. The Doctor tab lists the incidents with their evidence,
**Check now** and **Ask the agent**, and a new one shows a toast. The investigator only reads: it
never moves the robot.

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

## Keeping it out of places

https://github.com/user-attachments/assets/696fc57c-44bb-4984-a8cc-5d2f2e68e7ce

> **You:** Keep the robot out of the bedroom.
>
> **You:** Go to the bedroom.
>
> **You:** Keep it half a metre away from the vase.

`geofence` adds a keep-out zone around a room's outline, an object (its size and a margin, 0.3 m
unless you say), a place, or corners you give on the map, and names it after what it surrounds.
From then on the agent refuses a walk that would end in a zone, a straight walk that would enter
one, and a navigation goal inside one. When the profile names a `mask` topic
([`[geofence]`](profile.md#geofence)), NervROS publishes the zones there for Nav2's keepout
filter, so Nav2 plans around them too, and `doctor` says whether anything reads it. Without it, a
walk whose route crosses a zone, driving by hand, and skills that move the base themselves
(exploring, a pick backing off) are not held back at a zone.

The 3D view draws the zones in red (the Layers tab can hide them), and the World tab lists them:
**Add** starts the request in the composer, and **Remove** asks the agent to remove one, which you
approve every time. Adding is refused while the robot stands inside the zone, and for a room type
several rooms share until you name one by its id. Zones added in chat are kept for the robot
across sessions, and those set in the profile stay until the profile changes. A plan made before a
zone was added is checked again when it runs, from a schedule or by its hash.

## The robot's self-description

https://github.com/user-attachments/assets/cdadbd00-b26b-4e14-a006-65d56f2dea02

The persona tells the model what the robot is. **Draft from the robot** in the Doctor tab, or
`nervros-cli whoami`, writes a new one from the robot as it is now: the executor's skills as the
planner offers them, the cameras and sensors on the graph, the stacks it runs, the URDF's body,
the rooms the world model knows and the named places. The routine model drafts it in the current
persona's style into `<persona>.draft.md` beside it (or `persona.draft.md` beside the profile);
the persona itself is never changed. `whoami --facts` prints what it gathered, without a model.

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
# In chat: /arm, /disarm, /yes N, /allow N, /no N, /again N, /stop, /pause, /resume, /compact, /quit
nervros-cli --profile my.toml chat
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
| `why`, `incidents` | What is wrong with the robot now, checked against the playbooks and kept as an incident; the incidents kept, or one in full |
| `missions`, `gaps` | The mission ledger, and the requests no skill could do |
| `replay` | A session as it happened |
| `eval`, `case` | Test suites against the live robot or a scripted one, and a session kept as a new case |
| `record` | The live robot written as a scripted robot for `eval --robot` |
| `whoami` | The robot's self-description drafted from the live robot into `<persona>.draft.md`; `--facts` prints what it gathered |
| `models`, `skills`, `mcp` | The model chains with today's counts, the skills, and the MCP tools to approve |

`doctor` checks the profile against the live graph, and `ros` runs one read tool with no model
(its output abridged here):

```text
$ nervros-cli --profile nervros.toml doctor
ok      674 interface definitions loaded
ok      tool find_objects -> /canopy/find_objects (canopy_msgs/srv/FindObjects)
ok      look image /chest_camera/color/image_raw
ok      look detections /detector_chest/instance_masks
ok      mission validate /nervros_executor/validate_mission
ok      mission catalog /nervros_executor/get_catalog
ok      mission stop /nervros_executor/stop_all
ok      mission pause /nervros_executor/pause_mission
ok      keep-out mask /keepout_filter_mask: Nav2 reads it

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
own pose. Each model and trial keeps its own robot files in the run's folder (keep-out zones,
missions, notes, places, programs, incidents), carried from one case to the next, so a suite
neither finds the operator's nor leaves its own behind. The apartment suite in grove-g1 has
46 cases:

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

That run was on the suite's first 44 cases.

`--only <text>` runs the cases whose ids hold it, `--repeat k` runs each case k times for pass^k,
`--models a,b` compares models, and `nervros-cli case last --id my-case --append suite.toml` keeps
a session as a new case. A suite is TOML: each `[[case]]` has an `id`, the `say` messages,
optional `setup` and `max_s`, and an `expect` table (`tools`, `not_tools`, `tool_said`, `skills`,
`mission`, `approvals_max`, `reply_has`, `reply_has_all`, `reply_lacks`, `moved_m`, `turned_deg`,
`compacted`); `scenarios/bench/suite.toml` is a small one.

`--robot <file>` runs a suite against a robot a file describes instead of the live graph. The
flag repeats, and a later file replaces an earlier one's entries key by key. A file scripts the
graph, the nodes each component container hosts, what each node publishes and subscribes to,
parameters with their descriptions, topic rates and messages, frames, and fixed answers for
services and actions; `scenarios/robot/robot.toml` has a mission executor that accepts every plan
and finishes every mission. Nothing moves and no camera gives an image, so it judges what the
model says and plans (a case that measures the robot's motion cannot pass there), and a model
comparison takes minutes with no simulator running; the model server must still be up.
`nervros-cli record robot.toml` writes one from the live robot: the graph, the containers, each
node's topics and services, the parameters (those `param_read_deny` hides written as
`(hidden)`), the rates and last messages of the topics the profile names, the robot's pose on the
map and the executor's catalog, but no images, clouds, maps or TF. `scenarios/bench` holds ten
generic tasks on a generic Nav2 robot:

```text
$ nervros-cli --profile scenarios/bench/nervros.toml eval scenarios/bench/suite.toml \
    --robot scenarios/robot/robot.toml --robot scenarios/bench/robot.toml --models qwen3.5-9b-local
...
10 of 10 trials passed; report ~/.local/state/nervros/evals/.../report.md
```

## Keys

| Key | |
|---|---|
| Enter, Shift+Enter | Send; a new line |
| Esc | Stop the reply |
| Ctrl+Shift+S | Stop the mission |
| Ctrl+K | The command palette |
| Ctrl+1 to Ctrl+8 | The dock's tabs |
| W S, A D, Q E | Drive by hand: walk, turn, step sideways (Robot tab, Drive on) |
