# Using NervROS

This guide covers every way to use NervROS. Most sections open with a short clip from a real run
and show the exact words typed.

The clips show a simulated Unitree G1 in a six-room apartment
([grove-g1](https://github.com/Adyansh04/grove-g1)). The local Qwen3.5-9B model answers unless a
section says otherwise. Missions play three or four times faster, and the 23-minute mapping run
keeps only its start and end. Everything else is real time.

Setup is in the [README](../README.md). Every setting is in the [profile](profile.md),
[models](models.md) and [executor](executor.md) pages.

- **Start:** [The window](#the-window)
- **Ask:** [Asking what it sees](#asking-what-it-sees) ·
  [Pointing and outlining](#pointing-and-outlining)
- **[Give it a task](#giving-it-a-task):** [Walking somewhere](#walking-somewhere) ·
  [The plan check](#a-short-one-and-the-plan-check) · [Fetching something](#fetching-something) ·
  [Stopping](#stopping) · [Pausing](#pausing) ·
  [One step's answer in the next](#one-steps-answer-in-the-next) ·
  [Doing it again](#doing-it-again)
- **Programs and skills:** [Counting, looping and measuring](#counting-looping-and-measuring) ·
  [Finding something](#finding-something) · [Teaching it a skill](#teaching-it-a-skill)
- **By hand:** [Clicking in the 3D view](#clicking-in-the-3d-view) ·
  [Commands without the model](#commands-without-the-model) · [Driving by hand](#driving-by-hand)
- **Watching and fixing:** [Keeping an eye on it](#keeping-an-eye-on-it) ·
  [Waiting for what a camera shows](#waiting-for-what-a-camera-shows) ·
  [A patrol and the digest](#a-patrol-and-the-digest) · [Proposed fixes](#proposed-fixes) ·
  [Debugging the ROS side](#debugging-the-ros-side) ·
  [When something goes wrong](#when-something-goes-wrong) ·
  [Runs the robot cut short](#runs-the-robot-cut-short)
- **Memory and limits:** [Remembering](#remembering) · [Keeping a goal](#keeping-a-goal) ·
  [Keeping it out of places](#keeping-it-out-of-places) · [Nav2 and MoveIt](#nav2-and-moveit) ·
  [The robot's self-description](#the-robots-self-description)
- **Maps:** [Mapping a new building](#mapping-a-new-building) ·
  [Finishing the map](#finishing-the-map)
- **Reference:** [Sessions and models](#sessions-and-models) ·
  [The command line](#the-command-line) · [Evals](#evals) · [Keys](#keys)

## The window

```bash
source scripts/ros-env.sh
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

![The window: chat, 3D world and cameras, dock](https://github.com/user-attachments/assets/d0e52596-0a56-49af-a0d3-42b28c34d7ef)

- **Top bar:** the robot, and the state of ROS, the mission executor and the model. On the right:
  the arming switch (until you arm it, the agent can look and answer but not act),
  **Reset layout**, **Follow robot**, **Edit world** (when the profile has a world editor),
  **Dock**, **Pause** or **Resume** while a mission runs (if the executor can pause one), and
  **Stop mission**, which is always there. A **Mission paused** chip shows while a mission is
  paused.
- **Chat, left:** Enter sends, Shift+Enter adds a line, and Esc stops the reply.
- **Viewer, middle:** the 3D world (map, rooms, objects, the robot, where it walked and where it
  is about to walk), the cameras with the detector's marks, and below them the mission's steps
  over time, the agent's log and plots. It is one timeline you can scrub back.
- **Dock, right:** Mission, Tree, Approvals, World, Robot, Layers, Library, Events, Agent and
  Doctor (Ctrl+1 to Ctrl+0). A dot marks a tab that wants a look: amber on Approvals while a
  request waits, green on Tree while a mission runs, red on Doctor when a check failed. Doctor
  also drafts the robot's self-description.
- **Status bar:** whether the agent is working, how full the model's context is, and how much of
  it came from the model server's cache.

## Asking what it sees

https://github.com/user-attachments/assets/ea9c76e9-0ab1-49e5-8326-06d29deee212

> **You:** What can you see right now?
>
> **NervROS:** I can see a red wardrobe (mark 1) with two doors, positioned against a plain wall
> on a wooden floor.

`look` takes the newest camera frame and draws each detection as a numbered mark, so you and the
model can both refer to things by number. Under the image, **Show in 3D** pauses the viewer at the
moment the snapshot was taken, and **Ask about it** starts a message about it.

> **You:** Where is the small white mug?
>
> **NervROS:** I found the small white mug (O783) in room R1. It's located at position x=-1.91,
> y=2.67 meters with a height of 0.78 meters. Would you like me to pick it up?

`find_objects` searches the world model, which remembers everything the cameras have seen, room
by room.

> **You:** Which rooms do you know, and how much of each have you seen?

`list_places` answers with each room's type, how much of its floor and walls the camera has seen,
and how many objects it holds. Then it lists the named places.

### Pointing and outlining

https://github.com/user-attachments/assets/520b0f75-001f-45a2-a9c7-51a8ac6cbfc0

> **You:** Point at the tray.
>
> **NervROS:** I've pointed at the tray. You can see it marked as mark 4 in the image.
>
> **You:** Outline the desk.
>
> **NervROS:** I've outlined the desk in the image. The desk is marked as region 1 and covers
> about 11.5% of the frame.

`point` asks a vision model, so it finds things the detector never marked, and it draws a ring
where it points. `segment` outlines what you name, using a vision model or the robot's own
segmentation service.

## Giving it a task

Anything that moves the robot is a mission. The agent writes a plan from the robot's own skills,
and NervROS compiles it to a behaviour tree. The executor checks the tree, and only then does the
plan reach you, as a card to approve.

### Walking somewhere

https://github.com/user-attachments/assets/7cf1cd95-a8dc-4519-a4cf-155976221887

> **You:** Walk to the bedroom, room R3.

The plan card lists each step with its track record on this robot ("13 of 15 · 24 s"), and the
viewer draws the planned path. While the mission runs, each step on the card shows its live state,
the Tree tab draws the behaviour tree step by step with the nodes running inside each, and the
timeline strip shows the steps over time. When it ends, the agent gets a report with its goal checks:

```text
report> Mission 01a0feba (Walk to the bedroom, room R3) ended: success after 24 s.
        Goal checks: at(R3) holds (the robot is in R3).
```

### A short one, and the plan check

https://github.com/user-attachments/assets/88c9effd-38af-4ce4-96c6-d5887d5ca9fb

> **You:** Turn left 90 degrees.

NervROS checks every plan against your words: left or right, forward or back, how far, how much,
which hand. Here the model got the sign right. In an earlier run its first plan was
`TurnInPlace(degrees=-90)`, a right turn, and the check sent it back before it reached the card:

```text
run_mission failed s1: the operator said turn left, but degrees=-90 turns the other way
(positive degrees turn left). The plan may not do what the operator asked: fix it, or if it is
right, send it again unchanged and the operator sees these concerns when approving
```

If the model sends the same plan again, the concern stays on the card with a one-click fix.
**Edit** changes, drops or moves a step before you approve.

https://github.com/user-attachments/assets/91718cec-719c-4fad-96ff-6c210f32c287

> **You:** Look for the wooden tray from here with FindInView, without walking.

A plan with a step that has worked at most half the time on this robot, over two runs or more, also
goes to the scene check: a vision model shown what each camera sees now. It can only add doubts.
Here `FindInView` for the tray had worked once in five, and the doubt "there is no tray in the
current view" shows under the step and in the approval, before anything moves. Gemini 3.5
Flash-Lite answered in this clip, and Gemini Robotics-ER 2 made the scene check.

### Fetching something

https://github.com/user-attachments/assets/0c9381e1-ae1e-4391-a63d-53ef5df6179c

> **You:** Bring the small white mug from the dining table to the tray on the office desk.

The plan has four steps, and the card tracks each one live while the arms work. The clip plays
four times faster while the mission runs.

```text
plan: GoToPlace(place=dining_table_side) -> PickObject(object_id=mug_4, arm=left)
      -> GoToPlace(place=office_desk_tray) -> PlaceInto(container_id=tray_1, arm=left)
```

Under three minutes later the report comes back with its checks. Only the arm's detector knows
these two objects, so the world model could not judge "inside", and the camera checked instead:
the detector marks a mug inside the tray's box.

```text
report> Mission 01a0ff3c (Bring the small white mug from the dining table to the tray on the
        office desk) ended: success after 170 s. Camera check: inside(mug_4, tray_1): the
        camera agrees: the detector marks a mug (mark 1) in or on the tray (mark 5).
```

> **NervROS:** The mission was successful! The small white mug has been brought from the dining
> table to the tray on the office desk.

The Tree tab draws the behaviour tree as it runs: each step a card on a spine with its wrappers
(time limit, retries, optional), its live state and the nodes inside it with their times. The
Library tab lists the missions before it and each step's track record.

A failed mission comes back with the step that failed, why it failed, and what the world model saw
of the object involved. The agent then proposes a changed plan for you to approve. You can keep
talking while a mission runs.

### Stopping

https://github.com/user-attachments/assets/d3111b11-fab2-429c-a8fb-b387522f74d1

> **You:** Walk to the office, room R4.
>
> **You:** Stop.

A plain "stop" (or "halt", "freeze", "stop!") never waits for a model. It stops the reply and the
robot at once, through the executor's `StopAll`. In the clip the walk had run for seven seconds:

```text
[halted: the operator stopped the reply and the robot]
[the robot stopped: holding posture; hands empty]
report> Mission 01a0febc (Walk to the office, room R4) ended: canceled after 9 s.
        It was stopped during s1 GoToPlace(place=R4) (stopped: operator).
```

- Esc stops only the reply. **Stop mission** or Ctrl+Shift+S stops the robot.
- A stop never opens a hand: a robot holding a mug keeps it.
- While a mission runs, the window sends the executor a heartbeat, so a mission whose window closed
  or hung stops by itself.

### Pausing

https://github.com/user-attachments/assets/7bf2d1ef-5c5e-4873-b48f-7222fd3dc6f3

> **You:** Walk to the living room, room R1.
>
> **You:** Pause.
>
> **You:** Resume.

While a mission runs, a plain "pause" (or "wait", "hold on") pauses it without asking a model.
The robot stops where it is, the arms hold and the hands keep their grip; a pick or a place in
progress finishes first. "Resume" (or "continue", "carry on", "go on") lets it go on. A walk to a
place sets out again, and a straight walk or a turn does only what was left of it.

The window's **Pause** button and the plan card's do the same, as in the clip: the card reads
"Paused · started 12 s ago" and the top bar shows **Mission paused**.

<details>
<summary>The rules</summary>

- Time spent paused does not count toward a step's time.
- The executor ends a mission that stays paused too long (ten minutes on grove-g1).
- The words work only while a mission runs and the profile names the executor's pause service
  (`pause` under [`[mission]`](profile.md#mission)). Otherwise they go to the model like any
  other message, except a plain "pause" or "resume": the executor answers those itself, and says
  so when nothing is running.

</details>

### One step's answer in the next

https://github.com/user-attachments/assets/99be5972-bc7b-457b-ae5c-feb447bf00e3

> **You:** Walk to the red mug, look for a mug there, and pick up the one you find with your left
> hand.

One mission walks, looks and picks. In the clip the plan check sent the first plan back: on this
demo robot a look works only for `mug` or `tray`, the names its mock detector knows, and the
model had planned one for "red mug".

Some skills hand on what they find:

- `FindInView` hands on `object_id`, the name the detector reports an object under.
- A walk to a description that several objects fit (`GoToPlace(place="red mug")`) hands on
  `target_id`, the id of the nearest one. It is allowed only when a later step uses it.

The model gives the step an id and uses its answer in a later step:

```json
{"steps": [
  {"id": "look", "skill": "FindInView", "args": {"phrase": "mug"}},
  {"skill": "PickObject", "args": {"object_id": "{look.object_id}", "arm": "right"}}]}
```

The plan card shows the id before its step: `look: FindInView(phrase=mug)`.

<details>
<summary>What the plan check requires</summary>

The id is a letter, then letters, digits or `_`, at most 24 characters. It must belong to an
earlier step that is not optional, the answer must be one that step gives, and it must be of a
kind the argument takes.

</details>

### Doing it again

- "Save that as the evening check" keeps a plan that worked, and "Run the evening check" runs it
  again. The Library tab lists saved plans.
- A schedule runs a plan on a clock or when something happens, approved once for every run (see
  [Keeping an eye on it](#keeping-an-eye-on-it)). Stopping the robot cancels every schedule.
- Requests that no skill can do are logged as skill gaps, for whoever writes the next skill.

### Counting, looping and measuring

https://github.com/user-attachments/assets/c46fbd2f-89fd-4069-83ee-c0dcf27b8c6f

> **You:** Count the chairs in every room.
>
> **NervROS:** There are 5 chairs in the living room, 1 in the office, and 1 in the study. All
> other rooms currently have no chairs recorded.

For anything that takes a loop, a count or a measurement, the agent writes a short program in
[Rhai](https://rhai.rs) and runs it as a job. Rhai is a small scripting language with no files,
network or `eval`. Counting what the world model knows needs no walking, so this one ran at once:

```rhai
let n = #{}; for r in rooms() { n[r.type] = objects("chair", r.id).len(); } n
```

> **You:** Run a program that looks for a mug in the office, room R4, then at the dining table,
> and stops at the first place where the camera sees one.

```rhai
let found = false;
let places_to_check = ["office", "dining_table"];
for p in places_to_check {
  if walk(p) {
    if find("mug") != () {
      say("Found a mug!");
      found = true;
      break;
    }
  }
}
found
```

A program that moves is approved once, with what it may run and for how long: "it may move the
base (GoToPlace) and look (FindInView), for at most 5 min". It then runs on its own. The chat
shows a card with its latest lines and a **Stop** button, and its result comes back as a report
the agent answers.

In the clip the look in the office saw no mug, so the program walked on to the dining table, saw
one there and ended on `true`. A mission would have ended at the empty look. Gemini 3.5 Flash-Lite
answered in this clip.

The program's functions are the robot's: `objects(words, room)`, `rooms()`, `places()`, `pose()`,
`walk(place)`, `find(words)`, any skill by its name
(`PickObject(#{object_id: id, arm: "right"})`) or `run("Skill", #{...})`, `read(topic)`,
`rate(topic, seconds)`, `tf(parent, child)`, `wait(seconds)` and `say(text)`.

"Keep that program as count chairs" keeps it by name. The Library tab lists kept programs to run
again, and each run is approved again.

<details>
<summary>How a running program is held to the rules</summary>

- Every mission a program sends is checked like any plan, and keep-out zones and the profile's
  rules hold. A rule that would ask refuses instead, since a job cannot stop to ask, and a skill
  the approval did not name is refused.
- A program waits for a mission that is already running. It stops at its time limit, after a few
  seconds of computing between two robot calls, and when the robot is stopped.
- Its **Stop** button and its time limit cancel the mission it runs, and nothing else the robot
  does.
- A program that ends on `()` reports that it found nothing, with the last thing it said.

</details>

### Finding something

https://github.com/user-attachments/assets/51157255-5ddb-487b-b77f-5181f9481dd7

> **You:** Where could my keys be? Don't move.
>
> **NervROS:** Based on what the world model knows, keys are most likely in these places (best
> first):
> - R8 (hallway) - 37% likely
> - R1 (living room) - 26% likely
> - R3 (bedroom) - 19% likely
> - R4 (unexplored room) - 9% likely
>
> The robot can't search for keys itself since the detector only looks for mug and tray. …

The world model has never seen keys, so the ranking is by the rooms keys are usually in. Nothing
moves. The demo's mock detector knows only mugs and trays, so the robot cannot go and look for
keys itself.

> **You:** Find the red mug.

`search` ranks where to look:

1. where the world model saw the thing, and how long ago,
2. where it went since,
3. the rooms such things are usually in, the least seen first, nearer before farther.

The approval names the first places and why: "O46 (a mug was seen there 6 min ago), then O783 (a
mug was seen there 89 s ago)", and so on. The search walks to each place and looks with the
detector, up to four places in ten minutes, until it sees the thing, and reports where. In the
clip it saw the red mug at the first place.

"Where did you last see the red mug?" is a recall, answered at once.

### Teaching it a skill

https://github.com/user-attachments/assets/4e1bf457-9f63-498c-b293-d18661542491

> **You:** Point at the red mug with your right hand.
>
> **NervROS:** I have started a mission to walk to the red mug (mark O27) and point at it with
> your right hand.

No skill points, so the agent teaches the robot one. A taught skill is a small tree of the leaves
the executor allows, such as `MoveHand` and `Wait`, with its own arguments. In the clip it is a
single leaf:

```json
{"name": "PointAt", "description": "Point at an object with the specified arm",
 "args": [{"name": "object", "type": "world_id", "description": "The object to point at"},
          {"name": "arm", "type": "string", "description": "The arm to use (right or left)"}],
 "body": {"leaf": "MoveHand", "target": "{object}", "arm": "{arm}", "mode": "point"}}
```

A body can also nest `sequence`, `fallback`, `retry`, `timeout_s` and `force_success` nodes.

You approve the teaching first. The executor checks the skill, which is then a draft. The plan
that uses it comes to you like any plan, with its new step marked as a taught skill on its first
try. Once that mission succeeds, the skill is kept on the robot, and the next "point at the tray"
uses it. The Library tab lists the taught skills, each with what it was taught for and a
**Forget** button. Gemini 3.5 Flash-Lite answered in this clip.

<details>
<summary>What the executor accepts, and when a skill is forgotten</summary>

- Only its learnable leaves and the plain control nodes, each value a literal or one of the
  skill's arguments, within a time and size limit, and with the authority its leaves take.
- A taught skill that fails more often than not over four runs is forgotten again.

</details>

## Clicking in the 3D view

https://github.com/user-attachments/assets/bfe51e79-41ec-4697-a017-dfaefe0bfd26

Click a room, an object or a point on the floor in the 3D view. It lights up, and a bar above the
message box offers what the agent can do with it. **Go there** plans the walk at once, for you to
approve; the other buttons fill in a message for you to send. In the clip: the office, then
**Go there**; then the desk, then **What is it?**, which fills in "Tell me about O47 (desk)." to
send:

> **NervROS:** O47 is a large grey metal desk with multiple drawers on both sides. It measures 2m
> x 0.95m and is about 0.88m tall.

**Follow robot** in the top bar keeps the 3D view on the robot as it walks. With it off, the view
frames the whole map.

## Commands without the model

https://github.com/user-attachments/assets/e8cce4e9-8e55-47d8-9585-80c6ae21a2d4

Ctrl+K opens the command palette: type a few letters and press Enter. Every command also works as
a `/word` typed in the chat. Most need no model: they open a tab, change the view or stop the
robot. `/look`, `/places` and `/explore` send their request to the agent. In the clip: Ctrl+K,
"follow", Enter; then `/layers` and `/doctor`.

| Command | Does |
|---|---|
| `/look` | Asks the agent to look through the camera |
| `/places` | Asks which places it can go to |
| `/stop` | Stops the robot |
| `/pause`, `/resume` | Pauses the running mission, or lets it go on |
| `/arm`, `/disarm` | Lets the agent act, or only observe |
| `/explore` | Asks it to explore the building |
| `/compact` | Condenses the conversation |
| `/follow` | Follows the robot in 3D, or shows the whole map |
| `/layout` | Puts the viewer's panes back |
| `/edit` | Opens the world editor |
| `/live` | Goes back to the live view after **Show in 3D** |
| `/robot`, `/world`, `/tree`, `/approvals`, `/events`, `/layers` | Opens that tab of the dock |
| `/doctor` | Checks the setup again and opens Doctor |
| `/models`, `/missions` | Opens Agent, or Library |
| `/dock` | Shows or hides the dock |

## Driving by hand

https://github.com/user-attachments/assets/e6edf6ef-67c4-441b-a0b9-3381bf94d3ed

The Robot tab shows whether the robot is upright, where it is and in which room, what each hand
holds, and its motors and battery when the robot reports them.

**Drive** asks the executor for the base: W and S walk, A and D turn, and Q and E step sideways.
The pad lights the key you hold, and **Slow** halves the speed. The executor refuses while a
mission runs, and it stops the base when the keys stop, so a closed window stops the robot too.
Driving needs the robot armed.

## Keeping an eye on it

https://github.com/user-attachments/assets/d5717403-34a7-4bb5-bd64-cc8ebdc6eeef

> **You:** Tell me if the chest camera's images drop below 5 Hz.
>
> **NervROS:** I'm now watching the chest camera's image rate. I'll let you know if it drops below
> 5 Hz within the next 30 minutes.

A watch runs in the background and posts to the chat when its condition holds: a rate dropping, a
value crossing a line, or a log line matching.

> **You:** Plot the robot's forward speed for the next two minutes.

The viewer brings its **Plots** tab forward and draws the robot's speed live: still at first, then
the scheduled walk below.

> **You:** Every ten minutes, walk to the bedroom, room R3, three times.

A schedule is approved once, runs now and then on its clock, and reports after each run:

```text
report> Mission 01a0ff18 (Walk to bedroom R3 every 10 minutes, 3 times, run 1 of 3 of schedule
        p1) ended: success after 18 s. Goal checks: at(R3) holds (the robot is in R3).
```

The Mission tab lists the schedule with a **Cancel** button. A schedule can also wait for
something: an object of a kind appearing, or a condition on a topic. With nothing to run, it only
tells you: "Tell me when a mug appears in the bedroom" needs no approval.

"Check the robot's health" runs the same connection check as the Doctor tab. You can ask it even
while a mission runs.

### Waiting for what a camera shows

https://github.com/user-attachments/assets/1ebe06cc-de7a-490f-b902-597ea91f72d6

> **You:** Tell me when the head camera shows a red mug.
>
> **You:** Walk to the dining table.

A watch can also wait for what a camera shows, such as someone putting a box on the sofa. A vision
model looks every 15 seconds, and the watch posts once it says yes twice in a row, so one glimpse it
mistook does not count. Here it fires once the robot stands at the table: "the camera model says a
red mug on the table".

> **You:** When did the head camera first show the red mug?

With `[keyframes]`, the cameras keep a frame each time their view changes. `when` shows a vision
model a dozen of them in time order, those around the moments the world model saw the mug change
when it did, as for "when did the red mug leave the dining table?". A model reading a dozen frames
can name the one beside the right one, so the vision check then looks at that frame alone, and at
its neighbours until one shows it. It answers with the time and the frame: here 60 s ago, the frame
where the mug comes into view, and **Show in 3D** takes the viewer back to that moment.

### A patrol and the digest

https://github.com/user-attachments/assets/e0343475-e62f-4f6a-8e6a-5a693125325d

> **You:** Check on the living room and the bedroom.

A patrol walks the round as a job and looks around in each place: a full turn on the spot in
quarter turns, each view held still so the world model takes it in. It then reports what it saw in
each room and what moved or went missing meanwhile:

```text
report> The patrol j1 (living room, bedroom) finished: visited living room, bedroom; saw the
        living room (R1): 4 chairs, an armchair, a book, a bookshelf, a coffee table, a dining
        table and a sofa; the bedroom (R3): a bed, a clock, a nightstand and a potted plant, and
        in view the study (R7): a wardrobe; nothing moved or went missing in the world model
        meanwhile
```

> **You:** How did today go?

The digest counts the missions by how they ended, sets the runs the robot itself cut short apart,
gives each skill's record, and lists what moved in the world model, the incidents and the goals.
`nervros-cli digest --hours 24` prints the same in a terminal. Gemini 3.5 Flash-Lite answered in
this clip.

### Proposed fixes

https://github.com/user-attachments/assets/c23c8e5b-65c4-4367-88e8-5c94d21a6e42

When a step keeps failing, NervROS may propose a lasting fix: a note the planner reads, a keep-out
zone, or a node parameter the profile lets `param_set` change. It never applies one itself. The
Agent tab's **Proposed fixes** list each with what failed and why the fix would help, then
**Accept**, which runs it as yours, and **Dismiss**, which keeps that step from getting another.
Here Accept on the note adds it to **Memory**, and the fix then shows the step's record before and
since, which says whether it helped.

### Debugging the ROS side

With a `[ros_tools]` table in the profile, the agent reads the graph as `ros2` does: "what is
publishing the map?", "is the chest camera dropping frames?", "show me the controller's
parameters", "any errors in the logs?".

- Nodes composed into a container are listed under it.
- `params` gives each parameter's value with its description.
- The read tools take `odom` for `/odom`.

Acting through ROS (a service call, an action goal, a parameter change or a publish) needs the
profile to allow it, an absolute name, and your approval. A call or a publish may move the robot,
so it asks every time; a parameter change can be allowed for the rest of the session
(**Allow for session**).

## When something goes wrong

https://github.com/user-attachments/assets/eb499b8c-4e19-487d-939e-51b98aa5eefa

> **You:** Why can't the robot find anything with its head camera?
>
> **NervROS:** The robot's head camera detector (`detector_head`) has stopped running, so it
> can't detect objects through that camera. The chest camera's detector (`detector_chest`) is
> still working. …

The detector had been stopped for the clip, and the incident had opened by itself when the
detector left the graph. The answer went on to say that the operator has to start
`world_model.launch.py` again, and the Doctor tab shows the same incident with its evidence.

NervROS keeps a flight recorder of the robot's last five minutes: warnings and errors from
`/rosout`, the nodes that started and stopped, the missions, and the topics the profile watches.

An incident opens when a mission fails, a watched topic goes quiet, a watched node stops, or a node
logs a burst of errors. A failed step in a program's or a search's mission, such as a look that saw
nothing, is the job's answer; only the job's timeout or error opens one. The recording is then
frozen, and every playbook that may explain the incident runs against the robot as it is.

A playbook is what someone who debugged the robot wrote down: a few read-only checks and, when
they hold, the verdict and the fix. A check can look at a topic's rate or a field of its newest
message, a line in the recording, a node that is missing or running twice, a transform that is
missing or two poses that disagree, or the failure's own words. NervROS carries playbooks that
fit any robot, and the profile adds its own robot's ([`[incidents]`](profile.md#incidents)).

- An incident the robot opened by itself, with a likely cause, reaches the agent with your next
  message, so "why can't it find anything?" starts from it.
- `why` tells the newest incident, or one by its id or its mission's: the likely causes and what
  each rests on, what was ruled out, and the minutes before it. Asked when nothing happened, it
  checks the robot now. "Check the robot's health" runs the playbooks too.
- After a debugging session, "remember this" keeps what was found as a playbook
  (`remember_finding`, approved as an edit), so the next incident like it is named at once.
- The Doctor tab lists the incidents with their evidence, the newest one open, with **Check now**
  and **Ask the agent**. A new incident shows a toast.

The investigator only reads: it never moves the robot.

### Runs the robot cut short

https://github.com/user-attachments/assets/3c3d470d-7af2-498e-b83c-8c70945c5c65

Not every failed mission is a failed plan. The executor's result says when the robot or the executor
ended one: the robot could not move or fell, the app's heartbeat stopped, the executor was shutting
down, a skill's server was not up, the base was being driven by hand, or another mission held the
robot. NervROS adds two of its own: it lost the executor, or it stopped while the mission ran, which
the next start says. In the clip, a walk NervROS was running when it stopped is marked at start, and
a walk approved while the base is driven by hand from the Robot tab cannot start:

```text
report> Mission 01a149ab (Walk to the office, room R4) ended: rejected after 0 s. It could not
        start (teleop is driving the base).
```

Such a run counts against no skill's record and no goal, its report says what it needs instead of a
new plan ("do not retry until the operator ends teleop"), and the Mission tab dims it, with its
cause beside how it ended.

## Remembering

https://github.com/user-attachments/assets/4e948469-2f78-470e-b5e3-1755cc7346aa

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

## Keeping a goal

https://github.com/user-attachments/assets/2cbd5b3b-22d3-4f12-84ce-f697941a21d9

> **You:** Set a goal to keep the red mug on the wooden tray, then work toward it.

The agent sets a goal with what holds once it is met, in the world model's ids, such as `inside(O27,
O261)`, and you approve it like an edit. A name only the detector knows, such as `mug_4`, is taken
as the world model's likeliest object of its kind, and the agent is told which; with none it is
refused, as the world model could never see that goal met. The agent then plans a mission for it at
once. After every mission NervROS checks the goal itself and tells the agent what is still unmet,
and the agent plans the next mission for it, which you approve as usual. The world model may give an
object it saw moved another id, so for `inside` and `on` an object of the same kind seen there after
the goal's own object was last seen counts, once that object is gone from where it was or when the
one there was seen after the goal was set: a mug on the tray after the mug left the table is the
goal's mug. A goal ends when it holds, when its missions (five unless you say) or its time run out,
or when the agent gives it up as impossible, with the reason. A run the robot cut short counts
against no goal. Goals outlast a restart: at start the window says where each one stands.

The clip plays four times faster while the mission runs. Its report ends with the goal, met:

```text
report> Mission 01a14b19 (Pick up the red mug and place it into the wooden tray) ended: success
        after 211 s. … Camera check: inside(mug_4, tray_1): the camera agrees: the detector marks
        a mug (mark 1) in or on the tray (mark 5).
        Goal: goal 1 (keep the red mug on the wooden tray): met.
```

The Agent tab lists the goals and where each stands, each with **Drop**. Gemini 3.5 Flash-Lite
answered in this clip.

## Keeping it out of places

https://github.com/user-attachments/assets/3df05f4a-344a-4d54-a824-6ecfa8bebf57

> **You:** Keep the robot out of the bedroom.
>
> **You:** Go to the bedroom.
>
> **You:** Keep it half a metre away from the vase.

`geofence` adds a keep-out zone around a room's outline, an object (its size plus a margin, 0.3 m
unless you say otherwise), a place, or corners you give on the map. It names the zone after what
it surrounds. From then on the agent refuses a walk that would end in a zone, a straight walk that
would enter one, and a navigation goal inside one.

A slow zone lets the robot in, no faster than a share of its top speed:

> **You:** Make the corridor a slow zone at 30%.

Slowing a zone down is free; removing one or letting the robot go faster in it asks you first.

The 3D view draws keep-out zones in red and slow zones in amber with their speed (the Layers tab
can hide them), and the World tab lists them. **Keep out** and **Slow zone** start the request in
the composer, and **Remove** asks the agent to remove one, which you approve.

<details>
<summary>Nav2, and what a zone does not hold back</summary>

- With [`[nav2]`](profile.md#nav2), NervROS publishes the keep-out zones for Nav2's keepout
  filter, so Nav2 plans around them too, and the slow zones for its speed filter, which slows the
  robot inside them; `doctor` says whether anything reads them. Without it, a few things are not
  held back at a keep-out zone: a walk whose route crosses one, driving by hand, and skills that
  move the base themselves (exploring, a pick backing off).
- Adding a zone is refused while the robot stands inside it, and for a room type that several
  rooms share until you name one by its id.
- Zones added in chat are kept for the robot across sessions. Zones set in the profile stay until
  the profile changes.
- A plan made before a zone was added is checked again when it runs, from a schedule or by its
  hash.

</details>

## Nav2 and MoveIt

https://github.com/user-attachments/assets/43ac126e-f920-4fce-8a64-9ac0baea3062

https://github.com/user-attachments/assets/3091295f-aee5-4a00-8305-f9cb5ecc8e03

With [`[nav2]`](profile.md#nav2) and [`[moveit]`](profile.md#moveit) in the profile, the agent
works the robot's navigation and arm stacks directly: it reads them, changes how they behave, and
runs their standard motions itself where the robot's executor has no skill for them.

> **You:** Is navigation up, and how well does the robot know where it is?
>
> **You:** Could you walk to the office desk tray from here? Don't move.
>
> **You:** Shape the collision monitor's slow polygon as a box from 0.3 to 1.2 metres ahead.
>
> **You:** Can the right hand reach the red mug from here?
>
> **You:** Walk through the hallway to the office desk tray without stopping on the way.
>
> **You:** Raise your right hand 5 centimetres.

| Ask about | Tool | What changes, and what asks you first |
|---|---|---|
| Navigation's state, a path's length and the zones it crosses, the cost at a spot | `nav_status`, `nav_check` | Nothing. A path check waits while Nav2 walks, since it would cut into Nav2's own planning |
| The collision monitor: on or off, each polygon, a polygon's shape | `collision_monitor` | Turning on and growing are free; turning off and shrinking ask |
| Costmaps, localization, the navigation servers, maps | `nav_admin` | Each asks, and none runs during a mission |
| The arms: where each is, its named poses, the planners | `arm_status` | Nothing |
| Whether a hand reaches something, whether a pose is clear, the path an arm would take | `arm_check` | Nothing; the path is drawn in the 3D view |
| The obstacles the arms plan around, attaching an object to a hand, the octomap | `scene` | Adding is free; the rest asks |
| How fast the arm moves NervROS runs go, the planner, poses remembered by name | `arm_settings` | Slower is free; faster and another planner ask |

Plans may hold motions NervROS runs itself beside the executor's skills: a route through places
without stopping at each (`WalkRoute`), a wait, a short straight hand move (`HandLine`), and, on a
robot whose executor lacks them, walks, turns, arm poses and grips. The plan card marks such a
step **NervROS runs it through Nav2** (or MoveIt), its preview shows where a walk or a route goes,
and the 3D view draws the way while it runs. Stop and Pause work as for any step: a stopped step
ends as stopped, a paused walk stands and goes on with what was left when you resume, and an arm
move finishes first. If NervROS goes quiet with one of its motions running, a watchdog on the
robot cancels it within two seconds. Where the robot's executor holds the arms, it lends them for
the step and takes them back after it.

The Robot tab shows a card for each stack: whether it is up, how well the robot is localized,
what Nav2 is doing and the speed limit in force, and where each arm is.

<details>
<summary>What to watch for</summary>

- On the G1, a speed below about 0.15 m/s does not move the legs at all. A slow zone at 30% and
  the collision monitor's slow polygon together come below it, and the robot stands still until
  one of them lets go.
- A route's places are its waypoints: one inside furniture fails the whole route, so name places
  or rooms with room to stand.
- Grip never opens a hand that holds something: put the object down with a place.

</details>

## The robot's self-description

https://github.com/user-attachments/assets/32d93917-9227-47be-83e7-5b408e125657

The persona tells the model what the robot is. **Draft from the robot** in the Doctor tab, or
`nervros-cli whoami`, writes a new one from the robot as it is now: the executor's skills as the
planner offers them, the cameras and sensors on the graph, the stacks it runs, its body from the
URDF, the rooms the world model knows, and the named places.

The routine model drafts it in the current persona's style, into `<persona>.draft.md` beside it
(or `persona.draft.md` beside the profile). The persona itself is never changed.
`whoami --facts` prints what it gathered, without a model.

## Mapping a new building

https://github.com/user-attachments/assets/7c772eab-a4a0-4b78-a464-bbe55f937ab0

> **You:** Explore the building to fill in the map.

In a building it has never seen, the robot maps as it explores. SLAM draws the floor plan, the
world model splits it into rooms and types each room from the objects it finds there, and the
viewer shades in orange what the camera has yet to see. The plan is one step,
`ExploreBuilding()`, approved once. The clip shows the first four minutes and the last one, four
times faster:

```text
report> Mission 01a0fe14 (Explore the building to fill in the map) ended: success after 1380 s.
```

In 23 minutes the robot found six rooms and 45 objects. The World tab shows how much of each room's
floor and walls the camera has seen, and its **Explore** button, or `/explore`, asks for the same
mission.

In grove-g1, `./scripts/demos/nervros.sh map name:=my-building` starts the simulator with no map
and saves the new world as `data/worlds/my-building`. Afterwards,
`nervros.sh app saved:=my-building` runs on it.

## Finishing the map

A new map needs a person's eye. The world model types each room from the objects it finds there
and labels each object from what the detector saw, so a study with a daybed and an armchair can
come out as a living room. Fix it by hand in the world editor, or in the chat. Nothing is kept
until you save, and a save reaches the running world model at once.

### In the editor

https://github.com/user-attachments/assets/0c4ec282-3535-4302-8221-021d0d24642a

**Edit world** in the top bar (or `/edit`) puts the editor in place of the 3D view: the floor plan
with every room and object, and in the dock the selection and a list to review, most doubtful
first. Here the list has one entry: R7, typed "living room" from its objects at a score of 0.30.
It is the study, so: pick it, type "study" in **Type**, then **Set type**, **Mark checked** and
**Save**.

```text
saved 2 edits to data/worlds/media-mapped; the world model read them
```

On the plan:

- Click an object to select it, drag it to move it, drag a corner to resize it, or drag the handle
  to turn it.
- **Add object** draws a box around something the detector missed.
- **Split off…** and **Merge with…** fix one object seen as two, or two seen as one.
- **Delete** asks whether to delete an object for good or only remove it. **Restore** undoes a
  removal.
- N goes to the next thing to review, C checks the selection, Del deletes it, and **Undo** and
  **Redo** reach back to the last save.

### In the chat

https://github.com/user-attachments/assets/fce54439-64e5-45ef-8697-3ad88ba75b84

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
object, a daybed here. Each edit is approved on its card and waits, unsaved, until the save.

The agent has the editor's tools. `review_world` lists what deserves a second look,
`inspect_object` shows an object, `edit_world` relabels, renames, checks, removes, merges, splits,
adds and moves objects and types rooms, and `world_edits` saves, undoes, redoes or discards. The
editor and the chat share one history, so an edit made in one shows in the other.

## Sessions and models

Every conversation is saved. The Agent tab lists earlier sessions to resume, what the agent
remembers, and each model with its quota for the day.

- The context is condensed before it fills, and the status bar counts it beside the context
  ("condensed 3×"). `/compact` condenses it at once, and right-clicking one of your messages
  condenses everything before it. The chat says each time.
- A model that hits its rate limit is set aside, and one whose server is not running is tried
  last, so the next one in its role's chain answers. A busy model is asked once more first. When
  every model is at its limit for the minute, the turn waits for the first one to free up.
- **Save this one as a test** turns the session into an eval case.

## The command line

`nervros-cli` runs the same agent without a window, from the same profile:

```bash
# In chat: /arm, /disarm, /yes N, /allow N, /no N, /again N, /stop, /pause, /resume, /compact, /quit
nervros-cli --profile my.toml chat
nervros-cli --profile my.toml chat --resume last
nervros-cli --profile my.toml ask "What can you see?" --image frame.jpg
```

| Command | Does |
|---|---|
| `chat` | Chats with the robot; `--say` sends a script of messages and exits, `--model <id>` talks on one model alone |
| `ask` | Sends one prompt to one role's models, with any images in order (`--image` once each) |
| `look`, `segment` | Saves the camera's marks, or the regions a prompt names, as an image |
| `ros` | Runs one read-only ROS tool with no model: graph, topic sample, interface, TF, parameters, logs |
| `doctor` | Checks the profile's tools, topics and mission services against the live graph |
| `why`, `incidents` | Checks what is wrong with the robot now against the playbooks and keeps it as an incident; lists the kept incidents, or shows one in full |
| `missions`, `gaps` | Shows the mission ledger, and the requests no skill could do |
| `digest` | What the robot did over the last hours (`--hours`), from its ledger, incidents and goals |
| `inbox` | Lists the proposed fixes, accepted ones with their step's record before and since; `--dismiss N` turns one down while the app is closed |
| `replay` | Replays a session as it happened |
| `eval`, `case` | Runs test suites against the live robot or a scripted one, and keeps a session as a new case |
| `record` | Writes the live robot as a scripted robot for `eval --robot` |
| `whoami` | Drafts the robot's self-description from the live robot into `<persona>.draft.md`; `--facts` prints what it gathered |
| `models`, `skills`, `mcp` | Lists the model chains with today's counts, the skills, and the MCP tools to approve |

`doctor` checks the profile against the live graph, and `ros` runs one read tool with no model
(output shortened here):

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
    s1 GoToPlace R3 stopped: stopped: operator
```

### Evals

`eval` runs a suite of requests against the live robot, each in a fresh session with every
approval granted. It judges what happened: the tools and skills used, the missions' outcomes, the
approvals asked for, the reply, and how far and which way the robot moved, measured on the
simulator's own pose.

Each model and trial keeps its own robot files in the run's folder: keep-out zones, missions,
notes, places, programs and incidents. They carry from one case to the next and are emptied when a
run starts in the same folder, so a suite neither finds the operator's files nor leaves its own
behind. The apartment suite in grove-g1 has 46 cases:

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

That run covered the suite's first 44 cases.

- `--only <text>` runs the cases whose ids contain it.
- `--repeat k` runs each case k times, for pass^k.
- `--models a,b` compares models.
- `nervros-cli case last --id my-case --append suite.toml` keeps a session as a new case.

<details>
<summary>The suite format</summary>

A suite is TOML. Each `[[case]]` has an `id`, the `say` messages, optional `setup` and `max_s`,
and an `expect` table (`tools`, `not_tools`, `tool_said`, `skills`, `mission`, `approvals_max`,
`reply_has`, `reply_has_all`, `reply_lacks`, `moved_m`, `turned_deg`, `compacted`).
`scenarios/bench/suite.toml` is a small one.

</details>

`--robot <file>` runs a suite against a robot that a file describes, instead of the live graph.
Nothing moves on a scripted robot and no camera gives an image, so a run judges what the model
says and plans; a case that measures the robot's motion cannot pass there. A model comparison
takes minutes with no simulator running, though the model server must still be up.

<details>
<summary>What a robot file holds, and how to record one</summary>

The flag repeats, and a later file replaces an earlier one's entries key by key. A file scripts
the graph and the nodes each component container hosts, what each node publishes and subscribes
to, parameters with their descriptions, topic rates and messages, frames, and fixed answers for
services and actions. `scenarios/robot/robot.toml` has a mission executor that accepts every plan
and finishes every mission.

`nervros-cli record robot.toml` writes such a file from the live robot: the graph, the
containers, each node's topics and services, the parameters (those `param_read_deny` hides are
written as `(hidden)`), the rates and last messages of the topics the profile names, the robot's
pose on the map and the executor's catalog. It records no images, point clouds, maps or TF.

</details>

`scenarios/bench` holds ten generic tasks on a generic Nav2 robot:

```text
$ nervros-cli --profile scenarios/bench/nervros.toml eval scenarios/bench/suite.toml \
    --robot scenarios/robot/robot.toml --robot scenarios/bench/robot.toml --models qwen3.5-9b-local
...
10 of 10 trials passed; report ~/.local/state/nervros/evals/.../report.md
```

## Keys

| Key | Does |
|---|---|
| Enter, Shift+Enter | Sends; adds a new line |
| Esc | Stops the reply |
| Ctrl+Shift+S | Stops the mission |
| Ctrl+K | Opens the command palette |
| Ctrl+1 to Ctrl+0 | Opens the dock's tabs, in the bar's order |
| W S, A D, Q E | Drives by hand: walk, turn, step sideways (Robot tab, Drive on) |
