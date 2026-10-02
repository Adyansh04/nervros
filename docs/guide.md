# Using NervROS

This guide walks through NervROS the way an operator uses it: asking what the robot sees, giving
it tasks, checking on it, and looking after it from the command line. Every prompt and reply
is from a real run: a simulated Unitree G1 in a six-room apartment
([grove-g1](https://github.com/Adyansh04/grove-g1)), with the local Qwen3.5-9B model answering.
The [README](../README.md) has the setup; the [profile](profile.md), [models](models.md) and
[executor](executor.md) pages have every setting.

## The window

```bash
source scripts/ros-env.sh
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

<!-- image window.png: The window at the start of a session -->

- The top bar names the robot and shows the ROS graph, the mission executor and the model at a
  glance. The switch on the right arms the robot: until it is armed, the agent can look and
  answer but not act. **Stop mission** is always there.
- The chat is on the left. Enter sends, Shift+Enter adds a line, and Esc stops the reply.
- In the middle is the viewer: the map with its rooms and objects, the robot, the cameras with
  what the detector marks, and the agent's log, all on one timeline you can scrub back.
- The dock on the right holds the tabs: Mission, World, Layers, Approvals, Events, Agent, Doctor
  and Robot (Ctrl+1 to Ctrl+8).
- The status bar shows how full the model's context is and how much of it came from the model
  server's cache. Ctrl+K opens the command palette, and each command also runs as a `/word` in
  the chat, such as `/look`, `/arm` or `/compact`.

## Asking what it sees

> **You:** What can you see right now?

The agent calls `look`: the newest camera frame, with each detection drawn as a numbered mark.
The model answers about the marks by number, and you can refer to them the same way.

<!-- image look.png: look: the desk with five numbered marks -->

> **NervROS:** The chest camera shows a green desk with drawers on the left, holding a brown tray
> (mark 5) and a blue laptop (mark 4). A chair (mark 3) is partially visible under the desk. The
> scene includes wooden flooring and plain walls. There's also a mug detected (mark 1).

<!-- video: look.mp4 -->

**Show in 3D** pauses the viewer at the moment the snapshot was taken, where the robot was and
what the world model held then, and **Ask about it** starts a message about that snapshot. "Look closer at mark 1" crops the mark from the
full frame at a higher resolution for the vision model, for a label or a small part.

### Pointing and segmenting

> **You:** Point at the tray.
>
> **NervROS:** I've pointed at the tray (mark 5). The pointer is now shown as a ring on the brown
> tray on the desk.

<!-- image point.png: point: a ring on the tray -->

`point` asks a vision model, so it also finds things the detector never marked. `segment`
outlines whatever you name:

> **You:** Segment the floor in the chest camera.
>
> **NervROS:** I've segmented the floor in the chest camera view. The segmented region (mark 1)
> covers about 54.7% of the frame and shows the wooden floor with the rectangular mat/rug visible.

<!-- image segment.png: segment: the floor outlined -->

<!-- videos: point.mp4, segment.mp4 -->

### Finding things

`find_objects` searches the world model, which remembers what the robot has seen around the
building, room by room:

> **You:** Where is the small white mug?

The World tab lists the rooms and objects, and clicking an object, a room or a point on the map
offers messages about it, such as "Walk to O17 (shelf).", in the composer for you to send.

<!-- image tab-world.png: The World tab -->

## Giving it a task

Anything that moves the robot is a mission. The agent writes a plan from the robot's own skills,
NervROS compiles it to a behaviour tree, the executor checks it, and only then does it reach you.

### A plan checked against your words

> **You:** Turn left 90 degrees.

The model's first plan was `TurnInPlace(degrees=-90)`, a right turn. It never reached the
approval card:

```text
run_mission failed s1: the operator said turn left, but degrees=-90 turns the other way
(positive degrees turn left). The plan may not do what the operator asked: fix it, or if it is
right, send it again unchanged and the operator sees these concerns when approving
```

The model fixed it, and the card asked for the corrected plan:

<!-- image plan-check.png: The plan check sending the plan back, then the approval card -->

```text
? approve run_mission: runs "turn left 90 degrees": 1 step(s), at most 1 min
report> Mission 01a0fc74 (turn left 90 degrees) ended: success after 2 s.
```

<!-- video: turn-left.mp4 -->

The checks cover left and right, forward and back, how far, how much and which hand, for the
skills the profile's `[mission.checks]` names. When the model insists, the concern stays on the
card with a one-click fix. **Edit** changes, drops or moves a step before you approve.

### Walking somewhere

> **You:** Go to the bedroom.

<!-- image walk-approval.png: The walk's approval, with the path previewed in the viewer -->

The plan card shows each step's track record on this robot ("4 of 4 · 21 s"), and the viewer
previews where the walk goes. When the mission ends, the agent gets a report with its goal
checks:

```text
report> Mission 01a0fc74 (go to the bedroom (room C)) ended: success after 29 s.
        Goal checks: at(R3) holds (the robot is in R3).
```

<!-- video: go-to-the-bedroom.mp4 -->

### Fetching something

> **You:** Bring the small white mug from the dining table to the tray on the office desk.

The executor refused the first plan for naming objects by the wrong ids (`s3: object_id must be
one of: mug_4; s6: container_id must be one of: tray_1`); the second plan went through as four
steps:

```text
plan: GoToPlace(place=dining_table_side) -> PickObject(object_id=mug_4, arm=left)
      -> GoToPlace(place=office_desk_tray) -> PlaceInto(container_id=tray_1, arm=left)
```

<!-- image pick-and-place.png: The mission running: each step's state, track record and the preview -->

The Mission tab shows the behaviour tree as it runs, node by node, and the missions before it:

<!-- image mission-tab.png: The Mission tab: the plan and its behaviour tree, all done -->

After five minutes:

```text
report> Mission 01a0fc75 (go to the dining table to pick up the mug) ended: success after 298 s.
        Camera check, before and after: inside(mug_4, tray_1): the camera agrees.
```

<!-- videos: pick-and-place.mp4 (its first four minutes), health-check.mp4 (the end) -->

A failed mission comes back with the step that failed, why, and what the world model saw of the
object it was about; the agent then proposes a changed plan for you to approve. A mission keeps
running while you talk: "Check the robot's health" above was asked halfway through.

### Stopping

> **You:** Stop.

A plain "stop" (or "halt", "freeze", "stop!") never waits on a model: it stops the reply and the
robot at once, through the executor's `StopAll`. Esc stops only the reply, and **Stop mission**
or Ctrl+Shift+S stops the robot.

```text
[halted: stopped by the operator: reply and robot]
[the robot stopped: {"stopped":true,"state":"holding posture; hands empty"}]
```

<!-- image stop.png: Stop, typed in the chat -->

<!-- video: stop.mp4 -->

A stop never opens a hand: a robot holding a mug keeps it. While a mission runs, the window sends
the executor a heartbeat, so a mission whose window closed or hung stops by itself.

### Doing it again

- "Save that as the evening check" keeps a plan that worked, and "Run the evening check" runs it
  again. The Mission tab lists saved plans.
- "Every 30 minutes, walk through the rooms, 6 times" makes a schedule, approved once for every
  run. A schedule can also wait for something: an object of a kind appearing, or a condition on a
  topic. Stopping the robot cancels every schedule.
- Requests no skill can do are logged as skill gaps, for whoever writes the next skill.

## Keeping an eye on it

> **You:** Check the robot's health.
>
> **NervROS:** The robot's health check is complete and all systems are functioning properly. All
> 11 checks pass, including both cameras operating at 10 Hz, the robot correctly located on the
> map, and the current mission (placing mug_4 into tray_1) running successfully with the left
> hand holding the mug.

<!-- image health.png: The health check, asked while the mission ran -->

> **You:** Tell me if the chest camera drops below 5 Hz.
>
> **NervROS:** I've set up a watch to monitor the chest camera's frame rate. It will alert me if
> the rate drops below 5 Hz for the next 30 minutes. Currently, the camera is operating at 10 Hz
> as expected.

A watch runs in the background and posts to the chat when its condition holds: a rate dropping, a
value crossing a line, or a log line matching. "Plot the robot's speed" draws a number from a
topic in the viewer's Plots tab:

```text
> plot {"topic": "/Odometry_loc", "field": "twist.twist.linear.x", "for_s": 120}
< plotting /Odometry_loc twist.twist.linear.x in the viewer's Plots tab for 120 s
```

<!-- videos: health-check.mp4, watch.mp4, plot.mp4 -->

The Doctor tab runs the same connection check on its own, and the Robot tab shows whether the
robot is upright, where it is, what each hand holds, and drives it by hand through the executor
when you switch **Drive** on (W and S walk, A and D turn, Q and E step sideways).

<!-- image tab-doctor.png: The Doctor tab -->
<!-- image tab-robot.png: The Robot tab -->

### Debugging the ROS side

With a `[ros_tools]` table the agent reads the graph as `ros2` does: "what is publishing the
map?", "is the chest camera dropping frames?", "show me the controller's parameters", "any errors
in the logs?". Acting through ROS (a service call, an action goal, a parameter, a publish) needs
the profile to allow it, and your approval each time.

## Remembering

> **You:** Remember this spot as the reading corner.

<!-- image remember.png: The approval for a new place, then the agent's summary of the session -->

> **NervROS:** I've saved this location as the reading corner (also known as "the reading
> corner"). The robot will remember this spot at coordinates (5.51, -3.89) with a yaw of -2.06
> radians for future navigation tasks.

"Go to the reading corner" works from then on, in this session and the next. Notes work the same
way: "remember that the kitchen door sticks" joins what the agent knows on every turn, and the
Agent tab lists them. "What have you done so far?" and "what happened to the mug?" read the
mission ledger and the world model's history.

<!-- videos: remember-a-place.mp4, recall.mp4 -->

The world model can be fixed in the chat ("the chair by the window is a stool", "merge the two
halves of the sofa"), each edit approved, or by hand: **Edit world** opens it on its floor plan.

## Sessions and models

Every conversation is saved. The Agent tab lists earlier sessions to resume, what the agent
remembers, and each model with its quota for the day:

<!-- image tab-agent.png: The Agent tab -->

The context is condensed before it fills; `/compact` condenses it at once, and right-clicking a
message condenses everything before it. A model that hits its rate limit is set aside and the
next one in its role's chain answers. **Save this one as a test** turns the session into an eval
case.

## The command line

`nervros-cli` runs the same agent without a window:

```bash
nervros-cli --profile my.toml chat                     # /yes N, /no N, /stop, /compact, /quit
nervros-cli --profile my.toml chat --resume last
nervros-cli --profile my.toml replay last              # a session as it happened
```

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

`missions` and `gaps` read the ledger:

```text
$ nervros-cli --profile nervros.toml missions --limit 3
01a0fc75-1e3c 16 min ago: go to the dining table to pick up the mug (success, 298 s)
01a0fc74-53ca 17 min ago: go to the bedroom (room C) (success, 29 s)
01a0fc74-1746 17 min ago: turn left 90 degrees (success, 2 s)
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
