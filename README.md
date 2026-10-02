# NervROS

A Rust agent for ROS 2 robots that you talk to. It looks through the robot's cameras, answers with
marked-up images, and carries out tasks as behaviour trees that you check and approve before they
run.

![The NervROS window: the chat on the left, the robot's camera and 3D world in the middle](apps/nervros-gui/tests/snapshots/window.png)

Early development. Built and tested against [grove-g1](https://github.com/Adyansh04/grove-g1), a
Unitree G1 stack on ROS 2 Jazzy, in simulation.

## Features

### Talking to the robot

- A desktop app: the chat beside an embedded [Rerun](https://rerun.io) viewer that keeps the
  cameras, the map, rooms, objects, the robot's model and the mission on one timeline.
- The same agent headless in `nervros-cli`: chat, replay a session, check the setup, look, segment
  and ask one model.
- Replies stream as the model writes them. An image you paste or drop goes with your next message.
- Every conversation is saved and can be resumed. The context is condensed before it fills, or
  when you ask (`/compact`, or up to a message).
- A command palette (Ctrl+K), keyboard shortcuts, and a status bar with the model, its quota and
  how full the context is.

### Seeing

- `look`: the camera frame with each detection drawn as a numbered mark that you and the model
  refer to by number; a closer look at one mark; earlier snapshots.
- `segment`: outlines whatever you name ("the floor", "every mug"), from a vision model or the
  robot's own segmentation service.
- `point`: points at what you name, including things no detector marks.
- The world model's rooms and objects in the viewer. Clicking one, or a point on the map, offers
  messages about it, such as "Walk to O17 (shelf)."

### Acting through missions

- The agent plans with the robot's own skills, read from its executor's catalog. A plan compiles
  to a behaviour tree, which the executor checks before you see it.
- You approve a plan once, in the app. Its card shows the steps, how each went on this robot
  before, and any concerns, and the viewer previews where its walks end. You can edit a step
  before approving.
- Plans are checked against your words: left or right, forward or back, how far, how much, which
  hand. A mismatch comes with a one-click fix, and a second model can review plans too.
- A mission reports back when it ends: its outcome, goal checks against the world model and the
  camera, and what happened to the object a failed step was about.
- A mission ledger keeps track records, saved plans, and the requests no skill could do.
- Schedules run a plan on a clock or when something happens, such as a patrol every 30 minutes,
  approved once.
- A heartbeat stops a mission whose app has closed or hung.
- Driving by hand from the Robot tab, which also shows the robot's state, hands and motors.

### Safety

- Every tool call passes a guard: observe and act lanes, arming, an autonomy mode, budgets per
  turn, a loop breaker and locks on the base and arms.
- Acting needs the robot armed and, when supervised, your approval. Rules on a tool's arguments
  can refuse a call or make it ask, and hard deny lists keep every tool off motor, velocity and
  controller topics.
- Stop is always allowed and never waits on a model. "Stop" typed in the chat takes the same path.
- Every payload is checked against its ROS interface before it reaches the robot.
- What the robot and the world model say reaches the model fenced as data, not instructions.
- A privacy mode for homes: camera frames stay on local models, and text stays off models that
  train on it.

### Debugging ROS the way `ros2` does

- The graph, topics (echo, rate, bandwidth, QoS mismatches), interfaces, TF, parameters and logs.
- Service calls, action goals, parameter changes and publishing, where the profile allows them,
  each approved.
- Watches that report when a rate drops, a value crosses a line or a log line matches; plots; a
  health check.
- `nervros-cli ros` runs one read tool without a model, and `nervros-cli doctor` checks the
  profile against the live graph.

### Memory and the world model

- Notes that last across sessions ("remember that the kitchen door sticks") and named places
  ("remember this spot as the reading corner").
- Reviewing and fixing the saved world model in the chat, or by hand in the built-in world editor.

### Fitting a robot

- Nothing in NervROS is robot-specific: one TOML profile per robot names its topics, services,
  cameras, skills and limits.
- Any ROS service, action or topic becomes a tool by naming it and its type in the profile.
- A robot runs missions through an executor that follows [a short contract](docs/executor.md).
- Tools from MCP servers, each pinned to the definition you approved, and skills: short procedures
  kept as `SKILL.md` folders.

### Models

- Local models through llama.cpp, Gemini, OpenRouter and any OpenAI-compatible provider, with a
  chain of models per role: the routine one, plan advice, plan checks, vision checks, summaries
  and segmentation.
- Quotas counted per model and per shared pool, reset as each provider counts them. A rate-limited
  model is set aside for as long as it asks, and the next one answers.
- A free-only switch: on OpenRouter, start-up refuses any model that is not free.

### Testing and tracing

- `nervros-cli eval`: suites of requests judged against the live robot (tools, skills, missions,
  approvals, replies, and how far and which way the robot moved), with pass^k and model
  comparisons.
- Scripted scenarios that run through the whole agent with no model and no ROS.
- Session logs you can replay, or keep as a test case.
- OpenTelemetry spans for each turn, model call, tool call and mission.

## Quick start

Ubuntu 24.04 with ROS 2 Jazzy, Rust through rustup (`rust-toolchain.toml` pins the version), and:

```bash
sudo apt install clang libclang-dev mold pkg-config libssl-dev libxkbcommon-dev \
  libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libudev-dev libwayland-dev
```

The local model needs Docker with the NVIDIA container toolkit and about 7 GB of GPU memory. The
first build of the app compiles the Rerun viewer: about twenty minutes and 30-odd GB of `target/`.

```bash
./scripts/local-llm.sh start          # Qwen3.5-9B in llama.cpp on 127.0.0.1:8081
./scripts/build-overlay.sh            # builds nervros_interfaces once
source scripts/ros-env.sh             # bash; add NERVROS_UDP_ONLY=1 if the graph runs as root
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

Without a window:

```bash
nervros-cli --profile my.toml chat
nervros-cli --profile my.toml ros topic_sample '{"topic": "/odom", "mode": "hz"}'
nervros-cli --profile my.toml eval suite.toml --repeat 3
```

The core needs neither ROS nor the viewer: `cargo build` builds it, and
`cargo test -p nervros-core --test scenarios` runs the scripted scenarios.

## Documentation

- [Guide](docs/guide.md): using NervROS, with examples.
- [Robot profile](docs/profile.md): one TOML file per robot.
- [Models](docs/models.md): providers, roles and quotas.
- [Mission executor contract](docs/executor.md): what a robot provides so the agent can plan and
  run behaviour trees on it.
