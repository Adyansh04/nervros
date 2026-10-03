# NervROS

Talk to your ROS 2 robot. NervROS looks through the robot's cameras, keeps track of its world,
and turns what you ask into missions that you check and approve before anything moves.

https://github.com/user-attachments/assets/dc4712e6-81cc-4f7a-b986-baae0cda8bf9

*A simulated Unitree G1 fetches a mug: the plan, the approval, then the walk, the pick and the
place, four times faster while the robot works.*

NervROS fits any ROS 2 robot through one profile file and answers with a local model by default.
It is built and tested in simulation on [grove-g1](https://github.com/Adyansh04/grove-g1), a
Unitree G1 stack on ROS 2 Jazzy. Early development.

## What you can do

| Say or do | NervROS | |
|---|---|---|
| "What can you see right now?" | Looks through a camera and answers about numbered marks on the frame. | [clip](docs/guide.md#asking-what-it-sees) |
| "Where is the small white mug?" | Searches its world model: which room, on what, when it was seen. | [clip](docs/guide.md#asking-what-it-sees) |
| "Point at the tray." "Outline the desk." | Finds and outlines what you name, even what the detector missed. | [clip](docs/guide.md#pointing-and-outlining) |
| "Walk to the bedroom." | Plans with the robot's own skills, waits for your approval, runs the plan and reports back. | [clip](docs/guide.md#walking-somewhere) |
| "Bring the mug to the tray." | The same with the arms: walk, pick, carry, place, each step tracked live. | [clip](docs/guide.md#fetching-something) |
| "Turn left 90 degrees." | Checks every plan against your words (left or right, how far, which hand) before you see it. | [clip](docs/guide.md#a-short-one-and-the-plan-check) |
| "Stop." | Stops the robot at once, without waiting on a model. | [clip](docs/guide.md#stopping) |
| Click a room or an object in 3D | Offers to walk there, or to tell you about it. | [clip](docs/guide.md#clicking-in-the-3d-view) |
| Ctrl+K, `/follow`, `/doctor` | Runs an app command; most need no model. | [clip](docs/guide.md#commands-without-the-model) |
| Hold W, A, S, D | Drives the robot by hand. | [clip](docs/guide.md#driving-by-hand) |
| "Tell me if the camera drops below 5 Hz." | Watches, plots, or runs a plan on a schedule while you do other things. | [clip](docs/guide.md#keeping-an-eye-on-it) |
| "Remember this spot as the charging spot." | Keeps places and notes from one session to the next. | [clip](docs/guide.md#remembering) |
| "Explore the building." | Walks a new building until the camera has seen every room, mapping as it goes. | [clip](docs/guide.md#mapping-a-new-building) |
| "R1 is the living room. Change its type." | Fixes the new map in the chat, or by hand in the world editor. | [clip](docs/guide.md#finishing-the-map) |
| "What is publishing the map?" | Reads the ROS graph, topics, TF, parameters and logs, as `ros2` does. | [guide](docs/guide.md#debugging-the-ros-side) |
| `nervros-cli chat`, `doctor`, `eval` | The same agent in a terminal, a setup check, and tests against the live robot. | [guide](docs/guide.md#the-command-line) |

## Safe by design

- **Nothing moves until you say so.** The robot starts observe-only: arm it in the top bar, then
  approve each plan on its card.
- **Plans are checked before you see them.** The robot's executor validates every behaviour
  tree, and NervROS checks the plan against what you asked.
- **Stop always works.** "Stop", the Stop button or Ctrl+Shift+S halts the robot without a
  model, and a mission whose window closed or hung stops by itself.
- **The robot's data stays data.** What the cameras, topics and world model say reaches the
  model fenced as data, never as instructions, and hard deny lists keep every tool off motor and
  controller topics.

## Quick start

Ubuntu 24.04 with ROS 2 Jazzy, Rust through rustup (`rust-toolchain.toml` pins the version), and:

```bash
sudo apt install clang libclang-dev mold pkg-config libssl-dev libxkbcommon-dev \
  libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libudev-dev libwayland-dev
```

The local model needs Docker with the NVIDIA container toolkit and about 7 GB of GPU memory. The
first build compiles the Rerun viewer: about twenty minutes and 30-odd GB of `target/`.

```bash
./scripts/local-llm.sh start          # Qwen3.5-9B in llama.cpp on 127.0.0.1:8081
./scripts/build-overlay.sh            # builds nervros_interfaces once
source scripts/ros-env.sh             # bash; add NERVROS_UDP_ONLY=1 if the graph runs as root
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

Without a window:

```bash
nervros-cli --profile my.toml chat
nervros-cli --profile my.toml doctor
nervros-cli --profile my.toml eval suite.toml --repeat 3
```

The core needs neither ROS nor the viewer: `cargo build` builds it, and
`cargo test -p nervros-core --test scenarios` runs the scripted scenarios.

## How it works

1. **You ask**, in the app or a terminal.
2. **The agent**, a Rust core with a local or cloud model, uses tools: the cameras, the world
   model, ROS topics, services and actions, its memory, and any MCP server you add.
3. **Anything that moves the robot is a mission.** The agent plans with the robot's own skills,
   NervROS compiles the plan to a behaviour tree, the robot's executor checks it, and you approve
   it.
4. **The executor runs it** and reports each step back; the agent tells you how it went, with
   checks against the world model and the camera.

Nothing in NervROS is robot-specific. One [profile](docs/profile.md) names the robot's topics,
services, cameras, skills and limits, and the robot runs missions through a short
[executor contract](docs/executor.md). Models come from llama.cpp, Gemini, OpenRouter or any
OpenAI-compatible server, each role with its own chain of fallbacks ([models](docs/models.md)).

Also in the box: sessions you can resume, replay or keep as a test case; a context that condenses
itself before it fills; a privacy mode that keeps camera frames on local models; and
OpenTelemetry spans for each turn, model call, tool call and mission.

## Documentation

- [Guide](docs/guide.md): every way to use NervROS, with clips and the exact prompts.
- [Robot profile](docs/profile.md): one TOML file per robot.
- [Models](docs/models.md): providers, roles and quotas.
- [Mission executor contract](docs/executor.md): what a robot provides.
