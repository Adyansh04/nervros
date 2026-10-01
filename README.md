# NervROS

A native Rust agent for ROS 2 robots. You talk to it in a desktop app. It looks through the robot's
cameras, answers with marked-up images, and carries out tasks as behaviour trees that are checked
and approved before they run.

Status: early development. It is built and tested against
[grove-g1](https://github.com/Adyansh04/grove-g1), a Unitree G1 stack on ROS 2 Jazzy, in simulation.

![The NervROS window: chat on the left, the robot's camera and 3D world in the middle](apps/nervros-gui/tests/snapshots/window.png)

## What it needs

On the host, Ubuntu 24.04 with:

- ROS 2 Jazzy, for r2r's bindings and the interface overlay (`colcon`);
- Rust through rustup: `rust-toolchain.toml` pins the version, and the first `cargo` call installs
  it;
- clang, libclang, mold and pkg-config for the builds;
- for the app's viewer: `libxkbcommon-dev libxcb-render0-dev libxcb-shape0-dev
  libxcb-xfixes0-dev libudev-dev libwayland-dev`, and a Vulkan driver;
- Docker with the NVIDIA container toolkit and about 7 GB of GPU memory for the local model.

```bash
sudo apt install clang libclang-dev mold pkg-config libssl-dev libxkbcommon-dev \
  libxcb-render0-dev libxcb-shape0-dev libxcb-xfixes0-dev libudev-dev libwayland-dev
```

The first build of the app compiles the Rerun viewer: about twenty minutes, with builds capped at
four jobs in `.cargo/config.toml`, and 30-odd GB of `target/` on disk. The core needs neither ROS
nor the viewer: a plain `cargo build` builds it and the schema generator only.

## Try it

NervROS runs on the host next to a ROS 2 Jazzy graph. Tests use a local model, so nothing leaves
the machine:

```bash
./scripts/local-llm.sh start          # Qwen3.5-9B in llama.cpp on 127.0.0.1:8081
./scripts/build-overlay.sh            # builds nervros_interfaces once
source scripts/ros-env.sh             # bash; add NERVROS_UDP_ONLY=1 if the graph runs as root
cargo run -p nervros-gui -- --profile profiles/example/nervros.toml
```

`nervros-cli` does the same headless: `chat`, `doctor`, `look`, `segment`, `models`, `ask`, and
`ros`, which runs one of the ROS debugging tools against the live graph without a model:

```bash
nervros-cli --profile my.toml ros topic_sample '{"topic": "/odom", "mode": "hz"}'
```

With a [`[ros_tools]`](docs/profile.md#ros_tools) table the agent can look at any part of the
graph as `ros2` would (topics with their QoS, rates, messages, nodes, services, actions,
parameters, TF and logs), and, where the profile lists them, call services, send action goals,
set parameters and publish, each approved by you. With [`[segment]`](docs/profile.md#segment) it
segments whatever you name in a camera's view, "the floor" or "every mug", and shows you the
regions. With [`[editor]`](docs/profile.md#editor) it reviews and fixes the saved world model when
you ask ("the chair by the window is a stool", "merge the two halves of the sofa"), and Edit world
opens the same world on its floor plan to fix by hand. It also runs the checks you would: "check
the robot's health", "tell me if the chest camera drops below 5 Hz", "plot the robot's speed",
"remember this spot as the reading corner". It remembers what you ask it to across sessions
("remember that the kitchen door sticks"), and runs patrols you approve once ("every 30 minutes,
walk through the rooms, 6 times").

To act, the agent writes a plan and the app asks you to approve it, once: the agent never asks you
first in the chat, and you deny what is wrong. A failed mission comes back with what went wrong, and
the agent proposes a changed plan for you to approve.

In the window, Enter sends, Esc stops the reply and Ctrl+Shift+S stops the mission. Ctrl+1 to
Ctrl+7 switch the dock between the mission, the world model, the viewer's layers, approvals, events,
the agent (earlier sessions to resume, what it remembers, the models) and the connection check. The
status bar shows how full the model's context is; the conversation is condensed before it fills,
and `/compact` condenses it at once. Every conversation is saved, so Resume in the Agent tab, or
`nervros-cli chat --resume last`, carries one on. Closing the window while a mission runs asks first
whether to stop the robot, and a new session tells you when the robot is already running one.
The embedded viewer is [Rerun](https://rerun.io), which keeps the cameras, map, rooms, objects and
mission steps on a timeline. [`[[viz.layer]]`](docs/profile.md#viz) adds what RViz would draw:
occupancy grids, marker arrays and laser scans, each with a switch in the Layers tab. Clicking an
object, a room or a point on the map offers messages about it, such as "Walk to O17 (shelf).",
filled into the composer for you to read and send.

## Documentation

- [The robot profile](docs/profile.md): one TOML file per robot; a ROS service or topic becomes a
  tool by naming it and its type.
- [Models](docs/models.md): providers, roles and quotas, all configuration.
- [The mission executor contract](docs/executor.md): what a robot's executor provides so the agent
  can plan and run behaviour trees on it.
