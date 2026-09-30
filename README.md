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

`nervros-cli` does the same headless: `chat`, `doctor`, `look`, `models` and `ask`.

In the window, Enter sends, Esc stops the reply and Ctrl+Shift+S stops the mission. Ctrl+1 to
Ctrl+5 switch the dock between the mission, approvals, events, models and the connection check.
The embedded viewer is [Rerun](https://rerun.io), which keeps the camera, map, rooms and objects
on a timeline.

## Documentation

- [The robot profile](docs/profile.md): one TOML file per robot; a ROS service or topic becomes a
  tool by naming it and its type.
- [Models](docs/models.md): providers, roles and quotas, all configuration.
- [The mission executor contract](docs/executor.md): what a robot's executor provides so the agent
  can plan and run behaviour trees on it.
