# Working on NervROS

Notes for anyone, human or coding agent, who changes this repository.

## What it is

NervROS is a robot-neutral Rust agent for ROS 2. You chat with it, it looks through the robot's
cameras, and it acts only through missions: behaviour trees it compiles from a validated plan, which
the robot's own executor runs after you approve them. Robot-specific values live in a robot
profile (TOML) kept in the robot's repository, never here.

## Layout

| Path | What it holds |
|---|---|
| `crates/rosidl-schema` | `.msg`, `.srv` and `.action` parser that keeps comments; JSON Schema generator. No ROS needed |
| `crates/nervros-ros` | The `RobotPort` trait, the r2r implementation and a scripted `FakeRobot` for tests |
| `crates/nervros-core` | Configuration, model providers and router, tools, guard, missions, the session actor, the event log |
| `crates/nervros-viz` | ROS and agent data to Rerun |
| `apps/nervros-gui` | The desktop app: our panels around an embedded Rerun viewer |
| `apps/nervros-cli` | Headless chat, doctor, scenario runner, replay |
| `ros/nervros_interfaces` | The ROS contract a robot's mission executor implements |

## Commands

```bash
cargo build                         # default members: no GUI
cargo build -p nervros-gui          # the GUI (large: embeds Rerun)
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo fmt --all
```

Builds are capped at four jobs in `.cargo/config.toml`. Leave that cap in place.

## Rules

- **Motion only through missions.** Never add a tool that publishes velocities, joint commands or
  low-level robot commands, or that switches controllers.
- **Safety lives in code, not in prompts.** The guard, the stop path, resource locks and approvals
  are deterministic; the stop path never waits on a model.
- **Free models only.** Never a paid model or tier. Tests use the local model or a scripted mock.
  Cloud calls need an explicit budget.
- **Secrets.** Keys come from a file or environment variable into a `SecretString`, go only in an
  auth header, and are never logged or put in a URL.
- **Errors.** No `unwrap`, `expect` or `panic!` on a runtime path. Libraries return typed errors
  (`thiserror`); binaries use `anyhow` with context.
- **Comments.** Explain why, not what, in one or two lines. Every public item gets a `///` doc.
- **Dependencies.** Few, pinned exactly in `[workspace.dependencies]`, one upgrade per PR.
- **Git.**
  - One branch per change, named `<type>/<kebab-name>`, where the type is `feat`, `fix`, `docs`,
    `chore`, `test` or `refactor`.
  - Commit only when the branch is finished and tested, in a few meaningful commits.
  - Merge PRs with a merge commit. Never squash.
