# NervROS

A native Rust agent for ROS 2 robots. You talk to it in a desktop app. It looks through the robot's
cameras, answers with marked-up images, and carries out tasks as behaviour trees that are checked
and approved before they run.

Status: early development. It is built and tested against
[grove-g1](https://github.com/Adyansh04/grove-g1), a Unitree G1 stack on ROS 2 Jazzy, in simulation.
