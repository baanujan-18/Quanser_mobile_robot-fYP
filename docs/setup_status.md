# FYP Setup Status

## Current Stage

Completed up to Phase 4 before QBot 2 physical testing.

## Completed Work

### Phase 1 - Ubuntu Setup

- Ubuntu 22.04 installed on Raspberry Pi 4.
- Raspberry Pi connected using MobaXterm SSH.
- Raspberry Pi terminal access confirmed.

### Phase 2 - Ubuntu Update and Essential Tools

- Ubuntu package list updated.
- System packages upgraded.
- Essential tools installed:
  - curl
  - wget
  - git
  - build-essential
  - python3-pip

### Phase 3 - ROS 2 Humble Setup

- ROS 2 Humble repository added.
- ROS 2 Humble base installation completed.
- ROS 2 environment added to `.bashrc`.
- ROS 2 talker/listener test completed successfully.

### Phase 4 - Kobuki Driver Setup

- ROS 2 workspace created at `~/fyp_ws`.
- Kobuki related packages cloned into `~/fyp_ws/src`.
- Dependencies installed using rosdep.
- Workspace built using:

```bash
colcon build --symlink-install
