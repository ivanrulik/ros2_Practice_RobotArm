# Practice Robot Arm

This repository contains a set of ROS 2 packages for experimenting with a simple robot arm in simulation and on real hardware. It is primarily a learning project for ROS 2 concepts and tooling.

## Repository Layout

- `src/` - ROS 2 packages with examples, descriptions and utilities
- `scripts/` - helper scripts used during development
- `docs/` - design documents and diagrams
- `.devcontainer/` - VS Code Dev Container configuration

## Prerequisites

The packages target **ROS 2 Humble** on Ubuntu 22.04 or later. Install ROS 2 following the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) or use the provided dev container.

### Using the Dev Container

1. Install [Docker](https://docs.docker.com/get-docker/) and [VS Code](https://code.visualstudio.com/) with the Dev Containers extension.
2. Open the repository in VS Code and select **Reopen in Container** when prompted.
3. After the container builds, build the workspace:
   ```bash
   colcon build --symlink-install
   ```
4. Source the workspace setup file in new terminals:
   ```bash
   source install/setup.bash
   ```

The container image includes GPU support so GUI tools like `rviz2` can run.

If you are on **Windows 11**, open the repository in the WSL 2 environment.
The dev container mounts the WSLg runtime so graphical ROS tools display on the
Windows desktop. Ensure WSLg is installed (included in recent Windows 11
releases) or run an X server such as VcXsrv.

## Building Locally

Outside the dev container, ensure you have ROS 2 Humble installed and then run:

```bash
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

## Running Examples

Example launch files are provided in the packages under `src/`. For instance:

```bash
ros2 launch practice_robotarm_controller demo.launch.py
```

Adjust the package and launch file names to suit your needs.

## Contributing

Contributions and bug reports are welcome. Please run `colcon test` before submitting pull requests.

## License

This project is released under the terms of the MIT license.
