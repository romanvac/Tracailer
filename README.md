# Tracailer (ROS 2 Jazzy port)

This repository accompanies the paper, "Tracailer: An Efficient Trajectory Planner for Tractor-Trailer Robots in Unstructured Environments".

The project is ported to ROS 2 Jazzy and includes Docker/devcontainer support for a more reproducible setup.

The workspace includes the planner, simulator, MPC controller, random map generation, custom messages, launch files, and RViz support.

**Note:** We may have forgotten some dependencies or setup details 😟, sorry! Contributions and fixes are very welcome.

## Quick start

The easiest way to run the project is through Docker or a VS Code devcontainer.

### Docker

Build the image:

```bash
./scripts/build_docker.sh
```

Run the container with GUI forwarding:

```bash
./scripts/run_docker.sh
```

Inside the container:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch planner run_all.launch.py
```

If needed, the number of trailers can be changed at build time:

```bash
colcon build --symlink-install --cmake-args -DTRAILER_NUM=4
```

When the tractor-trailer model appears in RViz2, use `2D Pose Estimate` to trigger planning.

### Devcontainer

Open the repository in VS Code with the Dev Containers extension and choose **Reopen in Container**.

Then build and run:

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch planner run_all.launch.py
```

https://github.com/user-attachments/assets/fdf73b8e-d21c-4661-8249-173dcd9c223d
