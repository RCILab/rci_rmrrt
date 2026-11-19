# Robot and object models

| Folder | What | Source |
| --- | --- | --- |
| `h1_2/` | Unitree H1-2: URDF with convex-hull collision meshes (`meshes/collision`), simplified visual meshes (`meshes/visual`), SRDF and joint limits | the H1-2 model of [unitree_ros](https://github.com/unitreerobotics/unitree_ros) (BSD-3-Clause) |
| `dual_panda/dual_panda.urdf.xacro` | two Pandas on a stand (S2, S3), built from the macros of `franka_description` | the dual-arm setup of [dual_panda_moveit_config](https://github.com/psh117/dual_panda_moveit_config) |
| `objects/ikea_stefan_chair.stl` | the chair carried in S4 | [assembly_knowledge](https://github.com/psh117/assembly_knowledge) |

The Panda robots of S1-S4 come from the packages listed in `dependencies.repos` and from
`franka_description` (ROS package `ros-noetic-franka-description`).
