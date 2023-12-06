This repository offers an assortment of high-quality models for dexterous hands and objects. Both of them are in URDF
format.

|   Robot Model   |                                                  Visual[^1]                                                  | Collision[^2] | 
|:---------------:|:------------------------------------------------------------------------------------------------------------:|:-------------:|
|  Allegro Hand   | [<img src="doc/gallery/allegro_rt.webp" width="400">](robots/hands/allegro_hand/allegro_hand_right_glb.urdf) |       0       |
|   Shadow Hand   |  [<img src="doc/gallery/shadow_rt.webp" width="400">](robots/hands/shadow_hand/shadow_hand_right_glb.urdf)   |       0       |
| SCHUNK SVH Hand |  [<img src="doc/gallery/svh_rt.webp" width="400">](robots/hands/schunk_hand/schunk_svh_hand_right_glb.urdf)  |       0       |
|  Ability Hand   | [<img src="doc/gallery/ability_rt.webp" width="400">](robots/hands/ability_hand/ability_hand_right_glb.urdf) |       0       |
|    Leap Hand    |     [<img src="doc/gallery/leap_rt.webp" width="400">](robots/hands/leap_hand/leap_hand_right_glb.urdf)      |       0       |
|  DClaw Gripper  |    [<img src="doc/gallery/dclaw_rt.webp" width="400">](robots/hands/dclaw_gripper/dclaw_gripper_glb.urdf)    |       0       |
|  Barrett Hand   |     [<img src="doc/gallery/bhand_rt.webp" width="400">](robots/hands/barrett_hand/bhand_model_glb.urdf)      |       0       |

[^1]: Ray tracing animation are rendered in `SAPIEN` using the urdf with `glb` version. Code can be found
in [SAPIEN](tools/generate_urdf_animation_sapien.py)
[yourdfpy](https://github.com/clemense/yourdfpy),
[IsaacGym](https://developer.nvidia.com/isaac-gym),
[SAPIEN](https://sapien.ucsd.edu/),
[PyBullet](https://pybullet.org/wordpress/)

## Robot Source

|   Robot Model   |                          Official Website                           |                                                 URDF Source                                                 |                                    CAD Model Source                                    |  License   |
|:---------------:|:-------------------------------------------------------------------:|:-----------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------:|:----------:|
|  Allegro Hand   | [Wonik Robotics](https://www.wonikrobotics.com/research-robot-hand) | [allegro_hand_ros](https://github.com/simlabrobotics/allegro_hand_ros/tree/master/allegro_hand_description) |                                          N/A                                           |    BSD     |
| SCHUNK SVH Hand |                 [SCHUNK](https://schunk.com/us/en)                  |             [schunk_svh_ros_driver](https://github.com/SCHUNK-GmbH-Co-KG/schunk_svh_ros_driver)             |                                          N/A                                           | Apache-2.0 |
|   Shadow Hand   |        [Shadow Robot Company](https://www.shadowrobot.com/)         |                     [schunk_svh_ros_driver](https://github.com/shadow-robot/sr_common)                      |                                          N/A                                           |  GPL-3.0   |
|   Robel DClaw   |     [Robel Benchmark](https://github.com/google-research/robel)     |                                                     N/A                                                     | [D'Claw CAD](https://drive.google.com/drive/folders/1H1xN5BU03-eXjuEyIL_iJ_4XzrdDSnlM) | Apache-2.0 |
|  Barrett Hand   |  [Barrett Technology](http://barrett.com/robot/products-hand.html)  |                        [bhand_model](https://github.com/jhu-lcsr-attic/bhand_model)                         |    [BarrettHand CAD](https://github.com/jhu-lcsr-attic/bhand_model/tree/master/cad)    |    BSD     |
|  Ability Hand   |                 [PSYONIC](https://www.psyonic.io/)                  |                     [ability-hand-api](https://github.com/psyonicinc/ability-hand-api)                      |                                          N/A                                           |    N/A     |

## Todo

- [ ] Attach hand on robot arm
- [ ] Manipulated objects for dexterous hands
- [ ] Add Barrett Hand
- [ ] PyBullet URDF checking
- [X] SAPIEN URDF checking
- [X] IsaacGym URDF checking
- [X] Self collision checking
