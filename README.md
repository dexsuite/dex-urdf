This repo provides a collection of high-quality dexterous hand models in URDF format.

| Robot Model  | Ray-Tracing[^1] | Rasterization[^2] | Load Checking[^3]  |  Hand Type   |
|:------------:|:---------------:|:-----------------:|:------------------:|:------------:|
| Allegro Hand |        0        |         0         | :white_check_mark: | Left & Right |

[^1] Ray tracing images are rendered in `SAPIEN` using the urdf with `glb` version
[^2] Rasterization images are rendered in `IsaacGym` using the with the `glb` version
[^3] Loading checking means the URDF has been tested in the
following platform: [yourdfpy](https://github.com/clemense/yourdfpy),
[IsaacGym](https://developer.nvidia.com/isaac-gym),
[SAPIEN](https://sapien.ucsd.edu/),
[PyBullet](https://pybullet.org/wordpress/)

## Why do we need this repo?

The robot models in this repo are different from the original robot description files provided by the robot driver
package in serval aspects:

<details>
<summary>Improved collision mesh</summary>
<br>
  All collision mesh are represented as stl or URDF primitives with simplified triangle meshes, i.e. fewer vertices and
  simpler edge connection. No self collision after loading into simulator.

|          Allegro Visual Model          |              Original Collision Model              |           Improved Collision Model            |
|:--------------------------------------:|:--------------------------------------------------:|:---------------------------------------------:|
| ![](doc/improved_collision/visual.png) | ![](doc/improved_collision/original_collision.png) | ![](doc/improved_collision/new_collision.png) | 

</details>

<details>
<summary>Improved visual mesh</summary>
</details>

<details>
<summary>More stable inertia parameters</summary>
</details>

<details>
<summary>Consistent URDF parsing across different parsers, e.g. different simulator</summary>
</details>

<details>
<summary>Consistent root convention and auxiliary links</summary>

For all dexterous hands, the orientation are kept consistent across all dexterous hands. For right hand, the x-axis is
forward, the y-axis the direction from litter finger to thumb, the z-axis is the direction from wrist to fingertip of
middle finger.
</details>

1. Consistent hand orientation
2. Consistent mesh format. All visual mesh are stored as `.obj` but not `.dae`, since different DAE loader may treat DAE
   differently, resulting inconsistent behavior across different URDF parser. All collision meshes are stored as `.stl`.

## Source files of robot

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
- [X] SAPIEN URDF checking
- [X] IsaacGym URDF checking
- [X] Self collision checking
