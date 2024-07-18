from pathlib import Path
from typing import Optional

import cv2
import ffmpeg
import numpy as np
import sapien.core as sapien
import tqdm
import transforms3d
import tyro
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer


def generate_joint_limit_trajectory(robot: sapien.physx.PhysxArticulation, loop_steps: int):
    joint_limits = robot.get_qlimits()
    for index, joint in enumerate(robot.get_active_joints()):
        if joint.type == "continuous":
            joint_limits[:, index] = [0, np.pi * 2]

    trajectory_via_points = np.stack([joint_limits[:, 0], joint_limits[:, 1], joint_limits[:, 0]], axis=1)
    times = np.linspace(0.0, 1.0, int(loop_steps))
    bins = np.arange(3) / 2.0

    # Compute alphas for each time
    inds = np.digitize(times, bins, right=True)
    inds[inds == 0] = 1
    alphas = (bins[inds] - times) / (bins[inds] - bins[inds - 1])

    # Create the new interpolated trajectory
    trajectory = alphas * trajectory_via_points[:, inds - 1] + (1.0 - alphas) * trajectory_via_points[:, inds]
    return trajectory.T


def render_urdf(urdf_path, use_rt, simulate, disable_self_collision, fix_root, headless, output_video_path):
    # Generate rendering config
    if not use_rt:
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")
    else:
        sapien.render.set_viewer_shader_dir("rt")
        sapien.render.set_camera_shader_dir("rt")
        sapien.render.set_ray_tracing_samples_per_pixel(64)
        sapien.render.set_ray_tracing_path_depth(8)
        sapien.render.set_ray_tracing_denoiser("oidn")

    # Setup
    engine = sapien.Engine()
    renderer = sapien.render.SapienRenderer()
    engine.set_renderer(renderer)
    config = sapien.SceneConfig()
    config.enable_tgs = True
    config.gravity = np.array([0, 0, 0])
    scene = engine.create_scene(config=config)
    scene.set_timestep(1 / 125)

    # Ground
    render_mat = sapien.render.RenderMaterial()
    render_mat.base_color = [0.06, 0.08, 0.12, 1]
    render_mat.metallic = 0.0
    render_mat.roughness = 0.9
    render_mat.specular = 0.8
    scene.add_ground(-0.5, render_material=render_mat, render_half_size=[1000, 1000])

    # Lighting
    scene.set_ambient_light(np.array([0.6, 0.6, 0.6]))
    scene.add_directional_light(np.array([1, 1, -1]), np.array([2, 2, 2]))
    scene.add_directional_light([0, 0, -1], [2, 2, 2])
    scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
    scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)
    scene.set_environment_map(create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2]))

    # Camera
    cam = scene.add_camera(name="Cheese!", width=1080, height=1080, fovy=1, near=0.1, far=10)
    cam.set_local_pose(sapien.Pose([0.36594, 0.0127696, 0.32213], [0.0260871, 0.386959, 0.0109527, -0.921663]))

    # Viewer
    if not headless:
        viewer = Viewer(renderer)
        viewer.set_scene(scene)
        viewer.control_window.show_origin_frame = False
        viewer.control_window.move_speed = 0.01
        viewer.control_window.focus_camera(cam)
        viewer.control_window._show_camera_linesets = True
    else:
        viewer = None
    record_video = output_video_path is not None

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    if "ability" in urdf_path or "inspire" in urdf_path or "bhand" in urdf_path or "svh" in urdf_path:
        loader.scale = 1.5
    elif "dclaw" in urdf_path:
        loader.scale = 1.25
    elif "allegro" in urdf_path:
        loader.scale = 1.4
    elif "shadow" in urdf_path:
        loader.scale = 1.2
    elif "leap" in urdf_path:
        loader.scale = 1.25
    elif "panda" in urdf_path:
        loader.scale = 1.5

    robot_builder = loader.load_file_as_articulation_builder(urdf_path)
    if disable_self_collision and not simulate:
        for link_builder in robot_builder.get_link_builders():
            link_builder.set_collision_groups(1, 1, 17, 0)
    robot = robot_builder.build(fix_root_link=fix_root, build_mimic_joints=False)
    if "ability" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "shadow" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.4]))
    elif "dclaw" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "allegro" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.05]))
    elif "bhand" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.2]))
    elif "leap" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "svh" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.2]))
    elif "inspire" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.2]))
    elif "panda" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.1]))

    # Robot motion
    loop_steps = 300
    for joint in robot.get_active_joints():
        joint.set_drive_property(200, 10)
    trajectory = generate_joint_limit_trajectory(robot, loop_steps=loop_steps)

    robot.set_qpos(trajectory[0])
    scene.step()

    # Video recorder
    if record_video:
        Path(output_video_path).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (1080, 1080))

    # Rendering
    for qpos in tqdm.tqdm(trajectory):
        if simulate:
            for i, joint in enumerate(robot.get_active_joints()):
                joint.set_drive_target(qpos[i])
            robot.set_qf(robot.compute_passive_force())
            scene.step()
        else:
            robot.set_qpos(qpos)

        angle = np.pi * 2 / loop_steps
        mat = transforms3d.axangles.axangle2mat([0, 0, 1], angle)
        quat = transforms3d.quaternions.mat2quat(mat)
        world_rotate = sapien.Pose(q=quat)
        cam.set_entity_pose(world_rotate * cam.get_entity_pose())

        if not headless:
            viewer.render()

        if record_video:
            scene.update_render()
            cam.take_picture()
            rgb = cam.get_picture("Color")[..., :3]
            rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
            writer.write(rgb[..., ::-1])

    if record_video:
        writer.release()
        print(f"Video generated: {output_video_path}, now convert it to webp.")

        # Convert mp4 to webp
        # Ref: https://gist.github.com/witmin/1edf926c2886d5c8d9b264d70baf7379
        stream = ffmpeg.input(output_video_path)
        stream = ffmpeg.filter(stream, "fps", fps=30, round="up").filter("scale", width="1080", height="1080")
        stream = ffmpeg.output(
            stream,
            output_video_path.replace("mp4", "webp"),
            lossless=1,
            codec="libwebp",
            vsync="0",
            preset="default",
            loop="0",
        )
        ffmpeg.run(stream, overwrite_output=True, quiet=True)

    if not headless:
        viewer.close()
    scene = None


def main(
    urdf_path: str,
    /,
    use_rt: bool = False,
    simulate: bool = True,
    fix_root: bool = True,
    output_video_path: Optional[str] = None,
    headless: bool = False,
    disable_self_collision: bool = False,
):
    """
    Loads the URDF and renders it either on screen or as an MP4 video.

    Args:
        urdf_path: Path to the .urdf file.
        use_rt: Whether to use ray tracing for rendering.
        simulate: Whether to physically simulate the urdf, rather than treat it as animated geometries
        fix_root: Whether to fix the root link of the URDF to the world
        output_video_path: Path where the output video in .mp4 format would be saved.
            By default, it is set to None, implying no video will be saved.
        headless: Set to visualize the rendering on the screen by opening the viewer window.
        disable_self_collision: Whether to disable the self collision of the urdf.
    """
    render_urdf(
        urdf_path=urdf_path,
        use_rt=use_rt,
        simulate=simulate,
        disable_self_collision=disable_self_collision,
        fix_root=fix_root,
        headless=headless,
        output_video_path=output_video_path,
    )


if __name__ == "__main__":
    tyro.cli(main)
