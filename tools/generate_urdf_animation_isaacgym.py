from pathlib import Path
from typing import Optional

import cv2
import ffmpeg
import numpy as np
import tqdm
import tyro
from isaacgym import gymapi


def generate_joint_limit_trajectory(joint_limits: np.ndarray, loop_steps: int):
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


def render_urdf(urdf_path, simulate, disable_self_collision, fix_root, headless, output_video_path):
    # initialize gym
    gym = gymapi.acquire_gym()

    # create a simulator
    sim_params = gymapi.SimParams()
    sim_params.substeps = 4
    sim_params.dt = 1.0 / 120.0
    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 0
    sim_params.physx.use_gpu = False
    sim_params.up_axis = gymapi.UP_AXIS_Z

    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

    if sim is None:
        print("*** Failed to create sim")
        quit()

    # create viewer using the default camera properties
    if not headless:
        viewer = gym.create_viewer(sim, gymapi.CameraProperties())
        if viewer is None:
            raise ValueError("*** Failed to create viewer")
    else:
        viewer = None
    record_video = output_video_path is not None

    # add ground plane
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)
    plane_params.distance = 0.5
    gym.add_ground(sim, plane_params)
    # gym.set_light_parameters(sim, 0, gymapi.Vec3(1, 1, 1), gymapi.Vec3(1, 1, 1), gymapi.Vec3(1, 1, -1))
    # gym.set_light_parameters(sim, 1, gymapi.Vec3(1, 1, 1), gymapi.Vec3(0.6, 0.6, 0.6), gymapi.Vec3(0, 0, -1))
    # gym.set_light_parameters(sim, 2, gymapi.Vec3(1, 1, 1), gymapi.Vec3(0, 0, 0), gymapi.Vec3(0, 1, -1))
    # gym.set_light_parameters(sim, 3, gymapi.Vec3(1, 1, 1), gymapi.Vec3(0, 0, 0), gymapi.Vec3(1, 0, -1))

    # set up the env grid
    spacing = 2
    env_lower = gymapi.Vec3(-spacing, 0.0, -spacing)
    env_upper = gymapi.Vec3(spacing, 0.0, spacing)

    # add cartpole urdf asset
    asset_path = Path(urdf_path)
    asset_root = str(asset_path.parent.parent)
    asset_name = asset_path.stem
    asset_file = f"{asset_path.parent.name}/{asset_path.name}"

    # Load asset with default control type of position for all joints
    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = fix_root
    asset_options.flip_visual_attachments = False
    asset_options.convex_decomposition_from_submeshes = True
    asset_options.disable_gravity = True
    asset = gym.load_asset(sim, asset_root, asset_file, asset_options)
    num_dof = gym.get_asset_dof_count(asset)
    dof_prop = gym.get_asset_dof_properties(asset)
    joint_limits = np.stack([dof_prop["lower"], dof_prop["upper"]], axis=1)

    # Create actor
    env = gym.create_env(sim, env_lower, env_upper, 2)
    initial_pose = gymapi.Transform(gymapi.Vec3(0, 0, 0), gymapi.Quat(0, 0, 0, 1))
    if "shadow" in urdf_path:
        initial_pose.p.z = -0.3

    if disable_self_collision:
        actor = gym.create_actor(env, asset, initial_pose, asset_name, 0, 1)
    else:
        actor = gym.create_actor(env, asset, initial_pose, asset_name, 0, 0)

    # Set actor DoF position
    dof_states = gym.get_actor_dof_states(env, actor, gymapi.STATE_ALL)
    init_qpos = np.clip(np.zeros(num_dof), joint_limits[:, 0], joint_limits[:, 1])
    dof_states["pos"] = init_qpos
    props = gym.get_actor_dof_properties(env, actor)
    props["driveMode"] = (gymapi.DOF_MODE_POS,) * num_dof
    props["stiffness"] = (100,) * num_dof
    props["damping"] = (10,) * num_dof
    gym.set_actor_dof_properties(env, actor, props)

    # Create trajectory
    loop_steps = 300
    trajectory = np.array([])
    joint_limits[~dof_prop["hasLimits"]] = np.array([0, np.pi * 2])[None, :]
    trajectory = generate_joint_limit_trajectory(joint_limits, loop_steps=loop_steps)

    if not headless:
        cam_pos = gymapi.Vec3(0.5, 0, 0.5)
        cam_target = gymapi.Vec3(0, 0, 0)
        gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

    if record_video:
        cam_prop = gymapi.CameraProperties()
        cam_prop.width = 1080
        cam_prop.height = 1080
        cam_prop.enable_tensors = False
        cam_prop.horizontal_fov = np.rad2deg(1)
        cam = gym.create_camera_sensor(env, cam_prop)
        cam_quat = gymapi.Quat(0.386959, 0.0109527, -0.921663, 0.0260871)
        cam_pos = gymapi.Vec3(0.36594, 0.0127696, 0.32213)
        cam_pose = gymapi.Transform(cam_pos, cam_quat)
        gym.set_camera_transform(cam, env, cam_pose)

    # Set joint position and position target
    gym.set_actor_dof_states(env, actor, dof_states, gymapi.STATE_POS)
    for i in range(num_dof):
        gym.set_dof_target_position(env, i, init_qpos[i])

    gym.simulate(sim)

    # Video recorder
    if record_video:
        Path(output_video_path).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(output_video_path, cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (1080, 1080))

    # Simulate
    step = 0
    for qpos in tqdm.tqdm(trajectory):
        if simulate:
            # Step the physics
            for i in range(num_dof):
                gym.set_dof_target_position(env, i, qpos[i])
            gym.simulate(sim)
            step += 1
            step = step % loop_steps
        else:
            dof_state = gym.get_actor_dof_states(env, actor, gymapi.STATE_POS)
            dof_state["pos"] = qpos
            gym.set_actor_dof_states(env, actor, dof_state, gymapi.STATE_POS)
        gym.fetch_results(sim, True)
        gym.step_graphics(sim)

        # Update the viewer
        if not headless:
            gym.draw_viewer(viewer, sim, True)
            gym.sync_frame_time(sim)

        # Update the video
        if record_video:
            gym.render_all_camera_sensors(sim)
            rgb = gym.get_camera_image(sim, env, cam, gymapi.IMAGE_COLOR).reshape([1080, -1, 4])[..., :3]
            writer.write(rgb[..., ::-1])

        angle = np.pi * 2 / loop_steps
        world_rot = gymapi.Quat.from_axis_angle(gymapi.Vec3(0, 0, 1), angle)
        world_pose = gymapi.Transform(r=world_rot)
        cam_pose = world_pose * cam_pose
        gym.set_camera_transform(cam, env, cam_pose)

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
        ffmpeg.run(stream, overwrite_output=True, quiet=False)

    if not headless:
        gym.destroy_viewer(viewer)
    gym.destroy_sim(sim)


def main(
    urdf_path: str,
    /,
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
        simulate: Whether to physically simulate the urdf, rather than treat it as animated geometries
        fix_root: Whether to fix the root link of the URDF to the world
        output_video_path: Path where the output video in .mp4 format would be saved.
            By default, it is set to None, implying no video will be saved.
        headless: Set to visualize the rendering on the screen by opening the viewer window.
        disable_self_collision: Whether to disable the self collision of the urdf.
    """
    render_urdf(
        urdf_path=urdf_path,
        simulate=simulate,
        disable_self_collision=disable_self_collision,
        fix_root=fix_root,
        headless=headless,
        output_video_path=output_video_path,
    )


if __name__ == "__main__":
    tyro.cli(main)
