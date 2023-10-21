import argparse
from typing import List

import matplotlib
import numpy as np
import sapien.core as sapien
from sapien.core import renderer as R
from sapien.utils import Viewer

COLOR_MAP = matplotlib.colormaps["Reds"]


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=str, help="Path to the URDF file.")
    parser.add_argument(
        "-rt", "--use_rt", action="store_true", default=False, help="Whether to use ray tracing for rendering."
    )
    parser.add_argument(
        "-s", "--simulate", action="store_true", default=True, help="Whether to physically simulate the urdf."
    )
    parser.add_argument(
        "-f", "--fix-root", action="store_true", default=True, help="Whether to physically simulate the urdf."
    )
    parser.add_argument(
        "--disable-self-collision",
        action="store_true",
        default=False,
        help="Whether to disable the self collision of the urdf.",
    )
    return parser.parse_args()


class ContactViewer(Viewer):
    def __init__(self, renderer: sapien.SapienRenderer, shader_dir="", resolutions=(1024, 768)):
        super().__init__(renderer, shader_dir, resolutions)
        self.contact_nodes = []

        # Material to highlight contact geom
        self.contact_collision_mat = self.renderer.create_material()
        self.contact_collision_mat.base_color = np.array([1, 0, 0, 1])

        # Material of the original collision mesh
        self.default_mesh_mat = self.renderer.create_material()
        self.default_mesh_mat.base_color = np.array([0, 1, 0, 1])
        self.default_primitive_mat = self.renderer.create_material()
        self.default_primitive_mat.base_color = np.array([0, 0, 1, 1])

        # Original material dict to recover the original color of object
        self.highlighted_visual_shape: List[sapien.RenderBody] = []
        self.highlighted_collision: List[sapien.CollisionShape] = []

        self.debug = []

    def highlight_contacted_geom(self, collision_shape: sapien.CollisionShape):
        if collision_shape in self.highlighted_collision:
            return
        actor = collision_shape.actor
        collision_index = actor.get_collision_shapes().index(collision_shape)
        collision_visual_shape = actor.get_collision_visual_bodies()[collision_index]
        self.highlighted_visual_shape.append(collision_visual_shape)
        for render_shape in collision_visual_shape.get_render_shapes():
            render_shape.set_material(self.contact_collision_mat)
        collision_visual_shape.set_visibility(1)

    def draw_contact(self):
        for i in range(len(self.contact_nodes)):
            node = self.contact_nodes.pop()
            self.scene.get_renderer_scene()._internal_scene.remove_node(node)

        contact_list = self.fetch_contact()
        for pos, normal, color in contact_list:
            self.draw_contact_arrow(pos, normal, color)

    def clear_contact(self):
        for i in range(len(self.highlighted_visual_shape)):
            shape = self.highlighted_visual_shape.pop()
            if shape.type == "mesh":
                mat = self.default_mesh_mat
            else:
                mat = self.default_primitive_mat
            for render_shape in shape.get_render_shapes():
                render_shape.set_material(mat)
            shape.set_visibility(0)

    def fetch_contact(self):
        min_impulse = 0.1
        max_impulse = 10
        contact_list = []
        for contact in self.scene.get_contacts():
            reported = False
            for point in contact.points:
                impulse = np.linalg.norm(point.impulse)
                if impulse > min_impulse and point.separation < -5e-4:
                    norm_impulse = (1 / impulse - 1 / min_impulse) / (1 / max_impulse - 1 / min_impulse)
                    color = np.array(COLOR_MAP(norm_impulse))
                    contact_list.append((point.position, point.normal, color))
                    if not reported:
                        print(
                            f"Find self collision: {contact.actor0, contact.actor1},"
                            f" impulse: {impulse}, separation: {point.separation}"
                        )
                        reported = True
                    self.highlight_contacted_geom(contact.collision_shape0)
                    self.highlight_contacted_geom(contact.collision_shape1)

        return contact_list

    @staticmethod
    def compute_rotation_from_normal(normal: np.ndarray):
        # Ref: https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
        normal = normal / np.linalg.norm(normal)
        x, y, z = normal
        if np.isclose(z, 1.0):
            return np.array([0.707, 0, -0.707, 0])
        xy_sqrt = np.sqrt(x * x + y * y)
        y_axis = [y / xy_sqrt, -x / xy_sqrt, 0]
        z_axis = [x * z / xy_sqrt, y * z / xy_sqrt, -xy_sqrt]

        rotation_matrix = np.stack([normal, y_axis, z_axis], axis=1)
        mat44 = np.eye(4)
        mat44[:3, :3] = rotation_matrix
        return sapien.Pose.from_transformation_matrix(mat44).q

    def draw_contact_arrow(self, pos: np.ndarray, normal: np.ndarray, color: np.ndarray):
        rs = self.scene.renderer_scene
        render_scene: R.Scene = rs._internal_scene

        material = self.renderer_context.create_material([0, 0, 0, 0], color, 0, 0.8, 0)
        cone = self.renderer_context.create_model([self.cone], [material])
        capsule = self.renderer_context.create_model([self.capsule], [material])
        contact_node = render_scene.add_node()
        obj = render_scene.add_object(cone, contact_node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 0

        obj = render_scene.add_object(capsule, contact_node)
        obj.set_position([0.5, 0, 0])
        obj.set_scale([1.02, 1.02, 1.02])
        obj.shading_mode = 2
        obj.cast_shadow = False
        obj.transparency = 0

        contact_node.set_position(pos)
        contact_node.set_rotation(self.compute_rotation_from_normal(normal))
        contact_node.set_scale([0.05, 0.05, 0.05])
        self.contact_nodes.append(contact_node)


def generate_joint_limit_trajectory(robot: sapien.Articulation, loop_steps: int):
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


def visualize_urdf(use_rt, urdf_file, simulate, disable_self_collision, fix_root):
    # Generate rendering config
    render_config = {}
    if not use_rt:
        render_config.update(dict(camera_shader_dir="ibl", viewer_shader_dir="ibl"))
    else:
        render_config.update(
            dict(
                camera_shader_dir="rt",
                viewer_shader_dir="rt",
                rt_samples_per_pixel=32,
                rt_max_path_depth=8,
                rt_use_denoiser=True,
            )
        )
    for k, v in render_config.items():
        setattr(sapien.render_config, k, v)

    # Setup
    engine = sapien.Engine()
    renderer = sapien.SapienRenderer(offscreen_only=False)
    engine.set_renderer(renderer)
    config = sapien.SceneConfig()
    config.gravity = np.array([0, 0, 0])
    scene = engine.create_scene(config=config)
    scene.set_timestep(1 / 125)

    # Ground
    render_mat = renderer.create_material()
    render_mat.base_color = [0.06, 0.08, 0.12, 1]
    render_mat.metallic = 0.0
    render_mat.roughness = 0.9
    render_mat.specular = 0.8
    scene.add_ground(-1, render_material=render_mat)

    # Lighting
    scene.set_ambient_light(np.array([0.6, 0.6, 0.6]))
    scene.add_directional_light(np.array([1, 1, -1]), np.array([1, 1, 1]))
    scene.add_directional_light([0, 0, -1], [1, 1, 1])
    scene.add_point_light(np.array([2, 2, 2]), np.array([1, 1, 1]), shadow=False)
    scene.add_point_light(np.array([2, -2, 2]), np.array([1, 1, 1]), shadow=False)

    # Viewer
    viewer = ContactViewer(renderer)
    viewer.set_scene(scene)
    viewer.set_camera_xyz(0.5, 0, 0.5)
    viewer.set_camera_rpy(0, -0.8, 3.14)
    viewer.toggle_axes(0)
    viewer.set_move_speed(0.01)

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    robot_builder = loader.load_file_as_articulation_builder(urdf_file)
    if disable_self_collision and not simulate:
        for link_builder in robot_builder.get_link_builders():
            link_builder.set_collision_groups(1, 1, 17, 0)
    robot = robot_builder.build(fix_root_link=fix_root)

    # Robot motion
    loop_steps = 600
    trajectory = np.array([])
    if simulate:
        for joint in robot.get_active_joints():
            joint.set_drive_property(10000, 500, 10000)
        trajectory = generate_joint_limit_trajectory(robot, loop_steps=loop_steps)

    robot.set_qpos(np.zeros([robot.dof]))
    scene.step()

    step = 0
    while not viewer.closed:
        viewer.render()
        if simulate:
            viewer.clear_contact()
            qpos = trajectory[step]
            robot.set_drive_target(qpos)
            robot.set_qf(robot.compute_passive_force(external=False))
            scene.step()
            step += 1
            step = step % loop_steps

            # Draw the contact information on the viewer
            # We are using the "Reds" colormap to indicate the magnitude of the contact force
            # Stronger red hue means larger contact force
            viewer.draw_contact()


def main():
    args = parse_args()
    visualize_urdf(args.use_rt, args.urdf, args.simulate, args.disable_self_collision, args.fix_root)


if __name__ == "__main__":
    main()
