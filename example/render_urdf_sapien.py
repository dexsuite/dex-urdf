import sapien.core as sapien
import argparse
from typing import List, Dict, Tuple

import matplotlib
import numpy as np
import sapien
import transforms3d
from sapien import internal_renderer as R
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
    def __init__(self, renderer: sapien.render.SapienRenderer = None, shader_dir="", resolutions=(1024, 768)):
        super().__init__(renderer, shader_dir, resolutions)

        # Contact arrow
        self.contact_nodes = []
        self.capsule = self.renderer_context.create_capsule_mesh(0.1, 0.5, 16, 4)
        self.cone = self.renderer_context.create_cone_mesh(16)

        # Material to highlight contact geom
        self.contact_collision_mat = sapien.render.RenderMaterial()
        self.contact_collision_mat.base_color = np.array([1, 0, 0, 1])

        self.highlighted_visual_body: List[sapien.render.RenderBodyComponent] = []

    def draw_contact(self):
        # Clear contact arrow in the previous step
        for i in range(len(self.contact_nodes)):
            node = self.contact_nodes.pop()
            self.render_scene.remove_node(node)

        # Remove previous collision visual shape
        for visual_body in self.highlighted_visual_body:
            entity = visual_body.get_entity()
            entity.remove_component(visual_body)
        self.highlighted_visual_body.clear()

        # Fetch the contact information for current step
        contact_list, actor_geom_map = self.fetch_contact()

        # Draw collision visual shape
        for rigid_body, geom_list in actor_geom_map.items():
            entity = rigid_body.get_entity()
            for c in entity.get_components():
                if isinstance(c, sapien.render.RenderBodyComponent):
                    # Turn off collision mesh if it is in show_collision mode
                    if c.name == "Collision":
                        c.disable()
                    # Turn on visual mesh if it is in show_collision mode
                    else:
                        c.enable()
            contact_shape = self.build_collision_visual_shape(geom_list)
            entity.add_component(contact_shape)
            contact_shape.set_property("shadeFlat", 1)
            self.highlighted_visual_body.append(contact_shape)

        # Draw contact arrow
        for pos, normal, color in contact_list:
            self.draw_contact_arrow(pos, normal, color)

    def fetch_contact(
        self,
    ) -> Tuple[List, Dict[sapien.physx.PhysxRigidBaseComponent, List[sapien.physx.PhysxCollisionShape]]]:
        min_impulse = 0.1
        max_impulse = 10
        contact_list = []

        body_geom_map = {}
        for contact in self.scene.get_contacts():
            impulse = np.array([p.impulse for p in contact.points])
            total_impulse = np.linalg.norm(np.sum(impulse, axis=0))
            impulse_value = np.linalg.norm(impulse, axis=1)
            if total_impulse > min_impulse:
                weight = impulse_value / np.sum(impulse_value)
                position = np.sum(np.array([p.position for p in contact.points]) * weight[:, None], axis=0)
                norm_impulse = np.clip(
                    (1 / total_impulse - 1 / min_impulse) / (1 / max_impulse - 1 / min_impulse), 0, 1
                )
                color = np.array(COLOR_MAP(norm_impulse))
                contact_list.append((position, np.sum(impulse, axis=0), color))
                body0, body1 = contact.bodies[0:2]
                print(
                    f"Find self collision: {body0.get_name(), body1.get_name()},"
                    f" impulse: {total_impulse}, position: {position}"
                )

                for actor, shape in zip([body0, body1], [contact.shapes[0], contact.shapes[1]]):
                    if actor in body_geom_map:
                        body_geom_map[actor].append(shape)
                    else:
                        body_geom_map[actor] = [shape]

        return contact_list, body_geom_map

    @staticmethod
    def compute_rotation_from_normal(normal: np.ndarray):
        # Ref: https://math.stackexchange.com/questions/1956699/getting-a-transformation-matrix-from-a-normal-vector
        normal = normal / np.linalg.norm(normal)
        x, y, z = normal
        if np.isclose(z, 1.0):
            return np.array([0.707, 0, -0.707, 0])
        elif np.isclose(z, -1.0):
            return np.array([0.707, 0, 0.707, 0])
        xy_sqrt = np.sqrt(x * x + y * y)
        y_axis = [y / xy_sqrt, -x / xy_sqrt, 0]
        z_axis = [x * z / xy_sqrt, y * z / xy_sqrt, -xy_sqrt]

        rotation_matrix = np.stack([normal, y_axis, z_axis], axis=1)
        quat = transforms3d.quaternions.mat2quat(rotation_matrix)
        return quat

    def draw_contact_arrow(self, pos: np.ndarray, normal: np.ndarray, color: np.ndarray):
        render_scene: R.Scene = self.render_scene

        material = self.renderer_context.create_material([1, 0, 0, 0], color, 0, 0.8, 0)
        cone = self.renderer_context.create_model([self.cone], [material])
        capsule = self.renderer_context.create_model([self.capsule], [material])
        contact_node = render_scene.add_node()
        obj = render_scene.add_object(cone, contact_node)
        obj.set_scale([0.5, 0.2, 0.2])
        obj.set_position([1, 0, 0])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 0

        obj = render_scene.add_object(capsule, contact_node)
        obj.set_position([0.5, 0, 0])
        obj.shading_mode = 0
        obj.cast_shadow = False
        obj.transparency = 0

        contact_node.set_position(pos)
        contact_node.set_rotation(self.compute_rotation_from_normal(normal))
        contact_node.set_scale([0.05, 0.05, 0.05])
        self.contact_nodes.append(contact_node)

    def build_collision_visual_shape(
        self,
        collision_shape_list: List[sapien.physx.PhysxCollisionShape],
    ) -> sapien.render.RenderBodyComponent:
        new_visual = sapien.render.RenderBodyComponent()
        new_visual.disable_render_id()
        new_visual.name = "ContactShape"

        for collision_shape in collision_shape_list:
            if isinstance(collision_shape, sapien.physx.PhysxCollisionShapeSphere):
                vs = sapien.render.RenderShapeSphere(collision_shape.radius, self.contact_collision_mat)

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeBox):
                vs = sapien.render.RenderShapeBox(collision_shape.half_size, self.contact_collision_mat)

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCapsule):
                vs = sapien.render.RenderShapeCapsule(
                    collision_shape.radius, collision_shape.half_length, self.contact_collision_mat
                )

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeConvexMesh):
                vs = sapien.render.RenderShapeTriangleMesh(
                    collision_shape.vertices,
                    collision_shape.triangles,
                    np.zeros((0, 3)),
                    np.zeros((0, 2)),
                    self.contact_collision_mat,
                )
                vs.scale = collision_shape.scale

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeTriangleMesh):
                vs = sapien.render.RenderShapeTriangleMesh(
                    collision_shape.vertices,
                    collision_shape.triangles,
                    np.zeros((0, 3)),
                    np.zeros((0, 2)),
                    self.contact_collision_mat,
                )
                vs.scale = collision_shape.scale

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapePlane):
                vs = sapien.render.RenderShapePlane([1, 1e4, 1e4], self.contact_collision_mat)

            elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCylinder):
                vs = sapien.render.RenderShapeCylinder(
                    collision_shape.radius, collision_shape.half_length, self.contact_collision_mat
                )

            else:
                raise Exception("invalid collision shape, this code should be unreachable.")

            vs.local_pose = collision_shape.local_pose
            new_visual.attach(vs)
        # new_visual.set_property("shadeFlat", 1)
        return new_visual


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


def visualize_urdf(use_rt, urdf_file, simulate, disable_self_collision, fix_root):
    # Generate rendering config
    if not use_rt:
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")
    else:
        sapien.render.set_viewer_shader_dir("rt")
        sapien.render.set_camera_shader_dir("rt")
        sapien.render.set_ray_tracing_samples_per_pixel(1024)
        sapien.render.set_ray_tracing_path_depth(16)
        sapien.render.set_ray_tracing_denoiser("oidn")

    # Setup
    scene = sapien.Scene()
    sapien.physx.set_scene_config(enable_tgs=True)
    scene.set_timestep(1 / 125)

    # Ground
    render_mat = sapien.render.RenderMaterial()
    render_mat.base_color = [0.06, 0.08, 0.12, 1]
    render_mat.metallic = 0.0
    render_mat.roughness = 0.9
    render_mat.specular = 0.8
    scene.add_ground(-1.5, render_material=render_mat, render_half_size=[1000000, 1000000])

    # Lighting
    scene.set_ambient_light(np.array([0.6, 0.6, 0.6]))
    scene.add_directional_light(np.array([1, 1, -1]), np.array([1, 1, 1]))
    scene.add_directional_light([0, 0, -1], [1, 1, 1])
    scene.add_point_light(np.array([2, 2, 2]), np.array([1, 1, 1]), shadow=False)
    scene.add_point_light(np.array([2, -2, 2]), np.array([1, 1, 1]), shadow=False)

    # Viewer
    viewer = ContactViewer()
    viewer.set_scene(scene)
    viewer.control_window.set_camera_xyz(0.5, 0, 0.5)
    viewer.control_window.set_camera_rpy(0, -0.8, 3.14)
    viewer.control_window.show_origin_frame = False
    viewer.control_window.move_speed = 0.01

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    robot_builder = loader.load_file_as_articulation_builder(urdf_file)
    if disable_self_collision:
        for link_builder in robot_builder.get_link_builders():
            link_builder.set_collision_groups(1, 1, 17, 0)
    robot: sapien.physx.PhysxArticulation = robot_builder.build(fix_root_link=fix_root)
    for link in robot.get_links():
        link.disable_gravity = True
        if disable_self_collision:
            for shape in link.get_collision_shapes():
                shape.set_collision_groups([1, 1, 17, 0])

    # Robot motion
    loop_steps = 600
    trajectory = np.array([])
    if simulate:
        for joint in robot.get_active_joints():
            joint.set_drive_property(1000, 50)
        trajectory = generate_joint_limit_trajectory(robot, loop_steps=loop_steps)

    robot.set_qpos(np.zeros([robot.dof]))

    cam = scene.add_camera(name="Cheese!", width=1080, height=1080, fovy=1.2, near=0.1, far=10)
    cam.set_local_pose(sapien.Pose([0.307479, 0.0254082, 0.115112], [-0.0016765, 0.176569, -0.000300482, -0.984287]))

    step = 0
    while not viewer.closed:
        viewer.render()
        if simulate:
            qpos = trajectory[step]
            for joint, single_qpos in zip(robot.get_active_joints(), qpos):
                joint.set_drive_target(single_qpos)
            robot.set_qf(robot.compute_passive_force())
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
