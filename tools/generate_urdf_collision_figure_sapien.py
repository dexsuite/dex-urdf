import sapien.core as sapien
from pathlib import Path
from typing import Optional, List

import cv2
import numpy as np
import sapien
import tyro
from sapien import Pose
from sapien.utils import Viewer


def build_collision_visual_shape(
    collision_shape_list: List[sapien.physx.PhysxCollisionShape],
) -> sapien.render.RenderBodyComponent:
    new_visual = sapien.render.RenderBodyComponent()
    new_visual.disable_render_id()
    new_visual.name = "visual_collision_shape"

    primitive_mat = sapien.render.RenderMaterial()
    primitive_mat.base_color = np.array([123, 211, 234, 255]) / 255
    convex_mat = sapien.render.RenderMaterial()
    convex_mat.base_color = np.array([161, 238, 189, 255]) / 255

    for collision_shape in collision_shape_list:
        if isinstance(collision_shape, sapien.physx.PhysxCollisionShapeSphere):
            vs = sapien.render.RenderShapeSphere(collision_shape.radius, primitive_mat)

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeBox):
            vs = sapien.render.RenderShapeBox(collision_shape.half_size, primitive_mat)

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCapsule):
            vs = sapien.render.RenderShapeCapsule(collision_shape.radius, collision_shape.half_length, primitive_mat)

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeConvexMesh):
            m = collision_shape.vertices.shape[0]
            vs = sapien.render.RenderShapeTriangleMesh(
                collision_shape.vertices, collision_shape.triangles, np.zeros((0, 3)), np.zeros((0, 2)), convex_mat
            )
            vs.scale = collision_shape.scale

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeTriangleMesh):
            vs = sapien.render.RenderShapeTriangleMesh(
                collision_shape.vertices, collision_shape.triangles, np.zeros((0, 3)), convex_mat
            )
            vs.scale = collision_shape.scale

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapePlane):
            vs = sapien.render.RenderShapePlane([1, 1e4, 1e4], primitive_mat)

        elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCylinder):
            vs = sapien.render.RenderShapeCylinder(collision_shape.radius, collision_shape.half_length, primitive_mat)

        else:
            raise Exception("invalid collision shape, this code should be unreachable.")

        vs.local_pose = collision_shape.local_pose
        new_visual.attach(vs)
    return new_visual


def render_urdf(urdf_path, fix_root, disable_self_collision, headless, output_image_path):
    # Generate rendering config
    if not headless:
        sapien.render.set_viewer_shader_dir("default")
        sapien.render.set_camera_shader_dir("default")
    else:
        sapien.render.set_camera_shader_dir("shadow_catcher")
        sapien.render.set_ray_tracing_denoiser("oidn")
        sapien.render.set_ray_tracing_samples_per_pixel(1024)
        sapien.render.set_ray_tracing_path_depth(8)

    # Setup
    config = sapien.SceneConfig()
    config.enable_tgs = True
    config.gravity = np.array([0, 0, 0])
    sapien.physx.set_scene_config(config)
    scene = sapien.Scene()
    scene.set_timestep(1 / 125)

    # Ground
    render_mat = sapien.render.RenderMaterial()
    render_mat.base_color = [0.3, 0.3, 0.3, 1]
    render_mat.roughness = 0.9
    render_mat.specular = 0.8
    render_mat.metallic = 0.0
    scene.add_ground(-0.2, render_material=render_mat, render_half_size=[1000, 1000])

    # Lighting
    scene.set_ambient_light(np.array([0.8, 0.8, 0.8]))
    scene.add_area_light_for_ray_tracing(sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]), np.array([1, 1, 1]), 3, 3)

    # Camera

    cam = scene.add_camera(name="Cheese!", width=1080, height=1080, fovy=1.2, near=0.1, far=10)
    cam.set_local_pose(Pose([0.307479, 0.0254082, 0.115112], [-0.0016765, 0.176569, -0.000300482, -0.984287]))

    # Viewer
    if not headless:
        viewer = Viewer()
        viewer.set_scene(scene)
        viewer.control_window.show_origin_frame = False
        viewer.control_window.move_speed = 0.01
        viewer.control_window.focus_camera(cam)
        viewer.control_window._show_camera_linesets = True
    else:
        viewer = None
    capture_image = output_image_path is not None

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    if "ability" in urdf_path:
        loader.scale = 1.7
    elif "dclaw" in urdf_path:
        loader.scale = 1.25
    elif "allegro" in urdf_path:
        loader.scale = 1.4
    elif "shadow" in urdf_path:
        loader.scale = 1.2
    elif "bhand" in urdf_path:
        loader.scale = 1.5
    elif "leap" in urdf_path:
        loader.scale = 1.25
    elif "svh" in urdf_path:
        loader.scale = 1.5
    elif "inspire" in urdf_path:
        loader.scale = 1.5
    elif "panda" in urdf_path:
        loader.scale = 1.5

    robot_builder = loader.load_file_as_articulation_builder(urdf_path)
    if disable_self_collision:
        for link_builder in robot_builder.get_link_builders():
            link_builder.set_collision_groups(1, 1, 17, 0)
    robot = robot_builder.build(fix_root_link=fix_root)

    qpos = np.zeros(robot.dof)
    if "ability" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "shadow" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.35]))
    elif "dclaw" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15], [0.9659258, 0, 0, 0.258819]))
    elif "allegro" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.07]))
        qpos += np.ones(robot.dof) * 0.2
        qpos[[0, 4, 8, 12]] = 0
    elif "bhand" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
        qpos = np.array([0, -1, 0, 0, -1, 0, -1, 0])
    elif "leap" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "svh" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "inspire" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.15]))
    elif "panda" in urdf_path:
        robot.set_pose(sapien.Pose([0, 0, -0.1]))

    # Robot visual
    for link in robot.get_links():
        entity = link.get_entity()
        for c in entity.get_components():
            if isinstance(c, sapien.render.RenderBodyComponent):
                c.disable()
            if isinstance(c, sapien.physx.PhysxArticulationLinkComponent):
                collision_shape = c.get_collision_shapes()
                if len(collision_shape) > 0:
                    collision_visual = build_collision_visual_shape(collision_shape)
                    entity.add_component(collision_visual)
                    collision_visual.set_property("shadeFlat", 1)

    # Robot motion
    robot.set_qpos(qpos)

    # Video recorder
    if capture_image:
        Path(output_image_path).parent.mkdir(parents=True, exist_ok=True)

    # Rendering
    if headless:
        scene.update_render()
        cam.take_picture()
        rgb = cam.get_picture("Color")
        rgb = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGBA2BGRA)
        cv2.imwrite(output_image_path, bgr)
    else:
        while not viewer.closed:
            viewer.render()

    if not headless:
        viewer.close()
    scene = None


def main(
    urdf_path: str,
    /,
    fix_root: bool = True,
    output_image_path: Optional[str] = None,
    headless: bool = False,
    disable_self_collision: bool = False,
):
    """
    Loads the URDF and renders it either on screen or as an MP4 video.

    Args:
        urdf_path: Path to the .urdf file.
        fix_root: Whether to fix the root link of the URDF to the world
        output_image_path: Path where the output image in .png format would be saved.
            By default, it is set to None, implying no image will be saved.
        headless: Set to visualize the rendering on the screen by opening the viewer window.
        disable_self_collision: Whether to disable the self collision of the urdf.
    """
    render_urdf(
        urdf_path=urdf_path,
        fix_root=fix_root,
        disable_self_collision=disable_self_collision,
        headless=headless,
        output_image_path=output_image_path,
    )


if __name__ == "__main__":
    tyro.cli(main)
