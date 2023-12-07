from pathlib import Path

import numpy as np
import sapien
import trimesh
import tyro

CONVERT_PRIMITIVE = False


def parse_urdf(urdf_path, output_dir):
    # Setup
    engine = sapien.Engine()
    renderer = sapien.render.SapienRenderer(offscreen_only=False)
    engine.set_renderer(renderer)
    config = sapien.SceneConfig()
    config.enable_tgs = True
    config.gravity = np.array([0, 0, 0])
    scene = engine.create_scene(config=config)
    scene.set_timestep(1 / 125)

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    robot = loader.load(urdf_path)

    for link in robot.get_links():
        trimesh_scene = trimesh.Scene()
        for collision_shape in link.get_collision_shapes():
            no_mesh = False
            if CONVERT_PRIMITIVE:
                if isinstance(collision_shape, sapien.physx.PhysxCollisionShapeSphere):
                    mesh = trimesh.creation.icosphere(radius=collision_shape.radius)

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeBox):
                    mesh = trimesh.creation.box(extents=collision_shape.half_size * 2)

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCapsule):
                    mesh = trimesh.creation.capsule(
                        radius=collision_shape.radius, height=collision_shape.half_length * 2
                    )

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeConvexMesh):
                    mesh = trimesh.Trimesh(
                        vertices=collision_shape.vertices * collision_shape.scale, faces=collision_shape.triangles
                    )

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeTriangleMesh):
                    mesh = trimesh.Trimesh(
                        vertices=collision_shape.vertices * collision_shape.scale, faces=collision_shape.triangles
                    )

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapePlane):
                    mesh = trimesh.creation.box([1, 1e4, 1e4])

                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeCylinder):
                    mesh = trimesh.creation.cylinder(
                        radius=collision_shape.radius, height=collision_shape.half_length * 2
                    )

                else:
                    raise Exception("invalid collision shape, this code should be unreachable.")
            else:
                if isinstance(collision_shape, sapien.physx.PhysxCollisionShapeConvexMesh):
                    mesh = trimesh.Trimesh(
                        vertices=collision_shape.vertices * collision_shape.scale, faces=collision_shape.triangles
                    )
                elif isinstance(collision_shape, sapien.physx.PhysxCollisionShapeTriangleMesh):
                    mesh = trimesh.Trimesh(
                        vertices=collision_shape.vertices * collision_shape.scale, faces=collision_shape.triangles
                    )
                else:
                    no_mesh = True

            if not no_mesh:
                mesh.apply_transform(collision_shape.local_pose.to_transformation_matrix())
                trimesh_scene.add_geometry(mesh)
        if len(trimesh_scene.geometry) > 0:
            filename = Path(output_dir) / f"{link.name}.obj"
            trimesh_scene.export(str(filename))
        else:
            print(f"Skip pure primitive link: {link.name}")


def main(urdf_path: str, output_dir: str, /):
    """
    Loads the URDF and renders it either on screen or as an MP4 video.

    Args:
        urdf_path: Path to the .urdf file.
        output_dir: Path where the output collision mesh in .obj format would be saved.
    """
    parse_urdf(
        urdf_path=urdf_path,
        output_dir=output_dir,
    )


if __name__ == "__main__":
    tyro.cli(main)
