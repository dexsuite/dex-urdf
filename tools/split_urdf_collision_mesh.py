from xml.dom import minidom
from xml.etree import ElementTree as ET

import sapien
import trimesh
import tyro
from pathlib import Path


def parse_urdf(urdf_path, new_mesh_dir):
    # Setup
    scene = sapien.Scene()

    # Articulation
    loader = scene.create_urdf_loader()
    loader.load_multiple_collisions_from_file = True
    urdf_path = Path(urdf_path)
    robot = loader.load(str(urdf_path))
    new_mesh_dir = urdf_path.parent / new_mesh_dir
    new_mesh_dir.mkdir(exist_ok=True)

    # URDF XML
    with urdf_path.open("r") as f:
        urdf_string = f.read()
        urdf = ET.fromstring(urdf_string.encode("utf-8"))

    for link in robot.get_links():
        link_xml = urdf.find(f'.//link[@name="{link.name}"]')
        # remove original collision mesh tag
        collision_mesh_xml_list = link_xml.findall(f"./collision/geometry/mesh/....")
        for collision_mesh_xml in collision_mesh_xml_list:
            link_xml.remove(collision_mesh_xml)

        for k, collision_shape in enumerate(link.get_collision_shapes()):
            trimesh_scene = trimesh.Scene()
            if isinstance(collision_shape, sapien.physx.PhysxCollisionShapeConvexMesh):
                mesh = trimesh.Trimesh(
                    vertices=collision_shape.vertices * collision_shape.scale, faces=collision_shape.triangles
                )
                mesh.apply_transform(collision_shape.local_pose.to_transformation_matrix())
                trimesh_scene.add_geometry(mesh)

                # Add to urdf
                collision_name = f"{link.name}-{k}"
                convex_xml = ET.Element("collision", attrib=dict(name=collision_name))
                geometry = ET.Element("geometry")
                mesh = ET.Element("mesh", attrib=dict(filename=f"{new_mesh_dir.stem}/{collision_name}.obj"))
                geometry.append(mesh)
                convex_xml.append(geometry)
                link_xml.append(convex_xml)

                # Export mesh
                filename = Path(new_mesh_dir) / f"{collision_name}.obj"
                trimesh_scene.export(str(filename))

    urdf_path = Path(urdf_path)
    filename = urdf_path.stem
    new_urdf_path = urdf_path.parent / f"{filename}_split_mesh.urdf"
    with new_urdf_path.open("w") as f:
        rough_string = ET.tostring(urdf, encoding="utf8").decode()
        format_string = minidom.parseString(rough_string)
        f.write(format_string.toprettyxml(indent="\t", newl="\t"))


def main(urdf_path: str, new_mesh_dir: str, /):
    """
    Loads the URDF and renders it either on screen or as an MP4 video.

    Args:
        urdf_path: Path to the .urdf file.
        new_mesh_dir: Path where the output collision mesh in split .obj format would be saved.
    """
    parse_urdf(
        urdf_path=urdf_path,
        new_mesh_dir=new_mesh_dir,
    )


if __name__ == "__main__":
    tyro.cli(main)
