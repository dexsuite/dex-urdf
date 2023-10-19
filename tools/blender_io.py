import bpy
from pathlib import Path

IS_EXPORT = True

root_dir = Path("")
if not IS_EXPORT:
    # Import all obj file in a directory to blender scene
    for file in root_dir.glob("*.obj"):
        if "_C" not in str(file):
            print(file)
            imported_object = bpy.ops.import_scene.obj(filepath=str(file))


else:
    # After modeling the visual material for each mesh in the scene
    scene = bpy.data.scenes["Scene"]
    bpy.context.active_object.select_set(False)
    for name, obj in scene.objects.items():
        obj.select_set(True)
        print(name)
        bpy.ops.export_scene.obj(filepath=str(root_dir / f"{name}.obj"), use_selection=True)
        bpy.ops.export_scene.gltf(filepath=str(root_dir / f"{name}.glb"), use_selection=True, check_existing=False)
        obj.select_set(False)
