from pathlib import Path

import bpy

IS_EXPORT = True
IS_OBJ = True

FILE_TYPE = "glb"
root_dir = Path("")

if not IS_EXPORT:
    # Import all obj file in a directory to blender scene
    for file in root_dir.glob(f"*.{FILE_TYPE}"):
        print(file)
        bpy.ops.import_scene.gltf(filepath=str(file))

else:
    # After modeling the visual material for each mesh in the scene
    scene = bpy.data.scenes["Scene"]
    bpy.context.active_object.select_set(False)
    for name, obj in scene.objects.items():
        obj.select_set(True)
        print(name)
        bpy.ops.wm.usd_export(filepath=str(root_dir / f"{name}.usd"), selected_objects_only=True, export_armatures=False)
        obj.select_set(False)
