import bpy
from pathlib import Path

IS_EXPORT = True
IS_OBJ = True

file_type = "dae" if not IS_OBJ else "obj"

root_dir = Path("")
if not IS_EXPORT:
    # Import all obj file in a directory to blender scene
    for file in root_dir.glob(f"*.{file_type}"):
        print(file)
        if IS_OBJ:
            imported_object = bpy.ops.wm.obj_import(filepath=str(file))
        else:
            bpy.ops.wm.collada_import(filepath=str(file))

        # bpy.ops.import_scene.gltf(filepath=str(file))


else:
    # After modeling the visual material for each mesh in the scene
    scene = bpy.data.scenes["Scene"]
    bpy.context.active_object.select_set(False)
    for name, obj in scene.objects.items():
        obj.select_set(True)
        print(name)
        bpy.ops.wm.obj_export(
            filepath=str(root_dir / f"{name}.obj"), export_selected_objects=True, forward_axis="Y", up_axis="Z"
        )
        bpy.ops.export_scene.gltf(
            filepath=str(root_dir / f"{name}.glb"),
            use_selection=True,
            check_existing=False,
            export_yup=False,
            export_animations=False,
            export_morph=False,
        )
        obj.select_set(False)
