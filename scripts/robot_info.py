from pathlib import Path

ORIGINAL_ROBOT_NAME_MAP = {
    "ability": "hands/ability_hand/ability_hand_right_glb.urdf",
    "allegro": "hands/allegro_hand/allegro_hand_right_glb.urdf",
    "barrett": "hands/barrett_hand/bhand_model_glb.urdf",
    "dclaw": "hands/dclaw_gripper/dclaw_gripper_glb.urdf",
    "leap": "hands/leap_hand/leap_hand_right_glb.urdf",
    "schunk": "hands/schunk_hand/schunk_svh_hand_right_glb.urdf",
    "shadow": "hands/shadow_hand/shadow_hand_right_glb.urdf",
}

VARIATION_ROBOT_NAME_MAP = {
    "allegro_fsr": "hands/allegro_hand/variation/allegro_hand_right_fsr_glb.urdf",
}


def get_robot_path(filepath: str) -> str:
    asset_path = Path(__file__).parent.parent / "robots"
    urdf_path = asset_path / filepath
    return str(urdf_path.absolute())
