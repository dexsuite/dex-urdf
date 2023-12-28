import subprocess
from pathlib import Path

from tqdm import tqdm

from robot_info import ORIGINAL_ROBOT_NAME_MAP, get_robot_path

if __name__ == "__main__":
    output_dir = "./figures"
    for robot_name, path in tqdm(ORIGINAL_ROBOT_NAME_MAP.items()):
        png_path = Path(output_dir) / f"{robot_name}-collision.png"
        output_path = str(png_path.absolute())
        intput_path = get_robot_path(path)

        results = subprocess.run(
            [
                "python",
                "tools/generate_urdf_collision_figure_sapien.py",
                intput_path,
                f"--output_image_path={output_path}",
                "--headless",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        print(f"Render collision for {robot_name}")
