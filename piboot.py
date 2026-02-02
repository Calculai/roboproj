
import os
from pathlib import Path

# Absolute path is key for boot scripts
path_to_folder = Path("/home/pi/roboproject")
path_to_folder.mkdir(parents=True, exist_ok=True)

file_path = path_to_folder / "boot_signal.txt"

with open(file_path, "w") as f:
    f.write("Boot successful!")