

import os
from pathlib import Path

# Use a path that works on the Raspberry Pi
# On the Pi, this will create the file in /home/pi/roboproject/
path_to_folder = Path("/home/pi/roboproject")

# Create the directory if it doesn't exist yet
path_to_folder.mkdir(parents=True, exist_ok=True)

# Define the empty file
file_path = path_to_folder / "boot_signal.txt"

# Create the empty file
with open(file_path, "w") as f:
    pass  # 'pass' just means 'do nothing', effectively creating an empty file

print(f"Empty file created at: {file_path}")