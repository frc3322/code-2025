import json

def flip_path_file(file_path, center_y=8.052 / 2):
    # Load the .path file (JSON format)
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Flip y-values in waypoints
    for waypoint in data.get("waypoints", []):
        waypoint["anchor"]["y"] = 2 * center_y - waypoint["anchor"]["y"]
        if waypoint["prevControl"]:
            waypoint["prevControl"]["y"] = 2 * center_y - waypoint["prevControl"]["y"]
        if waypoint["nextControl"]:
            waypoint["nextControl"]["y"] = 2 * center_y - waypoint["nextControl"]["y"]

    # Flip rotation values
    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] *= -1

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] *= -1

    # Overwrite the original file
    with open(file_path, 'w') as file:
        json.dump(data, file, indent=2)

    print(f"Processed and saved: {file_path}")

# Set the file path
file_name = r"D:\Vs Code Projects\code-2025\src\main\deploy\pathplanner\paths\to R9 copy.path"  # Replace with the actual file name

flip_path_file(file_name)
