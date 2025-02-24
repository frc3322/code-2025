import json

def flip_path_file(file_path, center_x=8.052 / 2):
    # Load the .path file (JSON format)
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Flip x-values in waypoints
    for waypoint in data.get("waypoints", []):
        waypoint["anchor"]["x"] = 2 * center_x - waypoint["anchor"]["x"]
        if waypoint["prevControl"]:
            waypoint["prevControl"]["x"] = 2 * center_x - waypoint["prevControl"]["x"]
        if waypoint["nextControl"]:
            waypoint["nextControl"]["x"] = 2 * center_x - waypoint["nextControl"]["x"]

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
file_name = r"C:\Users\graya\Documents\FRC\code-2025\src\main\deploy\pathplanner\paths\R8 return copy.path"  # Replace with the actual file name

flip_path_file(file_name)
