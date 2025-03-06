import json
import os

def flip_path_data(data, center_y=8.052/2):
    # Flip y-values in waypoints
    for waypoint in data.get("waypoints", []):
        waypoint["anchor"]["y"] = 2 * center_y - waypoint["anchor"]["y"]
        if waypoint.get("prevControl"):
            waypoint["prevControl"]["y"] = 2 * center_y - waypoint["prevControl"]["y"]
        if waypoint.get("nextControl"):
            waypoint["nextControl"]["y"] = 2 * center_y - waypoint["nextControl"]["y"]

    # Flip rotation values in states
    if "goalEndState" in data and "rotation" in data["goalEndState"]:
        data["goalEndState"]["rotation"] *= -1

    if "idealStartingState" in data and "rotation" in data["idealStartingState"]:
        data["idealStartingState"]["rotation"] *= -1

    # Update the "folder" key if it exists
    if "folder" in data:
        data["folder"] = "RIGHT SIDE"

    return data

def process_folder(folder_path, center_y=8.052/2):
    # Iterate through all files in the given folder
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        if os.path.isfile(file_path):
            base, ext = os.path.splitext(filename)
            # Only process files with a .path extension that don't have " right" in the name
            if ext.lower() == ".path" and " right" not in base:
                # Load JSON data from the file
                with open(file_path, 'r') as file:
                    data = json.load(file)

                # Process the data
                processed_data = flip_path_data(data, center_y=center_y)

                # Create a new file name with " right" appended before the extension
                new_filename = f"{base} right{ext}"
                new_file_path = os.path.join(folder_path, new_filename)

                # Save the processed data into the new file (overwrite if exists)
                with open(new_file_path, 'w') as new_file:
                    json.dump(processed_data, new_file, indent=2)

                print(f"Processed and saved: {new_file_path}")

# Specify the folder path containing the .path files
folder_path = r"C:\Users\graya\Documents\FRC\code-2025\src\main\deploy\pathplanner\paths"
process_folder(folder_path)
