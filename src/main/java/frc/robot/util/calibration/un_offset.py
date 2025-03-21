import math

class Pose2d:
    """
    A simple 2D pose class with x, y coordinates and a heading angle in radians.
    """
    def __init__(self, x: float, y: float, theta: float):
        self.x = x
        self.y = y
        self.theta = theta  # Heading angle in radians

    def __repr__(self):
        # Display theta in degrees for readability.
        return f"Pose2d(x={self.x:.3f}, y={self.y:.3f}, theta={math.degrees(self.theta):.1f}°)"


def undo_offsets(modified_pose: Pose2d, applied_rotation_deg: float, local_offset: float, y_adjust: float = -0.125) -> Pose2d:
    """
    Reverses the offsets applied in the target pose adjustments.
    
    The original transformation sequence was:
      1. Apply a local offset: 
           P1 = local_offset_pose2d(P0, local_offset)
      2. Rotate by ±90°:
           P2 = rotate(P1, applied_rotation_deg)  # only heading changes
      3. Add a y-adjust along the new heading:
           P3 = P2 translated by y_adjust along its heading
    
    This function reverses those steps:
      a. Undo the y-adjust: subtract the y adjustment along the final heading.
      b. Undo the rotation: subtract the applied rotation from the heading.
      c. Undo the local offset: subtract the local offset translation along the recovered heading.
      
    Parameters:
      modified_pose: The final Pose2d after all adjustments (P3).
      applied_rotation_deg: The rotation (in degrees) that was applied (e.g. +90 or -90).
      local_offset: The offset distance that was applied in the original local offset step.
      y_adjust: The y-adjust distance that was applied (default is -0.125).
    
    Returns:
      The original Pose2d before the offsets were applied.
    """
    # Step 1: Undo the y-adjust.
    # The original transformation added: 
    #    x_new = x_old + y_adjust * cos(theta)
    #    y_new = y_old + y_adjust * sin(theta)
    # So we subtract that translation using the final heading.
    x_without_y_adjust = modified_pose.x - y_adjust * math.cos(modified_pose.theta)
    y_without_y_adjust = modified_pose.y - y_adjust * math.sin(modified_pose.theta)
    pose_after_y = Pose2d(x_without_y_adjust, y_without_y_adjust, modified_pose.theta)
    
    # Step 2: Undo the ±90° rotation.
    # Since the rotation was applied around the pose’s own translation (leaving position unchanged),
    # we simply subtract the applied rotation angle from the heading.
    theta_before_rotation = pose_after_y.theta - math.radians(applied_rotation_deg)
    pose_before_rotation = Pose2d(pose_after_y.x, pose_after_y.y, theta_before_rotation)
    
    # Step 3: Undo the local offset.
    # If the local offset was applied as:
    #    x_offset = local_offset * cos(original_heading)
    #    y_offset = local_offset * sin(original_heading)
    # then we subtract that same translation.
    original_x = pose_before_rotation.x - local_offset * math.cos(pose_before_rotation.theta)
    original_y = pose_before_rotation.y - local_offset * math.sin(pose_before_rotation.theta)
    original_pose = Pose2d(original_x, original_y, pose_before_rotation.theta)
    
    return original_pose


# Example usage:
if __name__ == "__main__":
    # Suppose this is the final modified pose after all transformations:
    modified = Pose2d(5.987, 3.737, 1.571)  # Example values
    # Suppose the original transformation applied a rotation of +90 degrees and a local offset of 0.5 units.
    applied_rotation = -90    # could also be -90 depending on the condition in your original code
    
    robotWidth = 40 * 0.0254
    offsetDistanceL4 = (-robotWidth / 2) - .21
    offsetDistanceL1To3 = (-robotWidth / 2) - .05

    # Undo the offsets:
    original = undo_offsets(modified, applied_rotation, offsetDistanceL4)
    
    print("Modified pose:", modified)
    print("Recovered original pose:", original)
