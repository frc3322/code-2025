package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.Logger;

public class EagleEyeVisualizer {
  public EagleEyeVisualizer() {}

  public Pose3d[] fieldGamePieces =
      new Pose3d[] {new Pose3d(2, 2, 0.0762, new Rotation3d(0, 0, 0))};

  public void update(Pose2d[] globalPositions) {
    Logger.recordOutput("eagleEyeVisualizer/gamePieces", globalPositions);
    Logger.recordOutput("eagleEyeVisualizer/gamePiecesOnField", fieldGamePieces);
  }
}
