package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.Logger;

public class EagleEyeVisualizer {
  public EagleEyeVisualizer() {}

  public void update(Pose2d[] globalPositions) {
    Logger.recordOutput("eagleEyeVisualizer/gamePieces", globalPositions);
  }
}
