package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface EagleEyeIO {

  @AutoLog
  public class EagleEyeIOInputs {
    public String[] gamePieceNames = new String[0];
    public String[] gamePieces = new String[0];
    public Pose2d[] globalPositions = new Pose2d[0];
  }

  public default void updateInputs(EagleEyeIOInputsAutoLogged inputs) {}
}
