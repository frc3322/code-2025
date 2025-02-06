package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  @AutoLog
  public class SensorIOInputs {
    public boolean leftDetected = false;
    public boolean rightDetected = false;
  }

  public void updateInputs(SensorIOInputs inputs);
}
