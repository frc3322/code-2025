package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface SensorIO {
  @AutoLog
  public class SensorIOInputs {
    boolean detected = false;
  }

  public boolean detected();

  public void updateInputs(SensorIOInputs inputs);
}
