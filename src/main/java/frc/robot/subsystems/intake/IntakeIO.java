package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double intakeVelocity = 0;
  }

  public default void setIntakeVelocity(double velocity) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
