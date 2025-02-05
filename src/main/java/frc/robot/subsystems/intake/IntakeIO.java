package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double intakeVelocity = 0;
    public double adjustVelocity = 0;
    public boolean inPosition = false;
  }

  public double getIntakeVelocity();

  public double getAdjustVelocity();

  public void setIntakeVelocity(double velocity);

  public void setAdjustVelocity(double velocity);

  public void updateInputs(IntakeIOInputs inputs);
}
