package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double climberPosition;
    public double climberSetpoint;
    public double winchPower;
  }

  public void setClimberVelocity(double power);

  public void setClimberSetpoint(double setpoint);

  public void setWinchVelocity(double power);

  public void updateInputs(ClimberIOInputs inputs);
}
