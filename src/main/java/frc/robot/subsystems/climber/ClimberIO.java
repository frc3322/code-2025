package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public class ClimberIOInputs {
    public double position = 0; // Current position in meters
    public double velocity = 0; // Current velocity in meters per second

    public double setpoint = 0; // Target position in meters
    public double pidOut = 0; // PID output
    public double ffOut = 0; // Feedforward output
  }

  public default void runWinch(double velocity) {}

  public default void goToPosition(double setPoint) {}

  public default void setSetPoint(double setPoint) {}

  public default void updateInputs(ClimberIOInputsAutoLogged inputs) {}
}
