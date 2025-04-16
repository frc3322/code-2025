package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

  @AutoLog
  public class ElevatorIOInputs {
    public double leftMotorPower = 0;
    public double rightMotorPower = 0; // Power applied to the motor

    public double position = 0; // Current position in meters
    public double velocity = 0; // Current velocity in meters per second

    public double setpoint = 0; // Target position in meters
    public double pidOut = 0; // PID output
    public double ffOut = 0; // Feedforward output

    public boolean atGoal = false; // Whether the elevator is at the target position
  }

  /**
   * Method to move the elevator to a specific position with a given velocity.
   *
   * @param positionMeters Target position in meters.
   * @param velocityMetersPerSec Target velocity in meters per second.
   */
  public default void goToPosition(double positionMeters, double velocityMetersPerSec) {}

  public default void presetSetpoint(double setpointMeters) {}

  public default void zeroEncoder() {}

  public default void setMotorSpeeds(double speeds) {}

  /**
   * Method to update the elevator's input values for simulation or logging.
   *
   * @param inputs The ElevatorIOInputsAutoLogged object to store the current state.
   */
  public default void updateInputs(ElevatorIOInputsAutoLogged inputs) {}
}
