package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

  @AutoLog
  public class PivotIOInputs {
    public double leftMotorPower = 0;
    public double rightMotorPower = 0;

    public double absolutePosition = 0;
    public double absoluteVelocity = 0;

    public double setpoint = 0;
    public double pidOut = 0;
    public double ffOut = 0;

    public boolean atGoal = false;
  }

  public default void goToPosition(double positionRotations, double velocityRotPerSec) {}

  public default void updateInputs(PivotIOInputsAutoLogged inputs) {}
}
