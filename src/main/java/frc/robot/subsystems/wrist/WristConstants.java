package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class WristConstants {
  public static final int wristMotorCurrentLimit = 40;
  public static final double minDistanceAutoAdjust = 1;

  public static final double wristMaxRotations = 1;
  public static final double wristMinRotations = -1;
  public static double placementHeight = 0.3048;
  public static double intakeOffset = inchesToMeters(2.566);

  public static double rotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  public static double radiansToRotations(double radians) {
    return radians / (2 * Math.PI);
  }

  public static double inchesToMeters(double inches) {
    return inches * 0.0254;
  }

  public static class GearboxConstants {
    public static final double gearRatio = 0.01;
  }

  public static class ControllerConstants {
    public static LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/WristKP", 4);
    public static LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/WristKI", 0.0);
    public static LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/WristKD", 0.0);

    public static LoggedNetworkNumber velocityConstraint = new LoggedNetworkNumber("/Tuning/WristVelocityConstraint", 8);
    public static LoggedNetworkNumber accelerationConstraint = new LoggedNetworkNumber("/Tuning/WristAccelerationConstraint", 10);
    public static LoggedNetworkNumber positionTolerance = new LoggedNetworkNumber("/Tuning/WristPositionTolerance", 0.0);
    public static LoggedNetworkNumber velocityTolerance = new LoggedNetworkNumber("/Tuning/WristVelocityTolerance", 0.0);
  }

  public static final class SimConstants {
    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = .2;
    public static final double velocityConstraint = 0;
    public static final double accelerationConstraint = 0;
    public static final double positionTolerance = 0;
    public static final double velocityTolerance = 0;

    public static final double simGearRatio = 100;
  }

  public static class Setpoints {
    // ALL SETPOINTS ARE IN ROTATIONS
    public static final double intakePosition = -.25;
    public static final double stowPosition = 0;
    public static final double outakePosition = 0;
  }

  public static enum WristStates {
    INTAKE(Setpoints.intakePosition),
    ALGAEGROUNDINTAKE(-Setpoints.intakePosition),
    STOW(Setpoints.stowPosition),
    OUTAKE(Setpoints.outakePosition);

    public double wristSetpoint;

    private WristStates(double wristSetpoint) {
      this.wristSetpoint = wristSetpoint;
    }
  }
}
