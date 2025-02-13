package frc.robot.subsystems.wrist;

public class WristConstants {
  public static final int wristMotorCurrentLimit = 40;
  public static final double minDistanceAutoAdjust = 1;

  public static final double wristMaxRotations = 1;
  public static final double wristMinRotations = -1;

  public static double rotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  public static double radiansToRotations(double radians) {
    return radians / (2 * Math.PI);
  }

  public static class GearboxConstants {
    public static final double gearRatio = 1;
  }

  public static class ControllerConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double velocityConstraint = 1;
    public static final double accelerationConstraint = 2;
    public static final double positionTolerance = 0;
    public static final double velocityTolerance = 0;
  }

  public static final class SimConstants {
    public static final double kP = .01;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double velocityConstraint = 0;
    public static final double accelerationConstraint = 0;
    public static final double positionTolerance = 0;
    public static final double velocityTolerance = 0;
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
