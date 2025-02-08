package frc.robot.subsystems.wrist;

public class WristConstants {
  public static final int wristMotorCurrentLimit = 40;

  public static class GearboxConstants {
    public static final double gearRatio = .01;
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

  public static class Setpoints {
    // ALL SETPOINTS ARE IN ROTATIONS
    public static final double intakePosition = 0;
    public static final double stowPosition = 0;

    public static final double algeaStowPosition = 0.25;

    public static final double outakeForwardPosition = 0.25;
    public static final double outakeBackwardPosition = -0.25;
  }

  public static enum WristStates {
    INTAKE(Setpoints.intakePosition),
    STOW(Setpoints.stowPosition),
    ALGEA_STOW(Setpoints.algeaStowPosition),
    OUTAKE_FORWARD(Setpoints.outakeForwardPosition),
    OUTAKE_BACKWARD(Setpoints.outakeBackwardPosition);

    public double wristSetpoint;

    private WristStates(double wristSetpoint) {
      this.wristSetpoint = wristSetpoint;
    }
  }
}
