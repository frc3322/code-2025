package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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

  public static class PlacementPoints {
    // positions starting from left bottom coral position going counter clockwise (in meters)
    private static final Rotation2d emptyRotation = Rotation2d.fromDegrees(0);
    public static final Pose2d coralPosition1 = new Pose2d(3.709, 3.862, emptyRotation);
    public static final Pose2d coralPosition2 = new Pose2d(3.957, 3.433, emptyRotation);
    public static final Pose2d coralPosition3 = new Pose2d(4.241, 3.268, emptyRotation);
    public static final Pose2d coralPosition4 = new Pose2d(4.737, 3.269, emptyRotation);
    public static final Pose2d coralPosition5 = new Pose2d(5.022, 3.432, emptyRotation);
    public static final Pose2d coralPosition6 = new Pose2d(5.269, 3.862, emptyRotation);
    public static final Pose2d coralPosition7 = new Pose2d(5.27, 4.19, emptyRotation);
    public static final Pose2d coralPosition8 = new Pose2d(5.021, 4.619, emptyRotation);
    public static final Pose2d coralPosition9 = new Pose2d(4.737, 4.784, emptyRotation);
    public static final Pose2d coralPosition10 = new Pose2d(4.242, 4.783, emptyRotation);
    public static final Pose2d coralPosition11 = new Pose2d(3.957, 4.62, emptyRotation);
    public static final Pose2d coralPosition12 = new Pose2d(3.71, 4.19, emptyRotation);
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
