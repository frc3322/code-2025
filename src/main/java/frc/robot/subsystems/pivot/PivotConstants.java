package frc.robot.subsystems.pivot;

public class PivotConstants {

  public static final int pivotMotorCurrentLimit = 60;

  public static final double sourceDetectionRadiusMeters = 3.5;

  public static final double leftSourceTargetAngleRadians = 0.523598776;
  public static final double rightSourceTargetAngleRadians = -0.523598776;

  public static double rotationsToRadians(double rotations) {
    return rotations * 2 * Math.PI;
  }

  public static double radiansToRotations(double radians) {
    return radians / (2 * Math.PI);
  }

  public static class ControllerConstants {
    public static final double kP = 2; // 2
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double velocityConstraint = 1; // 4
    public static final double accelerationConstraint = 3.2; // 3.2
    public static final double positionTolerance = 0.05;
    public static final double velocityTolerance = 0.05;

    public static final double kS = .035;
    public static final double kG = .035; // Upper:0.07 Lower:0
    public static final double kV = 0;
    public static final double kA = 0;
  }

  public static class PivotSetpoints {
    // ALL SETPOINTS ARE IN ROTATIONS
    public static final double stowPosition = 0;
    public static final double groundPosition = 0.27;
    public static final double aGroundPosition = 0;
    public static final double sourcePosition = 0.07;
    public static final double l1Position = -.1;
    public static final double l2Position = -.1;
    public static final double l3Position = -.1;
    public static final double l4Position = -.15;
    public static final double l4AutoPosition = -0.02;
    public static final double l2and3ScorePosition = -.2;
    public static final double l4ScorePosition = -0.25;
    public static final double reefAlgaePosition = 0;
    public static final double processerPosition = 0;
    public static final double bargePosition = 0;

    // ALL VELOCITY SETPOINTS ARE IN ROT/SEC
    public static final double stowVelocity = 0;
    public static final double groundVelocity = .5;
    public static final double aGroundVelocity = 0;
    public static final double sourceVelocity = 0;
    public static final double l1Velocity = 0;
    public static final double l2Velocity = 0;
    public static final double l3Velocity = 0;
    public static final double l4Velocity = 0;
    public static final double l4AutoVelocity = -0.05;
    public static final double l2and3ScoreVelocity = 0;
    public static final double l4ScoreVelocity = 0;
    public static final double reefAlgaeVelocity = 0;
    public static final double processerVelocity = 0;
    public static final double bargeVelocity = 0;
  }

  public static final class SimConstants {
    public static final double gearRatio = 48; // 4:1 -> 3:1 -> 4:1
    public static final double jKgMetersSquared = 1.161288; // Rough guess
    public static final double armLengthMeters = 0.6223;
    public static final double minRotationRadians = -Math.PI;
    public static final double maxRotationRadians = Math.PI;

    public static final double startingAngleRads = Math.PI;

    public static final double kP = 3;
    public static final double kI = 0;
    public static final double kD = 0.25;
    public static final double velocityConstraint = 0;
    public static final double accelerationConstraint = 0;
    public static final double positionTolerance = .1;
    public static final double velocityTolerance = .1;

    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
  }

  public static enum PivotStates {
    STOW(
        PivotSetpoints.stowPosition,
        PivotSetpoints.stowVelocity,
        StateType.NONE), // Arm straight up
    GROUND(
        PivotSetpoints.groundPosition,
        PivotSetpoints.groundVelocity,
        StateType.INTAKING), // Coral ground intake
    AGROUND(
        PivotSetpoints.aGroundPosition,
        PivotSetpoints.aGroundVelocity,
        StateType.INTAKING), // Algae ground intake
    SOURCE(PivotSetpoints.sourcePosition, PivotSetpoints.sourceVelocity, StateType.INTAKING),
    L1(
        PivotSetpoints.l1Position,
        PivotSetpoints.l1Velocity,
        StateType.REEFSCORING), // L1 coral scoring
    L2(
        PivotSetpoints.l2Position,
        PivotSetpoints.l2Velocity,
        StateType.REEFSCORING), // L2 coral scoring
    L3(
        PivotSetpoints.l3Position,
        PivotSetpoints.l3Velocity,
        StateType.REEFSCORING), // L3 coral scoring
    L4(
        PivotSetpoints.l4Position,
        PivotSetpoints.l4Velocity,
        StateType.REEFSCORING), // L4 coral scoring
    L4AUTO(PivotSetpoints.l4AutoPosition, PivotSetpoints.l4AutoVelocity, StateType.REEFSCORING),
    L2AND3SCORE(
        PivotSetpoints.l2and3ScorePosition,
        PivotSetpoints.l2and3ScoreVelocity,
        StateType.REEFSCORING),
    L4SCORE(PivotSetpoints.l4ScorePosition, PivotSetpoints.l4ScoreVelocity, StateType.REEFSCORING),
    REEFALGAE(
        PivotSetpoints.reefAlgaePosition,
        PivotSetpoints.reefAlgaeVelocity,
        StateType.REEFSCORING), // Algae from reef
    PROCESSER(
        PivotSetpoints.processerPosition,
        PivotSetpoints.processerVelocity,
        StateType.NONE), // Scoring Algae in processer
    BARGE(
        PivotSetpoints.bargeVelocity,
        PivotSetpoints.bargePosition,
        StateType.BARGESCORING); // Scoring Algae in barge

    public double armSetpoint;
    public double armVelocity;
    public StateType stateType;

    private PivotStates(double armSetpoint, double armVelocity, StateType stateType) {
      this.armSetpoint = armSetpoint;
      this.armVelocity = armVelocity;
      this.stateType = stateType;
    }
  }

  public static enum StateType {
    NONE,
    INTAKING,
    REEFSCORING,
    BARGESCORING
  }
}
