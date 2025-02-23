package frc.robot.subsystems.elevator;

public class ElevatorConstants {

  public static final int elevatorMotorCurrentLimit = 60;

  public static final double positionConversionFactor = .0526;
  public static final double velocityConversionFactor = .0526;

  public static class ControllerConstants {
    public static final double kP = 2;
    public static final double kI = 0.1;
    public static final double kD = 0.0;
    public static final double velocityConstraint = 3;
    public static final double accelerationConstraint = 2;
    public static final double positionTolerance = 0.0;
    public static final double velocityTolerance = 0.0;

    public static final double kS = 0.025;
    public static final double kG = 0.035; // Upper 0.06 // Lower 0.01
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public static class ElevatorSetpoints {
    public static final double stowPosition = 0.0;
    public static final double groundPosition = 0.0;
    public static final double aGroundPosition = 0.0;
    public static final double sourcePosition = 0.0;
    public static final double l1Position = 0.0;
    public static final double l2Position = 0.19;
    public static final double l3Position = 0.57;
    public static final double l4Position = 1.4;
    public static final double reefAlgaeLowPosition = 0.0;
    public static final double reefAlgaeHighPosition = 0.0;
    public static final double processerPosition = 0.0;
    public static final double bargePosition = 2.5;

    public static final double stowVelocity = 2;
    public static final double groundVelocity = 0.0;
    public static final double aGroundVelocity = 0.0;
    public static final double sourceVelocity = 0.0;
    public static final double l1Velocity = 0.0;
    public static final double l2Velocity = 0.0;
    public static final double l3Velocity = 0.0;
    public static final double l4Velocity = 0.0;
    public static final double reefAlgaeLowVelocity = 0.0;
    public static final double reefAlgaeHighVelocity = 0.0;
    public static final double processerVelocity = 0.0;
    public static final double bargeVelocity = 2;
  }

  public static final class SimConstants {
    public static final double gearRatio = 5; // Gear ratio
    public static final double elevatorMassKg = 18.1437 * 2; // Mass of elevator
    public static final double drumRadiusMeters = 0.01905; // .75 in
    public static final double startingHeightMeters = 0.0; // Height of elevator
    public static final double minHeightMeters = 0.0;
    public static final double maxHeightMeters = 0.8636; // guess, 95 in

    // PID and feedforward constants
    public static final double kP = 10;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double velocityConstraint = 0.0;
    public static final double accelerationConstraint = 0.0;
    public static final double positionTolerance = 0.02;
    public static final double velocityTolerance = 0.02;

    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public static enum ElevatorStates {
    STOW(ElevatorSetpoints.stowPosition, ElevatorSetpoints.stowVelocity),
    GROUND(ElevatorSetpoints.groundPosition, ElevatorSetpoints.groundVelocity),
    AGROUND(ElevatorSetpoints.aGroundPosition, ElevatorSetpoints.aGroundVelocity),
    SOURCE(ElevatorSetpoints.sourcePosition, ElevatorSetpoints.sourceVelocity),
    L1(ElevatorSetpoints.l1Position, ElevatorSetpoints.l1Velocity),
    L2(ElevatorSetpoints.l2Position, ElevatorSetpoints.l2Velocity),
    L3(ElevatorSetpoints.l3Position, ElevatorSetpoints.l3Velocity),
    L4(ElevatorSetpoints.l4Position, ElevatorSetpoints.l4Velocity),
    REEFALGAELOW(ElevatorSetpoints.reefAlgaeLowPosition, ElevatorSetpoints.reefAlgaeLowVelocity),
    REEFALGAEHIGH(ElevatorSetpoints.reefAlgaeHighPosition, ElevatorSetpoints.reefAlgaeHighVelocity),
    PROCESSER(ElevatorSetpoints.processerPosition, ElevatorSetpoints.processerVelocity),
    BARGE(ElevatorSetpoints.bargePosition, ElevatorSetpoints.bargeVelocity);

    public double elevatorSetpoint;
    public double elevatorVelocity;

    private ElevatorStates(double elevatorSetpoint, double elevatorVelocity) {
      this.elevatorSetpoint = elevatorSetpoint;
      this.elevatorVelocity = elevatorVelocity;
    }
  }
}
