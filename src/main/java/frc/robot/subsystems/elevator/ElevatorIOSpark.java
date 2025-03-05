package frc.robot.subsystems.elevator;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.CANIDs;
import frc.robot.subsystems.elevator.ElevatorConstants.ControllerConstants;

public class ElevatorIOSpark implements ElevatorIO {

  private SparkFlex leftMotor;
  private SparkFlex rightMotor;

  RelativeEncoder leftEncoder;

  private ProfiledPIDController elevatorPID;
  private ArmFeedforward elevatorFeedforward;

  private double pidOutput;
  private double ffOutput;

  public ElevatorIOSpark() {
    // Initialize the motors
    leftMotor = new SparkFlex(CANIDs.elevatorLeftCanId, MotorType.kBrushless);
    rightMotor =
        new SparkFlex(CANIDs.elevatorRightCanId, MotorType.kBrushless); // Adjust if necessary

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    // Configure the motors with necessary settings
    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.elevatorMotorCurrentLimit)
        .voltageCompensation(12)
        .inverted(true);
    leftConfig
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);

    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kPersistParameters));

    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.elevatorMotorCurrentLimit)
        .voltageCompensation(12)
        .follow(leftMotor, true);

    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig,
                SparkFlex.ResetMode.kResetSafeParameters,
                SparkFlex.PersistMode.kPersistParameters));

    leftEncoder = leftMotor.getEncoder();

    // Setup the PID controller for the elevator
    elevatorPID =
        new ProfiledPIDController(
            ControllerConstants.kP,
            ControllerConstants.kI,
            ControllerConstants.kD,
            new Constraints(
                ControllerConstants.velocityConstraint,
                ControllerConstants.accelerationConstraint));

    elevatorPID.setTolerance(
        ControllerConstants.positionTolerance, ControllerConstants.velocityTolerance);

    // Setup the feedforward controller for the elevator
    elevatorFeedforward =
        new ArmFeedforward(
            ControllerConstants.kS,
            ControllerConstants.kG,
            ControllerConstants.kV,
            ControllerConstants.kA);
  }

  @Override
  public void goToPosition(double positionRotations, double velocityRotPerSec) {
    // Calculate feedforward and PID output
    ffOutput = elevatorFeedforward.calculate(positionRotations, velocityRotPerSec);

    // Set the goal for the elevator PID
    elevatorPID.setGoal(positionRotations);
    pidOutput = elevatorPID.calculate(leftEncoder.getPosition());

    // Combine feedforward and PID output
    double combinedOutput = ffOutput + pidOutput;

    // Set the motor power to the combined output
    leftMotor.set(combinedOutput);
  }

  @Override
  public void presetSetpoint(double setpointMeters) {
    elevatorPID.setGoal(setpointMeters);
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    // Update the inputs for logging
    inputs.position = leftEncoder.getPosition();
    inputs.velocity = leftEncoder.getVelocity();

    inputs.leftMotorPower = leftMotor.get();
    inputs.rightMotorPower = rightMotor.get();

    inputs.setpoint = elevatorPID.getGoal().position;
    inputs.pidOut = pidOutput;
    inputs.ffOut = ffOutput;

    inputs.atGoal = elevatorPID.atGoal();
  }
}
