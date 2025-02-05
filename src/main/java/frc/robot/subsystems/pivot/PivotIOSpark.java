package frc.robot.subsystems.pivot;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.CANIDs;
import frc.robot.subsystems.pivot.PivotConstants.ControllerConstants;

public class PivotIOSpark implements PivotIO {

  private SparkMax leftMotor;
  private SparkMax rightMotor;

  private AbsoluteEncoder absoluteEncoder;

  private ProfiledPIDController pivotPID;
  private ArmFeedforward pivotFeedforward;

  private double pidOutput;
  private double ffOutput;

  public PivotIOSpark() {
    leftMotor = new SparkMax(CANIDs.pivotLeftCanId, MotorType.kBrushless);
    rightMotor = new SparkMax(CANIDs.pivotLeftCanId, MotorType.kBrushless);

    SparkMaxConfig leftConfig = new SparkMaxConfig();
    SparkMaxConfig rightConfig = new SparkMaxConfig();

    leftConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(PivotConstants.pivotMotorCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);
    leftConfig.absoluteEncoder.inverted(true).averageDepth(2);
    leftConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    tryUntilOk(
        leftMotor,
        5,
        () ->
            leftMotor.configure(
                leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    rightConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(PivotConstants.pivotMotorCurrentLimit)
        .voltageCompensation(12)
        .inverted(false)
        .follow(leftMotor);
    tryUntilOk(
        rightMotor,
        5,
        () ->
            rightMotor.configure(
                rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    absoluteEncoder = leftMotor.getAbsoluteEncoder();

    pivotPID =
        new ProfiledPIDController(
            ControllerConstants.kP,
            ControllerConstants.kI,
            ControllerConstants.kD,
            new Constraints(
                ControllerConstants.velocityConstraint,
                ControllerConstants.accelerationConstraint));

    pivotPID.setTolerance(
        ControllerConstants.positionTolerance, ControllerConstants.velocityTolerance);

    pivotFeedforward =
        new ArmFeedforward(
            ControllerConstants.kS,
            ControllerConstants.kG,
            ControllerConstants.kV,
            ControllerConstants.kA);
  }

  @Override
  public void goToPosition(double positionRotations, double velocityRotPerSec) {
    ffOutput = pivotFeedforward.calculate(positionRotations, velocityRotPerSec);

    pivotPID.setGoal(positionRotations);
    pidOutput = pivotPID.calculate(absoluteEncoder.getPosition());

    double combinedOutput = ffOutput + pidOutput;

    leftMotor.set(combinedOutput);
  }

  @Override
  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    inputs.absolutePosition = absoluteEncoder.getPosition();
    inputs.absoluteVelocity = absoluteEncoder.getVelocity();

    inputs.leftMotorPower = leftMotor.get();
    inputs.rightMotorPower = rightMotor.get();

    inputs.setpoint = pivotPID.getGoal().position;
    inputs.pidOut = pidOutput;
    inputs.ffOut = ffOutput;

    inputs.atGoal = pivotPID.atGoal();
  }
}
