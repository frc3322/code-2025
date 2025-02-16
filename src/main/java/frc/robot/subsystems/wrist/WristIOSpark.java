package frc.robot.subsystems.wrist;

import static frc.robot.util.SparkUtil.tryUntilOk;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.CANIDs;

public class WristIOSpark implements WristIO {
  private SparkMax wristMotor;

  private AbsoluteEncoder absoluteEncoder;

  private ProfiledPIDController wristPID;

  private double pidOutput;

  public WristIOSpark() {
    wristMotor = new SparkMax(CANIDs.wristCANId, MotorType.kBrushless);

    SparkMaxConfig wristConfig = new SparkMaxConfig();

    wristConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(WristConstants.wristMotorCurrentLimit)
        .voltageCompensation(12)
        .inverted(false);
    wristConfig.absoluteEncoder.inverted(false).averageDepth(2).zeroCentered(true);
    wristConfig.signals.absoluteEncoderPositionAlwaysOn(true);
    tryUntilOk(
        wristMotor,
        5,
        () ->
            wristMotor.configure(
                wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    absoluteEncoder = wristMotor.getAbsoluteEncoder();

    wristPID =
        new ProfiledPIDController(
            WristConstants.ControllerConstants.kP,
            WristConstants.ControllerConstants.kI,
            WristConstants.ControllerConstants.kD,
            new Constraints(
                WristConstants.ControllerConstants.velocityConstraint,
                WristConstants.ControllerConstants.accelerationConstraint));

    wristPID.setTolerance(
        WristConstants.ControllerConstants.positionTolerance,
        WristConstants.ControllerConstants.velocityTolerance);
  }

  @Override
  public void goToPosition(double position) {
    wristPID.setGoal(position);
    pidOutput = wristPID.calculate(absoluteEncoder.getPosition());

    wristMotor.set(pidOutput);
  }

  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.absolutePosition = absoluteEncoder.getPosition();
    inputs.absoluteVelocity = absoluteEncoder.getVelocity();

    inputs.motorPower = wristMotor.get();

    inputs.setpoint = wristPID.getGoal().position;
    inputs.pidOut = pidOutput;

    inputs.atGoal = wristPID.atGoal();
  }
}
