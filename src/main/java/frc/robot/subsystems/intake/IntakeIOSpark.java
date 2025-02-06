package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.CANIDs;

public class IntakeIOSpark implements IntakeIO {

  public SparkMax adjustMotor;
  public SparkFlex intakeMotor;

  public SparkFlexConfig intakeConfig;
  public SparkMaxConfig adjustConfig;

  public IntakeIOSpark() {
    intakeMotor = new SparkFlex(CANIDs.intakeCANId, MotorType.kBrushless);
    adjustMotor = new SparkMax(CANIDs.adjustCANId, MotorType.kBrushless);

    intakeConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(IntakeConstants.intakeMotorCurrentLimit)
        .inverted(false);

    adjustConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(IntakeConstants.adjustMotorCurrentLimit)
        .inverted(false);

    intakeMotor.configure(
        intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    adjustMotor.configure(
        adjustConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  public void setAdjustVelocity(double velocity) {
    adjustMotor.set(velocity);
    intakeMotor.set(0.1);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocity = intakeMotor.getEncoder().getVelocity();
    ;
    inputs.adjustVelocity = adjustMotor.getEncoder().getVelocity();
  }
}
