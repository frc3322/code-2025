package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class IntakeIOSpark implements IntakeIO {

  public SparkMax adjustMotor;
  public SparkFlex intakeMotor;

  public SparkFlexConfig intakeConfig;
  public SparkMaxConfig adjustConfig;

  public LaserCan leftSensor;
  public LaserCan rightSensor;

  public IntakeIOSpark() {
    intakeMotor = new SparkFlex(IntakeConstants.intakeCANId, null);
    adjustMotor = new SparkMax(IntakeConstants.adjustCANId, null);

    intakeConfig.idleMode(IdleMode.kCoast);

    adjustConfig.idleMode(IdleMode.kBrake);

    leftSensor = new LaserCan(IntakeConstants.leftSensorCAN);
    rightSensor = new LaserCan(IntakeConstants.rightSensorCAN);
  }

  public double getIntakeVelocity() {
    return intakeMotor.getEncoder().getVelocity();
  }

  public double getAdjustVelocity() {
    return adjustMotor.getEncoder().getVelocity();
  }

  public boolean leftSensorOutput() {
    LaserCan.Measurement measurement = leftSensor.getMeasurement();
    return measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public boolean rightSensorOutput() {
    LaserCan.Measurement measurement = rightSensor.getMeasurement();
    return measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public void setIntakeVelocity(double velocity) {
    intakeMotor.set(velocity);
  }

  public void setAdjustVelocity(double velocity) {
    adjustMotor.set(velocity);
    intakeMotor.set(0.1);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocity = getIntakeVelocity();
    inputs.adjustVelocity = getAdjustVelocity();
    inputs.leftSensor = leftSensorOutput();
    inputs.rightSensor = rightSensorOutput();
  }
}
