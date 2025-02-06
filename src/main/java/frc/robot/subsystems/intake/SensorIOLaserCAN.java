package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;

public class SensorIOLaserCAN implements SensorIO {

  public LaserCan leftSensor;
  public LaserCan rightSensor;

  public SensorIOLaserCAN(int leftId, int rightId) {
    leftSensor = new LaserCan(leftId);
    rightSensor = new LaserCan(rightId);
  }

  public boolean leftDetected() {
    LaserCan.Measurement measurement = leftSensor.getMeasurement();
    return measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public boolean rightDetected() {
    LaserCan.Measurement measurement = rightSensor.getMeasurement();
    return measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.leftDetected = leftDetected();
    inputs.rightDetected = rightDetected();
  }
}
