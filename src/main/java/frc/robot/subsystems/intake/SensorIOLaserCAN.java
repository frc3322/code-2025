package frc.robot.subsystems.intake;

import au.grapplerobotics.LaserCan;

public class SensorIOLaserCAN implements SensorIO {

  public LaserCan sensor;

  public SensorIOLaserCAN(int id) {
    sensor = new LaserCan(id);
  }

  public boolean detected() {
    LaserCan.Measurement measurement = sensor.getMeasurement();
    return measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.detected = detected();
  }
}
