// package frc.robot.subsystems.intake;

// import au.grapplerobotics.LaserCan;
// import frc.robot.Constants.CANIDs;

// public class SensorIOLaserCAN implements SensorIO {

//   public LaserCan leftSensor;
//   public LaserCan rightSensor;

//   public SensorIOLaserCAN() {
//     leftSensor = new LaserCan(CANIDs.leftSensorCAN);
//     rightSensor = new LaserCan(CANIDs.rightSensorCAN);
//   }

//   public boolean leftDetected() {
//     LaserCan.Measurement measurement = leftSensor.getMeasurement();
//     return measurement == null || measurement.status !=
// LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
//   }

//   public boolean rightDetected() {
//     LaserCan.Measurement measurement = rightSensor.getMeasurement();
//     return measurement == null || measurement.status !=
// LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
//   }

//   @Override
//   public void updateInputs(SensorIOInputs inputs) {
//     inputs.leftDetected = leftDetected();
//     inputs.rightDetected = rightDetected();
//   }
// }
