package frc.robot.subsystems.intake;

public class SensorIOSim implements SensorIO {

  public SensorIOSim() {}

  public boolean leftDetected() {
    return false;
  }

  public boolean rightDetected() {
    return false;
  }

  @Override
  public void updateInputs(SensorIOInputs inputs) {
    inputs.leftDetected = leftDetected();
    inputs.rightDetected = rightDetected();
  }
}
