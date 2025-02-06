package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {

  private DCMotor intakeMotor = DCMotor.getNeo550(2);
  private DCMotor adjustMotor = DCMotor.getNeo550(2);

  private FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              intakeMotor,
              IntakeConstants.SimConstants.intakeV,
              IntakeConstants.SimConstants.intakeA),
          intakeMotor);

  private FlywheelSim adjustSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              adjustMotor,
              IntakeConstants.SimConstants.adjustV,
              IntakeConstants.SimConstants.adjustA),
          intakeMotor);

  public IntakeIOSim() {}

  public void setIntakeVelocity(double velocity) {
    intakeSim.setInputVoltage(velocity);
    adjustSim.setInputVoltage(0.1);
  }

  public void setAdjustVelocity(double velocity) {
    adjustSim.setInputVoltage(velocity);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.adjustVelocity = adjustSim.getAngularVelocityRPM();
    inputs.intakeVelocity = intakeSim.getAngularVelocityRPM();
  }
}
