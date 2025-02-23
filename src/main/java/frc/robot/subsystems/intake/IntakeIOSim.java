package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {

  private DCMotor intakeMotor = DCMotor.getNEO(1);

  private FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              intakeMotor,
              IntakeConstants.SimConstants.intakeV,
              IntakeConstants.SimConstants.intakeA),
          intakeMotor);

  public IntakeIOSim() {}

  public void setIntakeVelocity(double velocity) {
    intakeSim.setInputVoltage(velocity);
  }

  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeVelocity = intakeSim.getAngularVelocityRPM();
  }
}
