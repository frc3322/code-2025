package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.wrist.WristConstants.GearboxConstants;
import frc.robot.subsystems.wrist.WristConstants.SimConstants;

public class WristIOSim implements WristIO {
  private DCMotor wristMotor = DCMotor.getNeo550(1);

  private DCMotorSim wristMotorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(wristMotor, 0.0005, GearboxConstants.gearRatio),
          wristMotor);

  private ProfiledPIDController wristPID =
      new ProfiledPIDController(
          SimConstants.kP,
          SimConstants.kI,
          SimConstants.kD,
          new Constraints(SimConstants.velocityConstraint, SimConstants.accelerationConstraint));

  private double pidOuput = 0;

  public WristIOSim() {
    wristPID.setTolerance(SimConstants.positionTolerance, SimConstants.velocityTolerance);
  }

  @Override
  public void goToPosition(double targetPositionRotations) {
    wristPID.setGoal(targetPositionRotations);

    pidOuput = wristPID.calculate(wristMotorSim.getAngularPositionRotations());

    wristMotorSim.setInputVoltage(pidOuput * 12);
  }

  public void updateInputs(WristIOInputsAutoLogged inputs) {
    wristMotorSim.update(0.020);

    inputs.absolutePosition = wristMotorSim.getAngularPositionRotations();
    inputs.absoluteVelocity = wristMotorSim.getAngularVelocityRPM();

    inputs.motorPower = pidOuput;

    inputs.setpoint = wristPID.getGoal().position;
    inputs.pidOut = pidOuput;

    inputs.atGoal = wristPID.atGoal();
  }
}
