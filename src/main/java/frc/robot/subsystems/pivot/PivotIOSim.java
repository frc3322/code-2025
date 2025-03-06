package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.pivot.PivotConstants.SimConstants;

public class PivotIOSim implements PivotIO {
  private DCMotor pivotMotors = DCMotor.getNEO(2);
  private SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          pivotMotors,
          SimConstants.gearRatio,
          SimConstants.jKgMetersSquared,
          SimConstants.armLengthMeters,
          SimConstants.minRotationRadians,
          SimConstants.maxRotationRadians,
          true,
          SimConstants.startingAngleRads);

  private ProfiledPIDController pivotPID =
      new ProfiledPIDController(
          SimConstants.kP,
          SimConstants.kI,
          SimConstants.kD,
          new Constraints(SimConstants.velocityConstraint, SimConstants.accelerationConstraint));

  private ArmFeedforward pivotFeedforward =
      new ArmFeedforward(SimConstants.kS, SimConstants.kG, SimConstants.kV, SimConstants.kA);

  private double pidOuput = 0;
  private double ffOutput = 0;

  public PivotIOSim() {
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    // !!!!!! THE PIVOT SIM UNITS ARE IN RADIANS !!!!!!
    pivotPID.setTolerance(SimConstants.positionTolerance, SimConstants.velocityTolerance);
  }

  @Override
  public void goToPosition(double targetPositionRotations, double targetVelocityRotations) {
    double targetPositionRadians = PivotConstants.rotationsToRadians(targetPositionRotations);
    double targetVelocityRadians = PivotConstants.rotationsToRadians(targetVelocityRotations);

    pivotPID.setGoal(targetPositionRadians);

    pidOuput = pivotPID.calculate(pivotSim.getAngleRads());
    ffOutput = pivotFeedforward.calculate(targetPositionRadians, targetVelocityRadians);
    double combinedOut = pidOuput + ffOutput;

    pivotSim.setInputVoltage(combinedOut * 12);
  }

  @Override
  public void presetSetpoint(double setpointRotations) {
    pivotPID.setGoal(setpointRotations);
  }

  public void updateInputs(PivotIOInputsAutoLogged inputs) {
    pivotSim.update(0.020);

    inputs.absolutePosition = PivotConstants.radiansToRotations(pivotSim.getAngleRads());
    inputs.absoluteVelocity = PivotConstants.radiansToRotations(pivotSim.getVelocityRadPerSec());

    inputs.leftMotorPower = pidOuput + ffOutput;
    inputs.rightMotorPower = pidOuput + ffOutput;

    inputs.setpoint = PivotConstants.radiansToRotations(pivotPID.getGoal().position);
    inputs.pidOut = pidOuput;
    inputs.ffOut = ffOutput;

    inputs.atGoal = pivotPID.atGoal();
  }
}
