package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.elevator.ElevatorConstants.SimConstants;

public class ElevatorIOSim implements ElevatorIO {
  private DCMotor elevatorMotor =
      DCMotor.getNeoVortex(2); // Using a single NEO motor for the elevator

  private ElevatorSim elevatorSim = new ElevatorSim(elevatorMotor, 0, 0, 0, 0, 0, false, 0);

  private ProfiledPIDController elevatorPID =
      new ProfiledPIDController(
          SimConstants.kP,
          SimConstants.kI,
          SimConstants.kD,
          new Constraints(SimConstants.velocityConstraint, SimConstants.accelerationConstraint));

  private ArmFeedforward elevatorFeedforward =
      new ArmFeedforward(SimConstants.kS, SimConstants.kG, SimConstants.kV, SimConstants.kA);

  private double pidOutput = 0;
  private double ffOutput = 0;

  public ElevatorIOSim() {
    // !!!!!! THE ELEVATOR SIM UNITS ARE IN METERS !!!!!!
    // !!!!!! THE ELEVATOR SIM UNITS ARE IN METERS !!!!!!
    // !!!!!! THE ELEVATOR SIM UNITS ARE IN METERS !!!!!!
    elevatorPID.setTolerance(SimConstants.positionTolerance, SimConstants.velocityTolerance);
  }

  @Override
  public void goToPosition(double targetPositionMeters, double targetVelocityMeters) {
    elevatorPID.setGoal(targetPositionMeters);

    pidOutput = elevatorPID.calculate(elevatorSim.getPositionMeters());
    ffOutput = elevatorFeedforward.calculate(targetPositionMeters, targetVelocityMeters);
    double combinedOutput = pidOutput + ffOutput;

    elevatorSim.setInputVoltage(combinedOutput * 12); // Apply voltage to the elevator motor
  }

  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    elevatorSim.update(0.020); // Update the elevator simulation with a 20ms timestep

    inputs.absolutePosition = elevatorSim.getPositionMeters(); // Position in meters
    inputs.absoluteVelocity =
        elevatorSim.getVelocityMetersPerSecond(); // Velocity in meters per second

    inputs.leftMotorPower = pidOutput + ffOutput; // The power applied to the motor
    inputs.rightMotorPower = pidOutput + ffOutput;

    inputs.setpoint = elevatorPID.getGoal().position; // Target position
    inputs.pidOut = pidOutput; // PID output
    inputs.ffOut = ffOutput; // Feedforward output

    inputs.atGoal = elevatorPID.atGoal(); // Whether the elevator is at its target position
  }
}
