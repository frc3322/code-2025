// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevatorIO;

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static Elevator instance;

  private final ElevatorVisualizer elevatorVisualizer = new ElevatorVisualizer();

  private ElevatorStates elevatorState = ElevatorStates.STOW;

  private boolean atGoal = false;

  private double elevatorHeight = 0;

  public static Elevator initialize(ElevatorIO elevatorIO) {
    if (instance == null) {
      instance = new Elevator(elevatorIO);
    }
    return instance;
  }

  public static Elevator getInstance() {
    return instance;
  }

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  public void updateInputs() {
    elevatorIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Elevator", inputs);

    elevatorHeight = inputs.position;
  }

  @Override
  public void periodic() {
    updateInputs();

    elevatorVisualizer.update(elevatorHeight);
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public boolean highEnough() {
    return elevatorHeight > 1.5;
  }

  public double getElevatorHeightMeters() {
    return elevatorHeight;
  }

  public ElevatorStates getElevatorState() {
    return elevatorState;
  }

  public void setState(ElevatorStates elevatorState) {
    elevatorIO.presetSetpoint(elevatorHeight);
    this.elevatorState = elevatorState;
  }

  public Command goToStateCommand(Supplier<ElevatorStates> elevatorStateSupplier) {
    return new RunCommand(
        () -> {
          ElevatorStates elevatorSetpoint = elevatorStateSupplier.get();
          elevatorIO.goToPosition(
              elevatorSetpoint.elevatorSetpoint, elevatorSetpoint.elevatorSetpoint);
        },
        this);
  }

  public Command setStateCommand(ElevatorStates elevatorState) {
    return new InstantCommand(
        () -> {
          setState(elevatorState);
          elevatorIO.goToPosition(elevatorState.elevatorSetpoint, elevatorState.elevatorVelocity);
        },
        this);
  }

  public Command lowerElevatorCommand() {
    return new RunCommand(() -> elevatorIO.setMotorSpeeds(-.1), this);
  }

  public Command stopElevatorCommand() {
    return new InstantCommand(() -> elevatorIO.setMotorSpeeds(0), this);
  }

  public Command zeroElevatorCommand() {
    return new InstantCommand(() -> elevatorIO.zeroEncoder(), this);
  }
}
