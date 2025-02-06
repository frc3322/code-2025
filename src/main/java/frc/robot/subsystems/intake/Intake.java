// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO intakeIO;

  private final SensorIO sensorIO;

  private static Intake instance;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private SensorIOInputsAutoLogged sensor = new SensorIOInputsAutoLogged();
  private double intakeState;
  private double adjustState;

  public Intake(IntakeIO intake, SensorIO sensor) {
    intakeIO = intake;
    sensorIO = sensor;
  }

  public static Intake getInstance() {
    return instance;
  }

  public static Intake initialize(IntakeIO intake, SensorIO sensor) {
    if (instance == null) {
      instance = new Intake(intake, sensor);
    }
    return instance;
  }

  public void updateInputs() {
    intakeIO.updateInputs(inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
    intakeIO.setIntakeVelocity(intakeState);
    intakeIO.setAdjustVelocity(adjustState);

    
    Logger.processInputs("Intake", inputs);
  }

  public Command setIntakeStateCommand(double state) {
    return new InstantCommand(
        () -> {
          intakeState = state;
        });
  }

  public Command setAdjustStateCommand(double state) {
    return new InstantCommand(
        () -> {
          adjustState = state;
        });
  }

  public Command adjustToMiddleCommand() {
    return new InstantCommand(
        () -> {
          if (coralNotInPosition()) {
            double direction = sensor.leftDetected ? -1 : 1;
            intakeIO.setAdjustVelocity(direction);
          }
        });
  }

  public boolean coralNotInPosition() {
    return sensor.leftDetected ^ sensor.rightDetected;
  }
}
