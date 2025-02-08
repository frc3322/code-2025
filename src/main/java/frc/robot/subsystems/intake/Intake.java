// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO intakeIO;

  private final SensorIO sensorIO;

  public IntakeStates intakeState;

  private static Intake instance;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private SensorIOInputsAutoLogged sensor = new SensorIOInputsAutoLogged();

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

    Logger.processInputs("Intake", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public IntakeStates getState() {
    return intakeState;
  }

  public Command setIntakeStateCommand(IntakeStates state) {
    return new InstantCommand(
        () -> {
          intakeState = state;
        });
  }

  public Command goToStateCommand(Supplier<IntakeStates> stateSupplier) {
    return new RunCommand(
        () -> {
          IntakeStates state = stateSupplier.get();
          intakeIO.setIntakeVelocity(state.intakeVelocity);
          intakeIO.setAdjustVelocity(state.adjustVelocity);
        },
        this);
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
