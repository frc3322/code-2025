// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO intakeIO;

  private static Intake instance;
  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double intakeState;
  private double adjustState;

  public Intake(IntakeIO intake) {
    intakeIO = intake;
  }

  public static Intake getInstance() {
    return instance;
  }

  public static Intake initialize(IntakeIO intake) {
    if (instance == null) {
      instance = new Intake(intake);
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

  public Command adjustToMiddleCommand(){
    return new InstantCommand(
      () ->{
        if(coralNotInPosition()){
         double direction = inputs.leftSensor ? -1 : 1;
         intakeIO.setAdjustVelocity(direction);
        }
      }
    );
  }

  public boolean coralNotInPosition() {
    return inputs.leftSensor ^ inputs.rightSensor;
  }
}
