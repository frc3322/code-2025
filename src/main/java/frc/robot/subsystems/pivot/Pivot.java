// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private final PivotIO pivotIO;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private static Pivot instance;

  private PivotStates pivotState = PivotStates.STOW;

  private boolean atGoal = false;

  public static Pivot initialize(PivotIO pivotIO) {
    if (instance == null) {
      instance = new Pivot(pivotIO);
    }
    return instance;
  }

  public static Pivot getInstance() {
    return instance;
  }

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivotIO) {
    this.pivotIO = pivotIO;
  }

  public void updateInputs() {
    pivotIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Pivot", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public PivotStates getPivotState() {
    return pivotState;
  }

  private void setState(PivotStates pivotState) {
    this.pivotState = pivotState;
  }

  public Command goToStateCommand(Supplier<PivotStates> pivotStateSupplier) {
    return new RunCommand(
        () -> {
          PivotStates pivotSetpoint = pivotStateSupplier.get();
          pivotIO.goToPosition(pivotSetpoint.armSetpoint, pivotSetpoint.armVelocity);
        },
        this);
  }

  public Command setStateCommand(PivotStates pivotState) {
    return new InstantCommand(
        () -> {
          setState(pivotState);
          pivotIO.goToPosition(pivotState.armSetpoint, pivotState.armVelocity);
        },
        this);
  }
}
