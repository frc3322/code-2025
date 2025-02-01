// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;

public class Pivot extends SubsystemBase {

  private final PivotIO pivotIO;

  private static Pivot instance;

  private PivotStates pivotState = PivotStates.STOW;

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

  @Override
  public void periodic() {
    switch (pivotState) {
      case STOW:
        break;
      case GROUND:
        break;
      case AGROUND:
        break;
      case L1:
        break;
      case L2:
        break;
      case L3:
        break;
      case L4:
        break;
      case REEFALGAE:
        break;
      case PROCESSER:
        break;
      case BARGE:
        break;
      default:
        break;
    }
  }

  private void setState(PivotStates pivotState) {
    this.pivotState = pivotState;
  }

  public Command setStateCommand(PivotStates pivotState) {
    return new InstantCommand(() -> setState(pivotState), getInstance());
  }
}
