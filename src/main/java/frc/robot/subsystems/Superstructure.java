// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Climber climber;
  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;

  private SuperState superState = SuperState.STOW;

  /** Creates a new Superstructure. */
  public Superstructure(
      Climber climber, Elevator elevator, Intake intake, Pivot pivot, Wrist wrist) {
    this.climber = climber;
    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("SuperStructure/State", getSuperState());
  }

  public SuperState getSuperState() {
    return this.superState;
  }

  public Command setAndGoToRobotStateCommand(SuperState superState) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              this.superState = superState;
            },
            this),
        goToSuperStateCommand(superState));
  }

  public Command goToSuperStateCommand(SuperState superState) {
    return new SequentialCommandGroup(
        climber.setFlipStateCommand(superState.CLIMBER_STATE),
        elevator.setStateCommand(superState.ELEVATOR_STATE),
        intake.setIntakeStateCommand(superState.INTAKE_STATE),
        pivot.setStateCommand(superState.PIVOT_STATE),
        wrist.setStateCommand(superState.WRIST_STATE));
  }
}
