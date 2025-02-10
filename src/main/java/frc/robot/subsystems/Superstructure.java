// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure extends SubsystemBase {

  private final Climber climber;
  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;

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
  }
}
