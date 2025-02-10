// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

public class WaitUntilConditionCommand extends Command {
  /** Creates a new WaitUntilCondition. */
  BooleanSupplier condition;

  boolean conditionMet = false;

  public WaitUntilConditionCommand(BooleanSupplier condition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.condition = condition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conditionMet = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conditionMet = condition.getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return conditionMet;
  }
}
