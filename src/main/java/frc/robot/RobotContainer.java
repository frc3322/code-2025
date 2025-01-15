// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.Drive.SwerveIO;
import frc.robot.subsystems.Drive.SwerveIOSpark;

public class RobotContainer {
  
  private Drive drivetrain;
  
  public RobotContainer() {
    
    switch(Constants.currentMode){
      case REAL:
        drivetrain = Drive.initialize(new SwerveIOSpark());
        break;
      case SIM:
        //drivetrain = new Drive();
        break;
      default:
        drivetrain = new Drive(new SwerveIO() {});
        break;
    }

    drivetrain.setDefaultCommand(drivetrain.humanDriveCommand(
      OIConstants.driverController::getLeftX,
      OIConstants.driverController::getLeftY,
      OIConstants.driverController::getRightX
    ));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
