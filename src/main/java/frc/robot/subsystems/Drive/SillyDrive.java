// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SillyDrive extends SubsystemBase {
      // https://docs.yagsl.com/configuring-yagsl/code-setup
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;

    public SillyDrive() {
        /* 
        * !_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        * TELEMETRY - SET THIS TO LOW FOR DEBUGGING, AND HIGH FOR FASTER CODE
        * !_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        */
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    
        // Swerve init - https://docs.yagsl.com/configuring-yagsl/code-setup 
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maximumModuleSpeed);
        }
        catch (IOException e) {
            DriverStation.reportError("Swerve config not found: " + e.toString(), true);
        }

        //https://github.com/BroncBotz3481/YAGSL-Example/blob/main/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java
        //Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setHeadingCorrection(false);
        //!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setCosineCompensator(false);
        //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); 
        // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
      }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void drive(double translationX, double translationY, double rotation, boolean isFieldRelative, boolean isOpenLoop){
        swerveDrive.drive(new Translation2d(translationX * swerveDrive.getMaximumModuleDriveVelocity(),
            translationY * swerveDrive.getMaximumModuleDriveVelocity()),
            rotation * swerveDrive.getMaximumModuleDriveVelocity(),
            isFieldRelative,
            isOpenLoop // Ususally false
        );
    }	

    public Command humanDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
      return new RunCommand(
        () -> {
          this.drive(
            MathUtil.applyDeadband(xInput.getAsDouble(),0.05), 
            MathUtil.applyDeadband(yInput.getAsDouble(),0.05), 
            MathUtil.applyDeadband(thetaInput.getAsDouble() * thetaInput.getAsDouble(),0.05), 
            false, 
            false
          );
        }
        , this);
  }
}
