// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.Drive.SwerveIO.SwerveIOInputs;


/*
  Edit configs in deploy/swerve.

  https://docs.yagsl.com/configuring-yagsl/configuration

  A LOT OF CODE BORROWED FROM https://github.com/WindingMotor/SwerveDriveAdvantage2024/blob/YAGSL/src/main/java/frc/robot/subsystems/swerve/SUB_Swerve.java
*/
public class Drive extends SubsystemBase {

  private SwerveIO swerveIO;

  private SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

  private static final Lock odometryLock = new ReentrantLock();

  private static Drive instance;

  public static Drive initialize(SwerveIO swerveIO){
    if (instance == null){
      instance = new Drive(swerveIO);
    }
    return instance;
  }

  public static Drive getInstance() {
    return instance;
  }
  
  /** Creates a new Drive. */
  public Drive(SwerveIO swerveIO) {
    this.swerveIO = swerveIO;
    //vision init goes here

    //pathplanner/choreo init goes here
  }

  private void updateInputs(){

		odometryLock.lock();
		swerveIO.updateInputs(inputs);
		if (Constants.currentMode != Mode.SIM) {
			//io.updateEstimations(vision);
      //vision here somehow idk check link at the top
		}
		swerveIO.updateOdometry();
		odometryLock.unlock();

		Logger.processInputs("Swerve", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }

  public Command humanDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
    return new RunCommand(
      () -> {
        swerveIO.drive(
          xInput.getAsDouble(), 
          yInput.getAsDouble(), 
          thetaInput.getAsDouble(), 
          true, 
          false
        );
      }
      , getInstance());
  }

}
