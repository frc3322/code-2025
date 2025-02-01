// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  
  private final ElevatorIO elevatorIO;
  
  private static Elevator instance;
  
  public static Elevator initialize(ElevatorIO elevatorIO) {
    if (instance == null){
      instance = new Elevator(elevatorIO);
    }
    return instance;
  }

  public static Elevator getInstance(){
    return instance;
  }
  
  /** Creates a new elevator. */
  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
