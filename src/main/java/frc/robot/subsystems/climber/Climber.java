// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public static SparkMax climbMotor =
      new SparkMax(Constants.CANIDs.winchCANId, MotorType.kBrushless);
  public static RelativeEncoder encoder = climbMotor.getEncoder();

  private double setpoint = 0;

  /** Creates a new Climber. */
  public Climber() {
    if (setpoint > encoder.getPosition()) {
      climbMotor.set(1);
    }
    if (setpoint < encoder.getPosition()) {
      climbMotor.set(-1);
    }
  }

  public void setClimberSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public Command setClimberSetpointCommand(double setpoint) {
    return new InstantCommand(() -> setClimberSetpoint(setpoint));
  }

  public Command climberGoToPositionCommand() {
    return new RunCommand(
        () -> {
          if (setpoint > encoder.getPosition()) {
            climbMotor.set(.25);
          }
          if (setpoint < encoder.getPosition()) {
            climbMotor.set(-.25);
          }
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Climber/climbPosition", encoder.getPosition());
  }
}
