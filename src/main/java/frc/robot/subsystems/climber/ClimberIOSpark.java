package frc.robot.subsystems.climber;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ClimberIOSpark implements ClimberIO {
  public SparkMax winchMotor;

  public SparkMaxConfig winchConfig;

  public ProfiledPIDController winchPID;

  public ClimberIOSpark() {
    winchMotor = new SparkMax(ClimberConstants.winchCANID, MotorType.kBrushless);

    winchConfig.idleMode(IdleMode.kBrake);

    winchPID.setPID(ClimberConstants.winchP, ClimberConstants.winchI, ClimberConstants.winchD);
  }

  public void setClimberVelocity(double power) {}

  public void setClimberSetpoint(double setpoint) {}

  public void setWinchVelocity(double power) {
    winchMotor.set(power);
  }

  public void updateInputs(ClimberIOInputs inputs) {}
}
