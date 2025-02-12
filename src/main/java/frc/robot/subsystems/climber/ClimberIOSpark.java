package frc.robot.subsystems.climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.CANIDs;
import frc.robot.subsystems.climber.ClimberConstants.ControllerConstraints;

public class ClimberIOSpark implements ClimberIO {
  private SparkMax flipMotor;
  private SparkMax winchMotor;

  private AbsoluteEncoder absoluteEncoder;

  private ProfiledPIDController flipPID;
  private ArmFeedforward flipFeedForward;

  private double setpoint;

  private double pidOutput;
  private double ffOutput;

  public ClimberIOSpark() {

    flipMotor = new SparkMax(CANIDs.flipCANId, MotorType.kBrushless);
    winchMotor = new SparkMax(CANIDs.winchCANId, MotorType.kBrushless);

    SparkMaxConfig flipConfig = new SparkMaxConfig();
    SparkMaxConfig winchConfig = new SparkMaxConfig();

    flipConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ClimberConstants.currentLimit)
        .voltageCompensation(12)
        .inverted(false);

    winchConfig.idleMode(IdleMode.kCoast).inverted(false);

    absoluteEncoder = flipMotor.getAbsoluteEncoder();
    absoluteEncoder = winchMotor.getAbsoluteEncoder();

    flipMotor.configure(flipConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    winchMotor.configure(
        winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    flipPID =
        new ProfiledPIDController(
            ControllerConstraints.kp,
            ControllerConstraints.ki,
            ControllerConstraints.kd,
            new Constraints(
                ControllerConstraints.velocityConstraint,
                ControllerConstraints.accelerationConstraint));

    flipFeedForward =
        new ArmFeedforward(
            ControllerConstraints.ks, ControllerConstraints.kg, ControllerConstraints.kv);
  }

  public void runWinch(double velocity) {
    winchMotor.set(velocity);
  }

  public void goToPosition(double setPoint, double velocity) {
    ffOutput = flipFeedForward.calculate(setPoint, velocity);

    flipPID.setGoal(setPoint);
    pidOutput = flipPID.calculate(absoluteEncoder.getPosition());

    double combinedOutput = ffOutput + pidOutput;
    flipMotor.set(combinedOutput);
  }

  public void setSetPoint(double setpoint) {
    this.setpoint = setpoint;
  }

  public void updateInputs(ClimberIOInputsAutoLogged inputs) {
    inputs.position = flipMotor.getEncoder().getPosition();
    inputs.setpoint = setpoint;
    inputs.pidOut = pidOutput;
    inputs.ffOut = ffOutput;
  }
}
