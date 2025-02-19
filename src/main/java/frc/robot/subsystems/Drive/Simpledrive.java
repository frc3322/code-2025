package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.commands.DriveCommands;

public class Simpledrive {

  private ProfiledPIDController xPID;
  private ProfiledPIDController yPID;
  private ProfiledPIDController thetaPID;

  private Pose2d targetPose;

  private Drive drivetrain;

  public Simpledrive(
      ProfiledPIDController xPidController,
      ProfiledPIDController yPidController,
      ProfiledPIDController thetaPidController,
      Drive drivetrain) {
    this.xPID = xPidController;
    this.yPID = yPidController;
    this.thetaPID = thetaPidController;
    this.drivetrain = drivetrain;
  }

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  public double getXSpeed() {
    return xPID.calculate(drivetrain.getPose().getX(), targetPose.getX());
  }

  public double getYSpeed() {
    return yPID.calculate(drivetrain.getPose().getY(), targetPose.getY());
  }

  public double getThetaSpeed() {
    return thetaPID.calculate(
        drivetrain.getPose().getRotation().getRadians(), targetPose.getRotation().getRadians());
  }

  public void autoDrive(Pose2d currentPose) {
    DriveCommands.directDrive(
        drivetrain, () -> getXSpeed(), () -> getYSpeed(), () -> getThetaSpeed());
  }
}
