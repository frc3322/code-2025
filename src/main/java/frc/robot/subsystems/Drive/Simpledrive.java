package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Simpledrive {

  private ProfiledPIDController xPID;
  private ProfiledPIDController yPID;
  private ProfiledPIDController thetaPID;

  private Pose2d targetPose;

  private Drive drivetrain;

  private boolean enabled = true;

  public Simpledrive(Drive drivetrain) {
    this.drivetrain = drivetrain;
    xPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPx,
            DriveConstants.SimpleDriveConstants.kIx,
            DriveConstants.SimpleDriveConstants.kDx,
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityX,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationX));

    xPID.setIZone(DriveConstants.SimpleDriveConstants.kIzoneX);

    yPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPy,
            DriveConstants.SimpleDriveConstants.kIy,
            DriveConstants.SimpleDriveConstants.kDy,
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityY,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationY));

    yPID.setIZone(DriveConstants.SimpleDriveConstants.kIzoneY);

    thetaPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPtheta,
            DriveConstants.SimpleDriveConstants.kItheta,
            DriveConstants.SimpleDriveConstants.kDtheta,
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityTheta,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationTheta));

    thetaPID.enableContinuousInput(Math.PI, -Math.PI);
  }

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  public Pose2d getTargetPose() {
    return targetPose;
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

  public Command autoDrive(Supplier<Pose2d> targetPoseSupplier) {
    // This is elliot and gray's child
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              Pose2d targetPose = targetPoseSupplier.get();

              Pose2d posePlus90 =
                  targetPose.rotateAround(targetPose.getTranslation(), new Rotation2d(Math.PI / 2));
              Pose2d poseMinus90 =
                  targetPose.rotateAround(
                      targetPose.getTranslation(), new Rotation2d(-Math.PI / 2));

              double posePlus90Yaw =
                  posePlus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees();
              double poseMinus90Yaw =
                  poseMinus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees();

              Logger.recordOutput(
                  "Simpledrive/Pose Based angle + 90",
                  posePlus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees());
              Logger.recordOutput(
                  "Simpledrive/Pose Based angle - 90",
                  poseMinus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees());

              // if pick lowest rotational distance
              if (Math.abs(posePlus90Yaw) < Math.abs(poseMinus90Yaw)) {
                targetPose =
                    targetPose.rotateAround(
                        targetPose.getTranslation(), new Rotation2d(Math.PI / 2));
              } else {
                targetPose =
                    targetPose.rotateAround(
                        targetPose.getTranslation(), new Rotation2d(-Math.PI / 2));
              }

              double yAdjustDistance = -.1;
              targetPose =
                  new Pose2d(
                      targetPose.getX()
                          + yAdjustDistance * Math.cos(targetPose.getRotation().getRadians()),
                      targetPose.getY()
                          + yAdjustDistance * Math.sin(targetPose.getRotation().getRadians()),
                      targetPose.getRotation());

              Logger.recordOutput("Simpledrive/Adjusted Pose", targetPose);

              setTargetPose(targetPose);
              xPID.reset(drivetrain.getPose().getX());
              yPID.reset(drivetrain.getPose().getY());
              thetaPID.reset(drivetrain.getPose().getRotation().getRadians());
            }),
        DriveCommands.directDrive(
            drivetrain, () -> getXSpeed(), () -> getYSpeed(), () -> getThetaSpeed()));
  }

  public boolean getEnabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }
}
