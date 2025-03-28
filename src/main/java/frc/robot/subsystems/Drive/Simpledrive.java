package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.SuperState;
import frc.robot.commands.DriveCommands;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Simpledrive {

  private ProfiledPIDController xPID;
  private ProfiledPIDController yPID;
  private ProfiledPIDController thetaPID;

  private Pose2d targetPose;

  private Rotation2d targetRotation;

  private Drive drivetrain;

  private boolean enabled = true;

  public Simpledrive(Drive drivetrain) {
    this.drivetrain = drivetrain;
    xPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPx.get(),
            DriveConstants.SimpleDriveConstants.kIx.get(),
            DriveConstants.SimpleDriveConstants.kDx.get(),
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityX,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationX));

    xPID.setIZone(DriveConstants.SimpleDriveConstants.kIzoneX);

    yPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPy.get(),
            DriveConstants.SimpleDriveConstants.kIy.get(),
            DriveConstants.SimpleDriveConstants.kDy.get(),
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityY,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationY));

    yPID.setIZone(DriveConstants.SimpleDriveConstants.kIzoneY);

    thetaPID =
        new ProfiledPIDController(
            DriveConstants.SimpleDriveConstants.kPtheta.get(),
            DriveConstants.SimpleDriveConstants.kItheta.get(),
            DriveConstants.SimpleDriveConstants.kDtheta.get(),
            new Constraints(
                DriveConstants.SimpleDriveConstants.kMaxVelocityTheta,
                DriveConstants.SimpleDriveConstants.kMaxAccelerationTheta));

    thetaPID.enableContinuousInput(Math.PI, -Math.PI);
  }

  public void setTargetPose(Pose2d targetPose) {
    this.targetPose = targetPose;
  }

  public void setTargetRotation(Rotation2d targeRotation) {
    this.targetRotation = targeRotation;
  }

  public Pose2d getTargetPose() {
    return targetPose;
  }

  public Rotation2d getTargetRotation() {
    return targetRotation;
  }

  public double getXSpeed() {
    return xPID.calculate(drivetrain.getPose().getX(), targetPose.getX());
  }

  public double getYSpeed() {
    return yPID.calculate(drivetrain.getPose().getY(), targetPose.getY());
  }

  public double getThetaSpeed() {
    return thetaPID.calculate(
        drivetrain.getPose().getRotation().getRadians(), targetRotation.getRadians());
  }

  public Pose2d getTargetReefPose(Pose2d targetPose, SuperState getTargetLevel) {
    return getTargetReefPose(() -> targetPose, () -> getTargetLevel);
  }

  public Pose2d getTargetReefPose(Pose2d targetPose, Supplier<SuperState> getTargetLevel) {
    return getTargetReefPose(() -> targetPose, getTargetLevel);
  }

  public Pose2d getTargetReefPose(
      Supplier<Pose2d> targetPoseSupplier, Supplier<SuperState> getTargetLevel) {
    Pose2d modifiedTargetPose = targetPoseSupplier.get();

    xPID.setP(DriveConstants.SimpleDriveConstants.kPx.get());
    xPID.setI(DriveConstants.SimpleDriveConstants.kIx.get());
    xPID.setD(DriveConstants.SimpleDriveConstants.kDx.get());

    yPID.setP(DriveConstants.SimpleDriveConstants.kPy.get());
    yPID.setI(DriveConstants.SimpleDriveConstants.kIy.get());
    yPID.setD(DriveConstants.SimpleDriveConstants.kDy.get());

    thetaPID.setP(DriveConstants.SimpleDriveConstants.kPtheta.get());
    thetaPID.setI(DriveConstants.SimpleDriveConstants.kItheta.get());
    thetaPID.setD(DriveConstants.SimpleDriveConstants.kDtheta.get());

    // move robot so bumpers touch the reef if we are in L2 or L3
    if (getTargetLevel.get() == SuperState.REEFL3
        || getTargetLevel.get() == SuperState.REEFL2
        || getTargetLevel.get() == SuperState.REEFL1) {
      modifiedTargetPose =
          Constants.FieldConstants.localOffsetPose2d(
              modifiedTargetPose, Constants.FieldConstants.getOffsetL1To3());
    } else {
      modifiedTargetPose =
          Constants.FieldConstants.localOffsetPose2d(
              modifiedTargetPose, Constants.FieldConstants.getOffsetL4());
    }

    Pose2d posePlus90 =
        modifiedTargetPose.rotateAround(
            modifiedTargetPose.getTranslation(), new Rotation2d(Math.PI / 2));
    Pose2d poseMinus90 =
        modifiedTargetPose.rotateAround(
            modifiedTargetPose.getTranslation(), new Rotation2d(-Math.PI / 2));

    double posePlus90Yaw = posePlus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees();
    double poseMinus90Yaw = poseMinus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees();

    Logger.recordOutput(
        "Simpledrive/Pose Based angle + 90",
        posePlus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees());
    Logger.recordOutput(
        "Simpledrive/Pose Based angle - 90",
        poseMinus90.relativeTo(drivetrain.getPose()).getRotation().getDegrees());

    // if pick lowest rotational distance
    if (Math.abs(posePlus90Yaw) < Math.abs(poseMinus90Yaw)) {
      modifiedTargetPose =
          modifiedTargetPose.rotateAround(
              modifiedTargetPose.getTranslation(), new Rotation2d(Math.PI / 2));
    } else {
      modifiedTargetPose =
          modifiedTargetPose.rotateAround(
              modifiedTargetPose.getTranslation(), new Rotation2d(-Math.PI / 2));
    }

    double yAdjustDistance = Constants.FieldConstants.ReefConstants.yAdjustDistance.get();
    modifiedTargetPose =
        new Pose2d(
            modifiedTargetPose.getX()
                + yAdjustDistance * Math.cos(modifiedTargetPose.getRotation().getRadians()),
            modifiedTargetPose.getY()
                + yAdjustDistance * Math.sin(modifiedTargetPose.getRotation().getRadians()),
            modifiedTargetPose.getRotation());

    Logger.recordOutput("Simpledrive/Adjusted Pose", modifiedTargetPose);

    return modifiedTargetPose;
  }

  public Pose2d getClosestCoralPose() {
    return drivetrain
        .getPose()
        .nearest(
            Arrays.asList(
                ReefConstants.coralPosition1.get(),
                ReefConstants.coralPosition2.get(),
                ReefConstants.coralPosition3.get(),
                ReefConstants.coralPosition4.get(),
                ReefConstants.coralPosition5.get(),
                ReefConstants.coralPosition6.get(),
                ReefConstants.coralPosition7.get(),
                ReefConstants.coralPosition8.get(),
                ReefConstants.coralPosition9.get(),
                ReefConstants.coralPosition10.get(),
                ReefConstants.coralPosition11.get(),
                ReefConstants.coralPosition12.get()));
  }

  public Command autoDriveToReef(
      Supplier<Pose2d> targetPoseSupplier, Supplier<SuperState> getTargetLevel) {
    // This is elliot and gray's child
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              setTargetPose(getTargetReefPose(targetPoseSupplier, getTargetLevel));
              setTargetRotation(targetPose.getRotation());
              xPID.reset(drivetrain.getPose().getX());
              yPID.reset(drivetrain.getPose().getY());
              thetaPID.reset(drivetrain.getPose().getRotation().getRadians());
            }),
        DriveCommands.directDrive(
            drivetrain, () -> getXSpeed(), () -> getYSpeed(), () -> getThetaSpeed()));
  }

  public Command autoDriveToPose(Supplier<Pose2d> targetPoseSupplier) {
    // This is elliot and gray's child
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              setTargetPose(targetPoseSupplier.get());
              setTargetRotation(targetPose.getRotation());
              xPID.reset(drivetrain.getPose().getX());
              yPID.reset(drivetrain.getPose().getY());
              thetaPID.reset(drivetrain.getPose().getRotation().getRadians());
            }),
        DriveCommands.directDrive(
            drivetrain, () -> getXSpeed(), () -> getYSpeed(), () -> getThetaSpeed()));
  }

  public Command autoRotateToPose(DoubleSupplier xInput, DoubleSupplier yInput, Supplier<Pose2d> targetPoseSupplier) {
    // This is elliot and gray's child
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              setTargetPose(targetPoseSupplier.get());
              setTargetRotation(targetPose.getRotation());
              xPID.reset(drivetrain.getPose().getX());
              yPID.reset(drivetrain.getPose().getY());
              thetaPID.reset(drivetrain.getPose().getRotation().getRadians());
            }),
        DriveCommands.directDrive(
            drivetrain, xInput, yInput, () -> getThetaSpeed()));
  }

  public Command turnToAngleCommand(
      DoubleSupplier xSpeed, DoubleSupplier ySpeed, Supplier<Rotation2d> targetRotationSupplier) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              setTargetRotation(targetRotationSupplier.get());
              thetaPID.reset(drivetrain.getPose().getRotation().getRadians());
            }),
        DriveCommands.directDrive(drivetrain, xSpeed, ySpeed, () -> getThetaSpeed()));
  }

  public boolean getEnabled() {
    return enabled;
  }

  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  public void resetPIDs(Pose2d position, Pose2d velocity) {
    xPID.reset(position.getX(), velocity.getX());
    yPID.reset(position.getY(), velocity.getX());
    thetaPID.reset(position.getRotation().getRadians(), velocity.getRotation().getRadians());
  }

  public Command resetPIDCommand(Supplier<Pose2d> position, Supplier<Pose2d> velocity) {
    return new InstantCommand(() -> resetPIDs(position.get(), velocity.get()));
  }
}
