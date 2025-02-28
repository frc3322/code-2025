// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import java.util.function.Supplier;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static enum SuperState {
    STOW(
        ClimberConstants.stow,
        ElevatorStates.STOW,
        IntakeStates.OFF,
        PivotStates.STOW,
        WristStates.STOW),
    GROUNDINTAKE(
        ClimberConstants.stow,
        ElevatorStates.GROUND,
        IntakeStates.INTAKE,
        PivotStates.GROUND,
        WristStates.INTAKE),
    GROUNDINTAKEFLIPPED(
        ClimberConstants.stow,
        ElevatorStates.GROUND,
        IntakeStates.INTAKE,
        PivotStates.GROUND,
        WristStates.INTAKE),
    ALGAEGROUNDINTAKE(
        ClimberConstants.stow,
        ElevatorStates.AGROUND,
        IntakeStates.INTAKE,
        PivotStates.AGROUND,
        WristStates.ALGAEGROUNDINTAKE),
    SOURCEINTAKE(
        ClimberConstants.stow,
        ElevatorStates.SOURCE,
        IntakeStates.INTAKE,
        PivotStates.SOURCE,
        WristStates.INTAKE),
    REEFL1(
        ClimberConstants.stow,
        ElevatorStates.L1,
        IntakeStates.OFF,
        PivotStates.L1,
        WristStates.L1OUT),
    REEFL2(
        ClimberConstants.stow,
        ElevatorStates.L2,
        IntakeStates.OFF,
        PivotStates.L2,
        WristStates.OUTAKE),
    REEFL3(
        ClimberConstants.stow,
        ElevatorStates.L3,
        IntakeStates.OFF,
        PivotStates.L3,
        WristStates.OUTAKE),
    REEFL4(
        ClimberConstants.stow,
        ElevatorStates.L4,
        IntakeStates.OFF,
        PivotStates.L4,
        WristStates.OUTAKE),
    AUTOL4(
        ClimberConstants.stow,
        ElevatorStates.L4,
        IntakeStates.OFF,
        PivotStates.STOW,
        WristStates.OUTAKE),
    ALGAEINTAKELOW(
        ClimberConstants.stow,
        ElevatorStates.REEFALGAELOW,
        IntakeStates.INTAKE,
        PivotStates.REEFALGAE,
        WristStates.INTAKE),
    ALGAEINTAKEHIGH(
        ClimberConstants.stow,
        ElevatorStates.REEFALGAEHIGH,
        IntakeStates.INTAKE,
        PivotStates.REEFALGAE,
        WristStates.INTAKE),
    PROCESSOR(
        ClimberConstants.stow,
        ElevatorStates.PROCESSER,
        IntakeStates.OFF,
        PivotStates.PROCESSER,
        WristStates.STOW),
    BARGE(
        ClimberConstants.stow,
        ElevatorStates.BARGE,
        IntakeStates.OFF,
        PivotStates.BARGE,
        WristStates.STOW),
    CLIMB(
        ClimberConstants.deploy,
        ElevatorStates.STOW,
        IntakeStates.OFF,
        PivotStates.GROUND,
        WristStates.STOW),
    CLIMBED(
        ClimberConstants.climb,
        ElevatorStates.STOW,
        IntakeStates.OFF,
        PivotStates.GROUND,
        WristStates.STOW);

    public final double CLIMBER_SETPOINT;
    public final ElevatorStates ELEVATOR_STATE;
    public final IntakeStates INTAKE_STATE;
    public final PivotStates PIVOT_STATE;
    public final WristStates WRIST_STATE;

    private SuperState(
        Double climberSetpoint,
        ElevatorStates elevatorState,
        IntakeStates intakeState,
        PivotStates pivotState,
        WristStates wristState) {
      CLIMBER_SETPOINT = climberSetpoint;
      ELEVATOR_STATE = elevatorState;
      INTAKE_STATE = intakeState;
      PIVOT_STATE = pivotState;
      WRIST_STATE = wristState;
    }
  };

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class CANIDs {
    public static final int pigeonCanId = 2;

    public static final int frontLeftDriveCanId = 5;
    public static final int backLeftDriveCanId = 14;
    public static final int frontRightDriveCanId = 8;
    public static final int backRightDriveCanId = 11;

    public static final int frontLeftTurnCanId = 3;
    public static final int backLeftTurnCanId = 12;
    public static final int frontRightTurnCanId = 6;
    public static final int backRightTurnCanId = 9;

    public static final int frontLeftEncoderCanId = 4;
    public static final int backLeftEncoderCanId = 13;
    public static final int frontRightEncoderCanId = 7;
    public static final int backRightEncoderCanId = 10;

    public static final int elevatorLeftCanId = 15;
    public static final int elevatorRightCanId = 16;

    public static final int pivotLeftCanId = 18;
    public static final int pivotRightCanId = 17;

    public static final int intakeCANId = 54;
    public static final int adjustCANId = 62;

    public static final int leftSensorCAN = 0;
    public static final int rightSensorCAN = 0;

    public static final int wristCANId = 56;

    public static final int winchCANId = 23;
  }

  public static class FieldConstants {
    // LEFT and RIGHT are defined from the perspective of standing in the driver
    // station
    // CENTER is the center of the field, OUTER is more towards the driver station

    public static final class ReefConstants {
      public static enum ReefSides {
        CENTER(autoCoralPosition1, autoCoralPosition12),
        CENTERLEFT(autoCoralPosition11, autoCoralPosition10),
        OUTERLEFT(autoCoralPosition9, autoCoralPosition8),
        OUTER(autoCoralPosition7, autoCoralPosition6),
        OUTERRIGHT(autoCoralPosition5, autoCoralPosition4),
        CENTERRIGHT(autoCoralPosition3, autoCoralPosition2);

        public Supplier<Pose2d> leftPose;
        public Supplier<Pose2d> rightPose;

        private ReefSides(Supplier<Pose2d> leftPose, Supplier<Pose2d> rightPose) {
          this.leftPose = leftPose;
          this.rightPose = rightPose;
        }
      };

      public static final Supplier<Pose2d> reefCenter =
          () -> flipPose(new Pose2d(4.5, 4, new Rotation2d()));

      public static final Supplier<Pose2d> coralPosition1 =
          () -> flipPose(new Pose2d(5.269, 3.862, Rotation2d.fromDegrees(180)));
      public static final Supplier<Pose2d> coralPosition2 =
          () -> flipPose(new Pose2d(5.022, 3.432, Rotation2d.fromDegrees(120)));
      public static final Supplier<Pose2d> coralPosition3 =
          () -> flipPose(new Pose2d(4.737, 3.269, Rotation2d.fromDegrees(120)));
      public static final Supplier<Pose2d> coralPosition4 =
          () -> flipPose(new Pose2d(4.241, 3.268, Rotation2d.fromDegrees(60)));
      public static final Supplier<Pose2d> coralPosition5 =
          () -> flipPose(new Pose2d(3.957, 3.433, Rotation2d.fromDegrees(60)));
      public static final Supplier<Pose2d> coralPosition6 =
          () -> flipPose(new Pose2d(3.709, 3.862, Rotation2d.fromDegrees(0)));
      public static final Supplier<Pose2d> coralPosition7 =
          () -> flipPose(new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)));
      public static final Supplier<Pose2d> coralPosition8 =
          () -> flipPose(new Pose2d(3.957, 4.62, Rotation2d.fromDegrees(-60)));
      public static final Supplier<Pose2d> coralPosition9 =
          () -> flipPose(new Pose2d(4.242, 4.783, Rotation2d.fromDegrees(-60)));
      public static final Supplier<Pose2d> coralPosition10 =
          () -> flipPose(new Pose2d(4.737, 4.784, Rotation2d.fromDegrees(-120)));
      public static final Supplier<Pose2d> coralPosition11 =
          () -> flipPose(new Pose2d(5.021, 4.619, Rotation2d.fromDegrees(-120)));
      public static final Supplier<Pose2d> coralPosition12 =
          () -> flipPose(new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)));

      public static final double robotWidth = 40 * 0.0254;
      public static final double offsetDistance = (-robotWidth / 2) - .2;
      public static final double unOffsetDistance = offsetDistance + (robotWidth / 2);

      public static final Supplier<Pose2d> autoCoralPosition1 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(5.269, 3.862, Rotation2d.fromDegrees(180)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition2 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(5.022, 3.432, Rotation2d.fromDegrees(120)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition3 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(4.737, 3.269, Rotation2d.fromDegrees(120)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition4 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(4.241, 3.268, Rotation2d.fromDegrees(60)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition5 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(3.957, 3.433, Rotation2d.fromDegrees(60)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition6 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(3.709, 3.862, Rotation2d.fromDegrees(0)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition7 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition8 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(3.957, 4.62, Rotation2d.fromDegrees(-60)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition9 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(4.242, 4.783, Rotation2d.fromDegrees(-60)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition10 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(4.737, 4.784, Rotation2d.fromDegrees(-120)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition11 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(5.021, 4.619, Rotation2d.fromDegrees(-120)), offsetDistance));
      public static final Supplier<Pose2d> autoCoralPosition12 =
          () ->
              flipPose(
                  localOffsetPose2d(
                      new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180)), offsetDistance));
    }

    /**
     * Returns a new Pose2d object that is offset from the given pose by a specified distance. The
     * offset is applied in the direction of the pose's current rotation.
     *
     * @param pose The original Pose2d object to offset.
     * @param distance The distance to offset the pose.
     * @return A new Pose2d object that is offset from the original pose by the specified distance.
     */
    public static Pose2d localOffsetPose2d(Pose2d pose, double distance) {
      return new Pose2d(
          pose.getTranslation().getX() + distance * Math.cos(pose.getRotation().getRadians()),
          pose.getTranslation().getY() + distance * Math.sin(pose.getRotation().getRadians()),
          pose.getRotation());
    }

    /**
     * Flips the given pose if the robot is on the Red alliance. The pose is flipped around the
     * center of the field.
     *
     * @param pose The original pose to be potentially flipped.
     * @return The flipped pose if on the Red alliance, otherwise the original pose.
     */
    public static Pose2d flipPose(Pose2d pose) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          Translation2d translation = new Translation2d(17.548 / 2, 8.052 / 2);
          return new Pose2d(
              translation.plus(translation.minus(pose.getTranslation())),
              pose.getRotation().rotateBy(new Rotation2d(Math.PI)));
        }
        return pose;
      }
      return pose;
    }

    public static final class SourceConstants {
      public static final Supplier<Pose2d> leftSource =
          () -> flipPose(new Pose2d(.8, 7.5, Rotation2d.fromDegrees(-35)));
      public static final Supplier<Pose2d> rightSource =
          () -> flipPose(new Pose2d(0.8, 0.5, Rotation2d.fromDegrees(35)));
    }

    public static final class PoseMethods {
      public static Pose2d flipPose(Pose2d poseToFlip) {
        return null;
      }

      // TODO: In these checks include an optional velocity that expands the radius

      // Js make another one that takes the velocity, and shift the coords of the
      // target by the
      // velocity.
      public static boolean atPose(
          Pose2d currentPose,
          Pose2d targetPose,
          double translationThreshold,
          double rotationThreshold) {
        boolean translationInThreshold =
            atTranslation(
                currentPose.getTranslation(), targetPose.getTranslation(), translationThreshold);
        boolean rotationInThreshold =
            atRotation(currentPose.getRotation(), targetPose.getRotation(), rotationThreshold);

        if (rotationThreshold == 0) {
          return translationInThreshold;
        }
        if (translationThreshold == 0) {
          return rotationInThreshold;
        }

        return translationInThreshold && rotationInThreshold;
      }

      public static boolean atTranslation(
          Translation2d currentTranslation, Translation2d targetTranslation, double threshold) {
        Translation2d robotTranslation = currentTranslation;

        return robotTranslation.getDistance(targetTranslation) < threshold;
      }

      public static boolean atRotation(
          Rotation2d currentRotation, Rotation2d targetRotation, double threshold) {
        Rotation2d robotRotation = currentRotation;

        double rotOffset = robotRotation.getDegrees() - targetRotation.getDegrees();

        return Math.abs(rotOffset) < threshold;
      }

      public static final double getAngleToPoseRads(
          Pose2d currentPose, Pose2d targetPose, double offsetRads, boolean reversable) {
        Translation2d targetTranslation = targetPose.getTranslation();
        Pose2d robotPose = currentPose;

        // Corrected formula with correct X and Y difference
        double dx = targetTranslation.getX() - robotPose.getX();
        double dy = targetTranslation.getY() - robotPose.getY();

        // Calculate the angle in radians
        double calculatedAngleRads = Math.atan2(dy, dx);

        if ((calculatedAngleRads + offsetRads) > Math.PI) {
          return -Math.PI + ((calculatedAngleRads + offsetRads) % Math.PI);
        } else if ((calculatedAngleRads + offsetRads) < -Math.PI) {
          return Math.PI + ((calculatedAngleRads + offsetRads) % Math.PI);
        }
        return calculatedAngleRads + offsetRads;
      }

      public static final boolean reverseSideScoring(Pose2d robotPose) {
        return Math.abs(
                getAngleToPoseRads(
                        robotPose, ReefConstants.reefCenter.get(), -Math.PI / 2, true) // Math.PI/2)
                    - robotPose.getRotation().getRadians())
            > Math.PI / 2;
      }
    }
  }
}
