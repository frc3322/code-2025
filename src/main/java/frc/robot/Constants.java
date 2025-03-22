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
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
        WristStates.STOW,
        StateMotion.RETRACT),
    ALGAESTOW(
        ClimberConstants.stow,
        ElevatorStates.STOW,
        IntakeStates.SOFTINTAKE,
        PivotStates.STOW,
        WristStates.STOW,
        StateMotion.RETRACT),
    GROUNDINTAKE(
        ClimberConstants.stow,
        ElevatorStates.GROUND,
        IntakeStates.INTAKE,
        PivotStates.GROUND,
        WristStates.INTAKE,
        StateMotion.DEPLOY),
    ALGAEGROUNDINTAKE(
        ClimberConstants.stow,
        ElevatorStates.AGROUND,
        IntakeStates.INTAKE,
        PivotStates.AGROUND,
        WristStates.ALGAEGROUNDINTAKE,
        StateMotion.DEPLOY),
    SOURCEINTAKE(
        ClimberConstants.stow,
        ElevatorStates.SOURCE,
        IntakeStates.INTAKE,
        PivotStates.SOURCE,
        WristStates.INTAKE,
        StateMotion.RETRACT),
    REEFL1(
        ClimberConstants.stow,
        ElevatorStates.L1,
        IntakeStates.OFF,
        PivotStates.L1,
        WristStates.L1OUT,
        StateMotion.DEPLOY),
    REEFL2(
        ClimberConstants.stow,
        ElevatorStates.L2,
        IntakeStates.OFF,
        PivotStates.L2,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    REEFL3(
        ClimberConstants.stow,
        ElevatorStates.L3,
        IntakeStates.OFF,
        PivotStates.L3,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    REEFL4(
        ClimberConstants.stow,
        ElevatorStates.L4,
        IntakeStates.OFF,
        PivotStates.L4,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    AUTOL4(
        ClimberConstants.stow,
        ElevatorStates.L4,
        IntakeStates.OFF,
        PivotStates.L4AUTO,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    ALGAEPLUCKLOW(
        ClimberConstants.stow,
        ElevatorStates.REEFALGAELOW,
        IntakeStates.INTAKE,
        PivotStates.REEFALGAE,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    ALGAEPLUCKHIGH(
        ClimberConstants.stow,
        ElevatorStates.REEFALGAEHIGH,
        IntakeStates.INTAKE,
        PivotStates.REEFALGAE,
        WristStates.OUTAKE,
        StateMotion.DEPLOY),
    PROCESSOR(
        ClimberConstants.stow,
        ElevatorStates.GROUND,
        IntakeStates.REVERSE,
        PivotStates.GROUND,
        WristStates.INTAKE,
        StateMotion.DEPLOY),
    BARGE(
        ClimberConstants.stow,
        ElevatorStates.BARGE,
        IntakeStates.SOFTINTAKE,
        PivotStates.BARGE,
        WristStates.STOW,
        StateMotion.RETRACT),
    CLIMB(
        ClimberConstants.deploy,
        ElevatorStates.STOW,
        IntakeStates.OFF,
        PivotStates.CLIMB,
        WristStates.STOW,
        StateMotion.RETRACT),
    CLIMBED(
        ClimberConstants.getClimbSetpointMethod(),
        ElevatorStates.STOW,
        IntakeStates.OFF,
        PivotStates.CLIMB,
        WristStates.STOW,
        StateMotion.RETRACT);

    public static enum StateMotion {
      DEPLOY,
      RETRACT
    }

    public final Supplier<Double> CLIMBER_SETPOINT;
    public final ElevatorStates ELEVATOR_STATE;
    public final IntakeStates INTAKE_STATE;
    public final PivotStates PIVOT_STATE;
    public final WristStates WRIST_STATE;
    public final StateMotion stateMotion;

    private SuperState(
        Supplier<Double> climberSetpoint,
        ElevatorStates elevatorState,
        IntakeStates intakeState,
        PivotStates pivotState,
        WristStates wristState,
        StateMotion stateMotion) {
      CLIMBER_SETPOINT = climberSetpoint;
      ELEVATOR_STATE = elevatorState;
      INTAKE_STATE = intakeState;
      PIVOT_STATE = pivotState;
      WRIST_STATE = wristState;
      this.stateMotion = stateMotion;
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
        CENTER(coralPosition1, coralPosition12),
        CENTERLEFT(coralPosition11, coralPosition10),
        OUTERLEFT(coralPosition9, coralPosition8),
        OUTER(coralPosition7, coralPosition6),
        OUTERRIGHT(coralPosition5, coralPosition4),
        CENTERRIGHT(coralPosition3, coralPosition2);

        public Supplier<Pose2d> leftPose;
        public Supplier<Pose2d> rightPose;

        private ReefSides(Supplier<Pose2d> leftPose, Supplier<Pose2d> rightPose) {
          this.leftPose = leftPose;
          this.rightPose = rightPose;
        }
      };

      public static final Supplier<Pose2d> reefCenter =
          () -> flipPose(new Pose2d(4.5, 4, new Rotation2d()));

      public static final Pose2d blueCoralPosition1 =
          new Pose2d(5.269, 3.862, Rotation2d.fromDegrees(180));
      public static final Pose2d blueCoralPosition2 =
          new Pose2d(5.022, 3.432, Rotation2d.fromDegrees(120));
      public static final Pose2d blueCoralPosition3 =
          new Pose2d(4.737, 3.269, Rotation2d.fromDegrees(120));
      public static final Pose2d blueCoralPosition4 =
          new Pose2d(4.241, 3.268, Rotation2d.fromDegrees(60));
      public static final Pose2d blueCoralPosition5 =
          new Pose2d(3.957, 3.433, Rotation2d.fromDegrees(60));
      public static final Pose2d blueCoralPosition6 =
          new Pose2d(3.709, 3.862, Rotation2d.fromDegrees(0));
      public static final Pose2d blueCoralPosition7 =
          new Pose2d(3.71, 4.19, Rotation2d.fromDegrees(0));
      public static final Pose2d blueCoralPosition8 =
          new Pose2d(3.957, 4.62, Rotation2d.fromDegrees(-60));
      public static final Pose2d blueCoralPosition9 =
          new Pose2d(4.242, 4.783, Rotation2d.fromDegrees(-60));
      public static final Pose2d blueCoralPosition10 =
          new Pose2d(4.737, 4.784, Rotation2d.fromDegrees(-120));
      public static final Pose2d blueCoralPosition11 =
          new Pose2d(5.021, 4.619, Rotation2d.fromDegrees(-120));
      public static final Pose2d blueCoralPosition12 =
          new Pose2d(5.27, 4.19, Rotation2d.fromDegrees(180));

      public static final Pose2d redCoralPosition1 =
          new Pose2d(12.279, 4.240, Rotation2d.fromDegrees(0));
      public static final Pose2d redCoralPosition2 =
          new Pose2d(12.526, 4.620, Rotation2d.fromDegrees(300));
      public static final Pose2d redCoralPosition3 =
          new Pose2d(12.811, 4.783, Rotation2d.fromDegrees(300));
      public static final Pose2d redCoralPosition4 =
          new Pose2d(13.307, 4.784, Rotation2d.fromDegrees(240));
      public static final Pose2d redCoralPosition5 =
          new Pose2d(13.591, 4.619, Rotation2d.fromDegrees(240));
      public static final Pose2d redCoralPosition6 =
          new Pose2d(13.839, 4.190, Rotation2d.fromDegrees(180));
      public static final Pose2d redCoralPosition7 =
          new Pose2d(13.838, 3.812, Rotation2d.fromDegrees(180));
      public static final Pose2d redCoralPosition8 =
          new Pose2d(13.591, 3.432, Rotation2d.fromDegrees(120));
      public static final Pose2d redCoralPosition9 =
          new Pose2d(13.306, 3.269, Rotation2d.fromDegrees(120));
      public static final Pose2d redCoralPosition10 =
          new Pose2d(12.811, 3.268, Rotation2d.fromDegrees(60));
      public static final Pose2d redCoralPosition11 =
          new Pose2d(12.527, 3.433, Rotation2d.fromDegrees(60));
      public static final Pose2d redCoralPosition12 =
          new Pose2d(12.278, 3.862, Rotation2d.fromDegrees(0));

      public static final Supplier<Pose2d> coralPosition1 =
          () -> decidePose(blueCoralPosition1, redCoralPosition1);
      public static final Supplier<Pose2d> coralPosition2 =
          () -> decidePose(blueCoralPosition2, redCoralPosition2);
      public static final Supplier<Pose2d> coralPosition3 =
          () -> decidePose(blueCoralPosition3, redCoralPosition3);
      public static final Supplier<Pose2d> coralPosition4 =
          () -> decidePose(blueCoralPosition4, redCoralPosition4);
      public static final Supplier<Pose2d> coralPosition5 =
          () -> decidePose(blueCoralPosition5, redCoralPosition5);
      public static final Supplier<Pose2d> coralPosition6 =
          () -> decidePose(blueCoralPosition6, redCoralPosition6);
      public static final Supplier<Pose2d> coralPosition7 =
          () -> decidePose(blueCoralPosition7, redCoralPosition7);
      public static final Supplier<Pose2d> coralPosition8 =
          () -> decidePose(blueCoralPosition8, redCoralPosition8);
      public static final Supplier<Pose2d> coralPosition9 =
          () -> decidePose(blueCoralPosition9, redCoralPosition9);
      public static final Supplier<Pose2d> coralPosition10 =
          () -> decidePose(blueCoralPosition10, redCoralPosition10);
      public static final Supplier<Pose2d> coralPosition11 =
          () -> decidePose(blueCoralPosition11, redCoralPosition11);
      public static final Supplier<Pose2d> coralPosition12 =
          () -> decidePose(blueCoralPosition12, redCoralPosition12);

      public static final double robotWidth = 40 * 0.0254;

      public static final LoggedNetworkNumber yAdjustDistance =
          new LoggedNetworkNumber("/Tuning/yAdjustDistance", -.115);

      public static final LoggedNetworkNumber robotL4Offset =
          new LoggedNetworkNumber("/Tuning/L4Offset", .20);
      public static final LoggedNetworkNumber robotL1To3Offset =
          new LoggedNetworkNumber("/Tuning/L1To3Offset", .05);
    }

    public static double getOffsetL4() {
      return (-FieldConstants.ReefConstants.robotWidth / 2)
          - FieldConstants.ReefConstants.robotL4Offset.get();
    }

    public static double getOffsetL1To3() {
      return (-FieldConstants.ReefConstants.robotWidth / 2)
          - FieldConstants.ReefConstants.robotL1To3Offset.get();
    }

    public static Pose2d decidePose(Pose2d bluePose, Pose2d redPose) {
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          return redPose;
        }
        return bluePose;
      }
      return bluePose;
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

    public static Pose2d sideOffsetPose2d(Pose2d pose, double distance) {
      return new Pose2d(
          pose.getTranslation().getX()
              + distance * Math.cos(pose.getRotation().getRadians() + Math.PI / 2),
          pose.getTranslation().getY()
              + distance * Math.sin(pose.getRotation().getRadians() + Math.PI / 2),
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
          () -> flipPose(new Pose2d(.8, 7.5, Rotation2d.fromDegrees(35)));
      public static final Supplier<Pose2d> rightSource =
          () -> flipPose(new Pose2d(0.8, 0.5, Rotation2d.fromDegrees(-35)));
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

      /**
       * Calculates the angle in radians from the current pose to the target pose, with an optional
       * offset and reversibility.
       *
       * @param currentPose The current pose of the robot.
       * @param targetPose The target pose to which the angle is calculated.
       * @param offsetRads An offset in radians to be added to the calculated angle.
       * @param reversable A boolean indicating if the angle calculation should consider
       *     reversibility.
       * @return The angle in radians from the current pose to the target pose, adjusted by the
       *     offset and considering reversibility.
       */
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
                        robotPose,
                        FieldConstants.ReefConstants.reefCenter.get(),
                        -Math.PI / 2,
                        true) // Math.PI/2)
                    - robotPose.getRotation().getRadians())
            > Math.PI / 2;
      }
    }
  }
}
