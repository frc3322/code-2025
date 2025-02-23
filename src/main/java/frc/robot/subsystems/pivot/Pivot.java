// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.pivot.PivotConstants.StateType;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  // --------------------------- Instance Variables ---------------------------
  private static Pivot instance; // Static instance of the Pivot (singleton pattern)
  private static PivotVisualizer pivotVisualizer =
      new PivotVisualizer(); // Static visualizer for pivot system

  // -------------------------- State Variables -----------------------------
  private PivotStates pivotState = PivotStates.STOW; // Current state of the pivot (initially STOW)
  private boolean atGoal = false; // Flag indicating whether the pivot has reached the goal
  private double pivotAngle = 0; // Current angle of the pivot

  // ----------------------- Hardware/Subsystem Instances ---------------------
  private final PivotIO pivotIO; // Pivot IO interface for communication with the pivot system
  private final PivotIOInputsAutoLogged inputs =
      new PivotIOInputsAutoLogged(); // Input handler for automatic logging

  // -------------------------- Control/Logic Flags ---------------------------
  private boolean flipped = false; // Manual control for the flip direction of the pivot
  boolean manual = false; // Flag for enabling manual control (overrides automatic behavior)

  // ------------------------ External Dependencies ---------------------------
  private Supplier<Pose2d>
      drivetrainPoseSupplier; // Supplier for the drivetrain's pose (position and orientation)

  public static Pivot initialize(PivotIO pivotIO, Supplier<Pose2d> drivetrainPoseSupplier) {
    if (instance == null) {
      instance = new Pivot(pivotIO, drivetrainPoseSupplier);
    }
    return instance;
  }

  public static Pivot getInstance() {
    return instance;
  }

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivotIO, Supplier<Pose2d> drivetrainPoseSupplier) {
    this.pivotIO = pivotIO;
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
  }

  public void updateInputs() {
    pivotIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Pivot", inputs);

    pivotAngle = inputs.absolutePosition * 2 * Math.PI;
  }

  @Override
  public void periodic() {
    updateInputs();

    pivotVisualizer.update(getPivotAngleRadians());

    Logger.recordOutput("Pivot/State", getPivotState());

    Logger.recordOutput(
        "Pivot/angle to reef",
        Constants.FieldConstants.PoseMethods.getAngleToPoseRads(
            drivetrainPoseSupplier.get(), ReefConstants.reefCenter.get(), -Math.PI / 2, true));
    Logger.recordOutput(
        "Pivot/atPoseBoolean",
        Math.abs(
                PivotConstants.rightSourceTargetAngleRadians
                    - drivetrainPoseSupplier.get().getRotation().getRadians())
            < Math.PI / 2);
    Logger.recordOutput("Pivot/drivePose", drivetrainPoseSupplier.get());
  }

  public boolean isAtGoal() {
    return Math.abs(pivotAngle) > .2;
  }

  public boolean pastL4Score() {
    return Math.abs(pivotAngle) > .3;
  }

  public boolean pastL2and3Score() {
    return Math.abs(pivotAngle) > .15;
  }

  public double getPivotAngleRadians() {
    return pivotAngle;
  }

  public boolean reverseArmDirection() {
    Pose2d robotPose = drivetrainPoseSupplier.get();
    // See which direction the arm should go while intaking.
    boolean intakingFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    boolean intaking = pivotState.stateType == StateType.INTAKING;
    boolean reefScoring = pivotState.stateType == StateType.REEFSCORING;

    if (intaking) {
      // Near left source?
      if (Constants.FieldConstants.PoseMethods.atTranslation(
          robotPose.getTranslation(),
          Constants.FieldConstants.SourceConstants.leftSource.get().getTranslation(),
          PivotConstants.sourceDetectionRadiusMeters)) {
        // Rotation close enough?
        if (Math.abs(
                PivotConstants.leftSourceTargetAngleRadians - robotPose.getRotation().getRadians())
            < Math.PI / 2) {
          return !intakingFlipped;
        } else return intakingFlipped;

      }
      // Near right source?
      else if (Constants.FieldConstants.PoseMethods.atTranslation(
          robotPose.getTranslation(),
          Constants.FieldConstants.SourceConstants.rightSource.get().getTranslation(),
          PivotConstants.sourceDetectionRadiusMeters)) {
        if (Math.abs(
                PivotConstants.rightSourceTargetAngleRadians - robotPose.getRotation().getRadians())
            < Math.PI / 2) {
          return intakingFlipped;
        } else return !intakingFlipped;
      }
      // Flip towards reef (if not near source, we are most likely targeting dropped coral)
      else {
        return !Constants.FieldConstants.PoseMethods.reverseSideScoring(robotPose);
      }
    } else if (reefScoring) {
      return Constants.FieldConstants.PoseMethods.reverseSideScoring(robotPose);
    }

    return false;
  }

  public boolean getDirectionReversed() {
    return flipped;
  }

  public PivotStates getPivotState() {
    return pivotState;
  }

  public void setState(PivotStates pivotState, BooleanSupplier flippedSupplier) {
    this.pivotState = pivotState;
    this.flipped = flippedSupplier.getAsBoolean();
  }

  public Command goToStateCommand(Supplier<PivotStates> pivotStateSupplier) {
    return new RunCommand(
        () -> {
          PivotStates pivotSetpoint = pivotStateSupplier.get();
          double modifiedArmSetpoint;

          modifiedArmSetpoint = flipped ? -pivotSetpoint.armSetpoint : pivotSetpoint.armSetpoint;
          pivotIO.goToPosition(modifiedArmSetpoint, pivotSetpoint.armVelocity);
        },
        this);
  }

  public Command setStateCommand(PivotStates pivotState, BooleanSupplier flippedSupplier) {
    return new InstantCommand(
        () -> {
          setState(pivotState, flippedSupplier);
        });
  }
}
