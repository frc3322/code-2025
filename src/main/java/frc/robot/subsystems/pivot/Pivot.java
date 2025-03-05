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
  private boolean directionReversed = false;

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
    return atGoal;
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
    boolean result;

    Pose2d robotPose = drivetrainPoseSupplier.get();
    // See which direction the arm should go while intaking.
    boolean intakingFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    boolean intaking = pivotState.stateType == StateType.INTAKING;
    boolean reefScoring = pivotState.stateType == StateType.REEFSCORING;

    if (intaking) {

      if (Math.abs(
              PivotConstants.leftSourceTargetAngleRadians - robotPose.getRotation().getRadians())
          < Math.PI / 2) {
        result = !intakingFlipped;
      } else result = intakingFlipped;

    } else if (reefScoring) {
      result = Constants.FieldConstants.PoseMethods.reverseSideScoring(robotPose);
    } else result = false;

    boolean targetSide = flipped ? result : !result;
    return targetSide;
  }

  public boolean getDirectionReversed() {
    return directionReversed;
  }

  public PivotStates getPivotState() {
    return pivotState;
  }

  public void setState(PivotStates pivotState) {
    pivotIO.presetSetpoint(pivotState.armSetpoint);
    this.pivotState = pivotState;
    this.directionReversed = reverseArmDirection();
  }

  public Command goToStateCommand(Supplier<PivotStates> pivotStateSupplier) {
    return new RunCommand(
        () -> {
          PivotStates pivotSetpoint = pivotStateSupplier.get();
          double modifiedArmSetpoint;

          if (pivotSetpoint == PivotStates.SOURCE) {
            directionReversed =
                !Constants.FieldConstants.PoseMethods.atPose(
                    drivetrainPoseSupplier.get(),
                    Constants.FieldConstants.SourceConstants.leftSource.get(),
                    3.5,
                    90);
          }

          modifiedArmSetpoint =
              getDirectionReversed() ? -pivotSetpoint.armSetpoint : pivotSetpoint.armSetpoint;
          pivotIO.goToPosition(modifiedArmSetpoint, pivotSetpoint.armVelocity);
        },
        this);
  }

  public Command setStateCommand(PivotStates pivotState) {
    return new InstantCommand(
        () -> {
          setState(pivotState);
        });
  }

  public Command setFlippedCommand(boolean flipped) {
    return new InstantCommand(
        () -> {
          this.flipped = flipped;
        });
  }
}
