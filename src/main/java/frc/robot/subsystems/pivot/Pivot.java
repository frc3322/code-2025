// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.pivot.PivotConstants.StateType;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {

  private final PivotIO pivotIO;

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private static Pivot instance;

  private static PivotVisualizer pivotVisualizer = new PivotVisualizer();

  private PivotStates pivotState = PivotStates.STOW;

  private boolean atGoal = false;

  private double pivotAngle = 0;

  private Elevator elevator = Elevator.getInstance();

  private Supplier<Pose2d> drivetrainPoseSupplier;

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

    if (elevator != null) {
      pivotVisualizer.update(getPivotAngleRadians(), elevator.getElevatorHeightMeters());
    }

    Logger.recordOutput(
        "Pivot/angle to reef",
        Constants.FieldConstants.PoseMethods.getAngleToPoseRads(
            drivetrainPoseSupplier.get(), ReefConstants.reefCenter, -Math.PI / 2, true));
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

  public double getPivotAngleRadians() {
    return pivotAngle;
  }

  public boolean reverseArmDirection(boolean intaking, boolean reefScoring) {
    Pose2d robotPose = drivetrainPoseSupplier.get();
    // See which direction the arm should go while intaking.
    if (intaking) {
      // Near left source?
      if (Constants.FieldConstants.PoseMethods.atTranslation(
          robotPose.getTranslation(),
          Constants.FieldConstants.SourceConstants.leftSource.getTranslation(),
          PivotConstants.sourceDetectionRadiusMeters)) {
        // Rotation close enough?
        if (Math.abs(
                PivotConstants.leftSourceTargetAngleRadians - robotPose.getRotation().getRadians())
            < Math.PI / 2) {
          return true;
        } else return false;

      }
      // Near right source?
      else if (Constants.FieldConstants.PoseMethods.atTranslation(
          robotPose.getTranslation(),
          Constants.FieldConstants.SourceConstants.rightSource.getTranslation(),
          PivotConstants.sourceDetectionRadiusMeters)) {
        if (Math.abs(
                PivotConstants.rightSourceTargetAngleRadians - robotPose.getRotation().getRadians())
            < Math.PI / 2) {
          return false;
        } else return true;
      }
      // Flip towards reef (if not near source, we are most likely targeting dropped coral)
      else {
        return !Constants.FieldConstants.PoseMethods.reverseSideScoring(robotPose);
      }
    }
    else if (reefScoring) {

    }

    return false;
  }

  public boolean getDirectionReversed() {
    boolean intaking = getPivotState().stateType == StateType.INTAKING;
    boolean reefScoring = getPivotState().stateType == StateType.REEFSCORING;
    
    return reverseArmDirection(intaking, reefScoring);
  }

  public PivotStates getPivotState() {
    return pivotState;
  }

  private void setState(PivotStates pivotState) {
    this.pivotState = pivotState;
  }

  public Command goToStateCommand(Supplier<PivotStates> pivotStateSupplier) {
    return new RunCommand(
        () -> {
          PivotStates pivotSetpoint = pivotStateSupplier.get();
          double modifiedArmSetpoint =
              getDirectionReversed() ? -pivotSetpoint.armSetpoint : pivotSetpoint.armSetpoint;
          pivotIO.goToPosition(modifiedArmSetpoint, pivotSetpoint.armVelocity);
        },
        this);
  }

  public Command setStateCommand(PivotStates pivotState) {
    return new InstantCommand(
        () -> {
          setState(pivotState);
          pivotIO.goToPosition(pivotState.armSetpoint, pivotState.armVelocity);
        },
        this);
  }
}
