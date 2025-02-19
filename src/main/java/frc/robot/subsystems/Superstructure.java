// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.FieldConstants.SourceConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSides;
import frc.robot.Constants.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.Wrist;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Climber climber;
  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;

  private SuperState superState = SuperState.STOW;

  private SuperState targetLevel = SuperState.REEFL1;

  private Pose2d targetReefPose = new Pose2d();

  private Pose2d[] reefPoses;

  /** Creates a new Superstructure. */
  public Superstructure(
      Climber climber, Elevator elevator, Intake intake, Pivot pivot, Wrist wrist) {
    this.climber = climber;
    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.wrist = wrist;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("SuperStructure/State", superState);
    Logger.recordOutput("SuperStructure/Target Level", targetLevel);

    Logger.recordOutput("SuperStructure/1", targetReefPose == ReefSides.CENTER.leftPose.get());
    Logger.recordOutput("SuperStructure/12", targetReefPose == ReefSides.CENTER.rightPose.get());
    Logger.recordOutput("SuperStructure/11", targetReefPose == ReefSides.CENTERLEFT.leftPose.get());
    Logger.recordOutput(
        "SuperStructure/10", targetReefPose == ReefSides.CENTERLEFT.rightPose.get());
    Logger.recordOutput("SuperStructure/9", targetReefPose == ReefSides.OUTERLEFT.leftPose.get());
    Logger.recordOutput("SuperStructure/8", targetReefPose == ReefSides.OUTERLEFT.rightPose.get());
    Logger.recordOutput("SuperStructure/7", targetReefPose == ReefSides.OUTER.leftPose.get());
    Logger.recordOutput("SuperStructure/6", targetReefPose == ReefSides.OUTER.rightPose.get());
    Logger.recordOutput("SuperStructure/5", targetReefPose == ReefSides.OUTERRIGHT.leftPose.get());
    Logger.recordOutput("SuperStructure/4", targetReefPose == ReefSides.OUTERRIGHT.rightPose.get());
    Logger.recordOutput("SuperStructure/3", targetReefPose == ReefSides.CENTERRIGHT.leftPose.get());
    Logger.recordOutput(
        "SuperStructure/2", targetReefPose == ReefSides.CENTERRIGHT.rightPose.get());

    // make reef pose list
    reefPoses = new Pose2d[12];
    reefPoses[0] = ReefSides.CENTER.leftPose.get();
    reefPoses[1] = ReefSides.CENTER.rightPose.get();
    reefPoses[2] = ReefSides.CENTERLEFT.leftPose.get();
    reefPoses[3] = ReefSides.CENTERLEFT.rightPose.get();
    reefPoses[4] = ReefSides.OUTERLEFT.leftPose.get();
    reefPoses[5] = ReefSides.OUTERLEFT.rightPose.get();
    reefPoses[6] = ReefSides.OUTER.leftPose.get();
    reefPoses[7] = ReefSides.OUTER.rightPose.get();
    reefPoses[8] = ReefSides.OUTERRIGHT.leftPose.get();
    reefPoses[9] = ReefSides.OUTERRIGHT.rightPose.get();
    reefPoses[10] = ReefSides.CENTERRIGHT.leftPose.get();
    reefPoses[11] = ReefSides.CENTERRIGHT.rightPose.get();

    Logger.recordOutput("FieldConstants/Reef Poses", reefPoses);

    Logger.recordOutput("FieldConstants/rightSource", SourceConstants.rightSource.get());
    Logger.recordOutput("FieldConstants/leftSource", SourceConstants.leftSource.get());
  }

  public SuperState getSuperState() {
    return this.superState;
  }

  public SuperState getTargetLevel() {
    return targetLevel;
  }

  public Pose2d getTargetReefPose() {
    return targetReefPose;
  }

  public ReefSides chooseReefSideFromJoystick(double x, double y) {
    double angle = (Math.atan2(x, y) + Math.PI) * 180 / Math.PI;
    int side = (int) (angle / 60) % 6;

    return ReefConstants.ReefSides.values()[side];
  }

  public Command setSuperStateCommand(SuperState superState) {
    return new InstantCommand(
        () -> {
          this.superState = superState;
          climber.setFlipState(superState.CLIMBER_STATE);
          elevator.setState(superState.ELEVATOR_STATE);
          intake.setState(superState.INTAKE_STATE);
          pivot.setState(superState.PIVOT_STATE);
          wrist.setState(superState.WRIST_STATE);
        },
        this);
  }

  public Command setSuperStateCommand(SuperState superState, boolean pivotFlipped) {
    return new SequentialCommandGroup(
        new InstantCommand(
            () -> {
              this.superState = superState;
              climber.setFlipState(superState.CLIMBER_STATE);
              elevator.setState(superState.ELEVATOR_STATE);
              intake.setState(superState.INTAKE_STATE);
              pivot.setState(superState.PIVOT_STATE);
              wrist.setState(superState.WRIST_STATE);
            },
            this),
        pivot.setDirectionBooleanCommand(pivotFlipped));
  }

  public Command setSuperStateCommand(Supplier<SuperState> superStateSupplier) {
    return new InstantCommand(
        () -> {
          this.superState = superStateSupplier.get();
          climber.setFlipState(superState.CLIMBER_STATE);
          elevator.setState(superState.ELEVATOR_STATE);
          intake.setState(superState.INTAKE_STATE);
          pivot.setState(superState.PIVOT_STATE);
          wrist.setState(superState.WRIST_STATE);
        },
        this);
  }

  public Command scoreCommand(SuperState superState) {
    if (superState == SuperState.REEFL1
        || superState == SuperState.PROCESSOR
        || superState == SuperState.BARGE) {
      return intake.setIntakeStateCommand(IntakeStates.REVERSE);
    }
    if (pivot.getDirectionReversed()) {
      return intake.setIntakeStateCommand(IntakeStates.OUTTAKEBACKWARD);
    }
    return intake.setIntakeStateCommand(IntakeStates.OUTTAKEFORWARD);
  }

  public Command scoreCommand(Supplier<SuperState> superStateSupplier) {
    if (superStateSupplier.get() == SuperState.REEFL1
        || superStateSupplier.get() == SuperState.PROCESSOR
        || superStateSupplier.get() == SuperState.BARGE) {
      return intake.setIntakeStateCommand(IntakeStates.REVERSE);
    }
    if (pivot.getDirectionReversed()) {
      return intake.setIntakeStateCommand(IntakeStates.OUTTAKEBACKWARD);
    }
    return intake.setIntakeStateCommand(IntakeStates.OUTTAKEFORWARD);
  }

  public Command setTargetLevelCommand(SuperState targetLevel) {
    return new InstantCommand(() -> this.targetLevel = targetLevel);
  }

  public Command setTargetReefPoseCommand(
      boolean left, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return new InstantCommand(
        () -> {
          ReefSides targetSide =
              chooseReefSideFromJoystick(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          if (left) {
            targetReefPose = targetSide.leftPose.get();
          } else {
            targetReefPose = targetSide.rightPose.get();
          }
        });
  }

  public Command setTargetReefPoseCommand(Supplier<Pose2d> targetReefPoseSupplier) {
    return new InstantCommand(
        () -> {
          this.targetReefPose = targetReefPoseSupplier.get();
        });
  }
}
