// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSides;
import frc.robot.Constants.FieldConstants.SourceConstants;
import frc.robot.Constants.SuperState;
import frc.robot.Constants.SuperState.StateMotion;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Simpledrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.pivot.PivotConstants.StateType;
import frc.robot.subsystems.wrist.Wrist;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

  private final Climber climber;
  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;
  private final Drive drive;
  private final Simpledrive simpledrive;

  private SuperState superState = SuperState.STOW;

  private SuperState targetLevel = SuperState.REEFL4;

  private Pose2d targetReefPose = new Pose2d();

  private boolean semiAutoEnabled = true;

  private Pose2d[] reefPoses;
  private Pose2d[] autoReefPoses;

  Map<SuperState, Command> scoringChoices;
  Map<SuperState, Command> levelChoices;

  /** Creates a new Superstructure. */
  public Superstructure(
      Climber climber,
      Elevator elevator,
      Intake intake,
      Pivot pivot,
      Wrist wrist,
      Drive drive,
      Simpledrive simpledrive) {
    this.climber = climber;
    this.elevator = elevator;
    this.intake = intake;
    this.pivot = pivot;
    this.wrist = wrist;
    this.drive = drive;
    this.simpledrive = simpledrive;

    setupTriggers();

    levelChoices =
        Map.of(
            SuperState.REEFL1, setSuperStateCommand(SuperState.REEFL1),
            SuperState.REEFL2, setSuperStateCommand(SuperState.REEFL2),
            SuperState.REEFL3, setSuperStateCommand(SuperState.REEFL3),
            SuperState.REEFL4, setSuperStateCommand(SuperState.REEFL4));
    scoringChoices =
        Map.of(
            SuperState.REEFL1, l1ScoreCommand(),
            SuperState.REEFL2, l2and3ScoreCommand(),
            SuperState.REEFL3, l2and3ScoreCommand(),
            SuperState.REEFL4, l4ScoreCommand());
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    Logger.recordOutput("SuperStructure/State", superState);
    Logger.recordOutput("SuperStructure/Target Level", targetLevel);
    Logger.recordOutput("SuperStructure/targetPose", targetReefPose);

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

  public Command deployCommand(SuperState superState) {
    Command command =
        new SequentialCommandGroup(
            wrist.setStateCommand(SuperState.STOW.WRIST_STATE).asProxy(),
            new WaitUntilCommand(wrist::isAtGoal),
            pivot.setStateCommand(SuperState.STOW.PIVOT_STATE).asProxy(),
            new WaitUntilCommand(pivot::isAtGoal),
            elevator.setStateCommand(superState.ELEVATOR_STATE).asProxy(),
            new WaitUntilCommand(elevator::isAtGoal),
            pivot.setStateCommand(superState.PIVOT_STATE).asProxy(),
            new WaitUntilCommand(pivot::isAtGoal),
            wrist.setStateCommand(superState.WRIST_STATE).asProxy());

    command.addRequirements(this);
    return command;
  }

  public Command retractCommand(SuperState superState) {
    Command command =
        new SequentialCommandGroup(
            wrist.setStateCommand(superState.WRIST_STATE).asProxy(),
            // new WaitUntilCommand(wrist::isAtGoal),
            pivot.setStateCommand(superState.PIVOT_STATE).asProxy(),
            // new WaitUntilCommand(pivot::isAtGoal),
            elevator.setStateCommand(superState.ELEVATOR_STATE).asProxy());

    command.addRequirements(this);
    return command;
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

  public void setupTriggers() {
    for (SuperState superState : SuperState.values()) {
      Trigger trigger = new Trigger(() -> this.superState == superState);
      if (superState.PIVOT_STATE.stateType == StateType.REEFSCORING) {
        trigger.onTrue(pivot.setFlippedCommand(true));
      }

      if (superState.stateMotion == StateMotion.RETRACT) {
        Command currentCommand = retractCommand(superState);
        trigger.onTrue(currentCommand);
      }
      if (superState.stateMotion == StateMotion.DEPLOY) {
        Command currentCommand = deployCommand(superState);
        trigger.onTrue(currentCommand);
      }
      trigger.onTrue(climber.setClimberSetpointCommand(superState.CLIMBER_SETPOINT));
      trigger.onTrue(intake.setIntakeStateCommand(superState.INTAKE_STATE));
    }
  }

  public ReefSides chooseReefSideFromJoystick(double x, double y) {
    double angle = (Math.atan2(x, y) + Math.PI) * 180 / Math.PI;
    int side = (int) (angle / 60) % 6;

    return ReefConstants.ReefSides.values()[side];
  }

  public void setSemiAutoEnabled(boolean enabled) {
    semiAutoEnabled = enabled;
  }

  public Command setSuperStateCommand(SuperState superState) {
    return new InstantCommand(() -> this.superState = superState);
  }

  public Command setTargetLevelCommand(SuperState targetLevel) {
    return new InstantCommand(() -> this.targetLevel = targetLevel);
  }

  public Command l4ScoreCommand() {
    Command command =
        new SequentialCommandGroup(
                pivot.setStateCommand(PivotStates.L4SCORE).asProxy(),
                new WaitCommand(1),
                intake.setIntakeStateCommand(IntakeStates.OUTTAKE))
            .asProxy();

    command.addRequirements(this);
    return command;
  }

  public Command l2and3ScoreCommand() {
    Command command =
        new SequentialCommandGroup(
            pivot.setStateCommand(PivotStates.L2AND3SCORE).asProxy(),
            new WaitCommand(.5),
            intake.setIntakeStateCommand(IntakeStates.OUTTAKE).asProxy());

    command.addRequirements(this);
    return command;
  }

  public Command l1ScoreCommand() {
    Command command =
        new SequentialCommandGroup(
            pivot.setStateCommand(PivotStates.L1),
            intake.setIntakeStateCommand(IntakeStates.OUTTAKE).asProxy());
    command.addRequirements(this);
    return command;
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

  public void setTargetReefPose(Pose2d targetReefPose) {
    this.targetReefPose = targetReefPose;
  }

  public Command setTargetReefPoseCommand(Supplier<Pose2d> targetReefPoseSupplier) {
    return new InstantCommand(
        () -> {
          setTargetReefPose(targetReefPoseSupplier.get());
        });
  }

  public Command semiAutoScoreCommand() {
    return new ParallelCommandGroup(
        drive
            .driveToPoseCommand(
                () -> simpledrive.getTargetReefPose(this::getTargetReefPose, this::getTargetLevel))
            .andThen(simpledrive.autoDriveToReef(this::getTargetReefPose, this::getTargetLevel)),
        autoScoreSequence());
  }

  public Command driveToLeftSourceCommand() {
    return new SequentialCommandGroup(
        drive.driveToPoseCommand(
            () ->
                FieldConstants.sideOffsetPose2d(
                    SourceConstants.leftSource.get(), -ReefConstants.robotWidth / 2)),
        setSuperStateCommand(SuperState.SOURCEINTAKE).asProxy());
  }

  public Command driveToRightSourceCommand() {
    return new SequentialCommandGroup(
        drive.driveToPoseCommand(
            () ->
                FieldConstants.sideOffsetPose2d(
                    SourceConstants.rightSource.get(), ReefConstants.robotWidth / 2)),
        setSuperStateCommand(SuperState.SOURCEINTAKE).asProxy());
  }

  public Command autoScoreSequence() {
    return new SequentialCommandGroup(
        setSuperStateCommand(SuperState.STOW).asProxy(),
        new WaitUntilCommand(
            () ->
                Constants.FieldConstants.PoseMethods.atPose(
                    drive.getPose(), drive.getTargetReefPose(), 2, 0)),
        new SelectCommand<>(levelChoices, this::getTargetLevel).asProxy(),
        new WaitUntilCommand(() -> elevator.getElevatorState() == targetLevel.ELEVATOR_STATE),
        new WaitUntilCommand(elevator::isAtGoal),
        new WaitUntilCommand(pivot::isAtGoal),
        new WaitUntilCommand(() -> wrist.getWristState() == targetLevel.WRIST_STATE),
        new WaitUntilCommand(wrist::isAtGoal),
        new WaitUntilCommand(
            () ->
                Constants.FieldConstants.PoseMethods.atPose(
                    drive.getPose(), drive.getTargetReefPose(), .1, 5)),
        new SelectCommand<>(scoringChoices, this::getTargetLevel).asProxy());
  }

  public Command autonL4Sequence() {
    return new SequentialCommandGroup(
        setSuperStateCommand(SuperState.STOW).asProxy(),
        drive.setTargetPoseSupplierCommand(this::getTargetReefPose),
        new WaitUntilCommand(
            () ->
                Constants.FieldConstants.PoseMethods.atPose(
                    drive.getPose(), drive.getTargetReefPose(), 2, 0)),
        setSuperStateCommand(SuperState.REEFL4).asProxy(),
        new WaitUntilCommand(() -> elevator.getElevatorState() == targetLevel.ELEVATOR_STATE),
        new WaitUntilCommand(elevator::isAtGoal),
        new WaitUntilCommand(pivot::isAtGoal),
        new WaitUntilCommand(() -> wrist.getWristState() == targetLevel.WRIST_STATE),
        new WaitUntilCommand(wrist::isAtGoal),
        new WaitUntilCommand(
            () ->
                Constants.FieldConstants.PoseMethods.atPose(
                    drive.getPose(), drive.getTargetReefPose(), .1, 5)),
        l4ScoreCommand().asProxy());
  }

  public Command goToTargetLevelCommand() {
    return new InstantCommand(() -> this.superState = this.targetLevel);
  }

  public Trigger getSemiAutoEnabledTrigger() {
    return new Trigger(() -> semiAutoEnabled);
  }

  public Trigger getSemiAutoDisabledTrigger() {
    return new Trigger(() -> !semiAutoEnabled);
  }
}
