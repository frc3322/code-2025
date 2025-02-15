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
import frc.robot.Constants.FieldConstants.ReefConstants.ReefSides;
import frc.robot.Constants.SuperState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
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

    Logger.recordOutput("SuperStructure/CenterLeft", targetReefPose == ReefSides.CENTER.leftPose);
    Logger.recordOutput("SuperStructure/CenterRight", targetReefPose == ReefSides.CENTER.rightPose);
    Logger.recordOutput(
        "SuperStructure/CenterLeftLeft", targetReefPose == ReefSides.CENTERLEFT.leftPose);
    Logger.recordOutput(
        "SuperStructure/CenterLeftRight", targetReefPose == ReefSides.CENTERLEFT.rightPose);
    Logger.recordOutput(
        "SuperStructure/OuterLeftLeft", targetReefPose == ReefSides.OUTERLEFT.leftPose);
    Logger.recordOutput(
        "SuperStructure/OuterLeftRight", targetReefPose == ReefSides.OUTERLEFT.rightPose);
    Logger.recordOutput("SuperStructure/OuterLeft", targetReefPose == ReefSides.OUTER.leftPose);
    Logger.recordOutput("SuperStructure/OuterRight", targetReefPose == ReefSides.OUTER.rightPose);
    Logger.recordOutput(
        "SuperStructure/OuterRightLeft", targetReefPose == ReefSides.OUTERRIGHT.leftPose);
    Logger.recordOutput(
        "SuperStructure/OuterRightRight", targetReefPose == ReefSides.OUTERRIGHT.rightPose);
    Logger.recordOutput(
        "SuperStructure/CenterRightLeft", targetReefPose == ReefSides.CENTERRIGHT.leftPose);
    Logger.recordOutput(
        "SuperStructure/CenterRightRight", targetReefPose == ReefSides.CENTERRIGHT.rightPose);
  }

  public SuperState getSuperState() {
    return this.superState;
  }

  public SuperState getTargetLevel() {
    return targetLevel;
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
            targetReefPose = targetSide.leftPose;
          } else {
            targetReefPose = targetSide.rightPose;
          }
        });
  }
}
