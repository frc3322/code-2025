package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static Wrist instance;

  private WristVisualizer visualizer = new WristVisualizer();

  private WristStates wristState = WristStates.STOW;

  private boolean atGoal = false;

  private Elevator elevator = Elevator.getInstance();

  private Pivot pivot = Pivot.getInstance();

  private Supplier<Pose2d> drivetrainPoseSupplier;

  private Supplier<Pose2d> targetPositionSupplier;

  private double wristAngle = 0;

  public static Wrist initialize(
      WristIO wristIO,
      Supplier<Pose2d> drivetrainPoseSupplier,
      Supplier<Pose2d> targetPositionSupplier) {
    if (instance == null) {
      instance = new Wrist(wristIO, drivetrainPoseSupplier, targetPositionSupplier);
    }
    return instance;
  }

  public static Wrist getInstance() {
    return instance;
  }

  /** Creates a new Wrist. */
  public Wrist(
      WristIO wristIO,
      Supplier<Pose2d> drivetrainPoseSupplier,
      Supplier<Pose2d> targetPositionSupplier) {
    this.wristIO = wristIO;
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
    this.targetPositionSupplier = targetPositionSupplier;
  }

  public void updateInputs() {
    wristIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Wrist", inputs);

    wristAngle = WristConstants.rotationsToRadians(inputs.absolutePosition);
  }

  @Override
  public void periodic() {
    updateInputs();

    visualizer.update(wristAngle, pivot.getPivotAngleRadians(), elevator.getElevatorHeightMeters());
  }

  public boolean isAtGoal() {
    return atGoal;
  }

  public WristStates getWristState() {
    return wristState;
  }

  public void setState(WristStates wristState) {
    this.wristState = wristState;
    Logger.recordOutput("Wrist/Wrist State", wristState);
  }

  private double getWristOffset(WristStates wristState) {
    if (wristState == WristStates.OUTAKE) {
      Logger.recordOutput("Wrist/Target Position", targetPositionSupplier.get());
      double localYDistance =
      - targetPositionSupplier
              .get()
              .relativeTo(drivetrainPoseSupplier.get()).getX() - WristConstants.intakeOffset;
      if (targetPositionSupplier.get().getTranslation().getDistance(drivetrainPoseSupplier.get().getTranslation()) < WristConstants.minDistanceAutoAdjust) {
        Logger.recordOutput("Wrist/Within Distance", true);
        Logger.recordOutput("Wrist/Wrist Offset", Math.atan(localYDistance / WristConstants.placementHeight));
        return WristConstants.radiansToRotations(Math.atan(localYDistance / WristConstants.placementHeight));
      }
      else {
        Logger.recordOutput("Wrist/Within Distance", false);
      }
    }
    Logger.recordOutput("Wrist/Wrist Offset", 0);
    return 0;
  }

  private void goToPositionLimited(double position) {
    wristIO.goToPosition(
        Math.min(
            WristConstants.wristMaxRotations,
            Math.max(WristConstants.wristMinRotations, position)));
  }

  public Command goToStateCommand(Supplier<WristStates> wristStateSupplier) {
    return new RunCommand(
        () -> {
          WristStates wristState = wristStateSupplier.get();
          Logger.recordOutput("Wrist/Wrist State", wristState);

          // goToPositionLimited(wristState.wristSetpoint + getWristOffset(wristState));
          goToPositionLimited(wristState.wristSetpoint);
        },
        this);
  }

  public Command setStateCommand(WristStates wristState) {
    return new InstantCommand(
        () -> {
          setState(wristState);
          goToPositionLimited(wristState.wristSetpoint);
        },
        this);
  }
}
