package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import java.util.function.BooleanSupplier;
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

  private Supplier<Double> heightSupplier;

  private double wristAngle = 0;

  public static Wrist initialize(WristIO wristIO, Supplier<Pose2d> drivetrainPoseSupplier) {
    if (instance == null) {
      instance = new Wrist(wristIO, drivetrainPoseSupplier);
    }
    return instance;
  }

  public static Wrist getInstance() {
    return instance;
  }

  /** Creates a new Wrist. */
  public Wrist(WristIO wristIO, Supplier<Pose2d> drivetrainPoseSupplier) {
    this.wristIO = wristIO;
    this.drivetrainPoseSupplier = drivetrainPoseSupplier;
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

  private void setState(WristStates wristState) {
    this.wristState = wristState;
  }

  private double getWristOffset(WristStates wristState) {
    if (wristState == WristStates.OUTAKE) {
      double distance =
          drivetrainPoseSupplier
              .get()
              .getTranslation()
              .getDistance(targetPositionSupplier.get().getTranslation());
      if (distance < WristConstants.minDistanceAutoAdjust) {
        return Math.atan(distance / heightSupplier.get());
      }
    }
    return 0;
  }

  private void goToPositionLimited(double position) {
    wristIO.goToPosition(
        Math.min(
            WristConstants.wristMaxRotations,
            Math.max(WristConstants.wristMinRotations, position)));
  }

  public Command goToStateCommand(
      Supplier<WristStates> wristStateSupplier, BooleanSupplier flippedPositionSupplier) {
    return new RunCommand(
        () -> {
          WristStates wristState = wristStateSupplier.get();
          double realWristSetpont =
              flippedPositionSupplier.getAsBoolean()
                  ? -(.5 - wristState.wristSetpoint)
                  : wristState.wristSetpoint;
          goToPositionLimited(realWristSetpont + getWristOffset(wristState));
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
