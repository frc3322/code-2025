package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.wrist.WristConstants.WristStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  private final WristIO wristIO;

  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static Wrist instance;

  private WristStates wristState = WristStates.STOW;

  private boolean atGoal = false;

  public static Wrist initialize(WristIO wristIO) {
    if (instance == null) {
      instance = new Wrist(wristIO);
    }
    return instance;
  }

  public static Wrist getInstance() {
    return instance;
  }

  /** Creates a new Wrist. */
  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  public void updateInputs() {
    wristIO.updateInputs(inputs);
    atGoal = inputs.atGoal;

    Logger.processInputs("Wrist", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
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

  public Command goToStateCommand(Supplier<WristStates> wristStateSupplier) {
    return new RunCommand(
        () -> {
          WristStates wristSetpoint = wristStateSupplier.get();
          wristIO.goToPosition(wristSetpoint.wristSetpoint);
        },
        this);
  }

  public Command setStateCommand(WristStates wristState) {
    return new InstantCommand(
        () -> {
          setState(wristState);
          wristIO.goToPosition(wristState.wristSetpoint);
        },
        this);
  }
}
