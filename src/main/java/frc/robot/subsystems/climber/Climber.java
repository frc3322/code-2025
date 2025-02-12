package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.FlipStates;
import frc.robot.subsystems.climber.ClimberConstants.WinchStates;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  public final ClimberIO climberIO;

  private static Climber instance;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private FlipStates flipState = FlipStates.STOW;
  private WinchStates winchState = WinchStates.OFF;

  public static Climber getInstance() {
    return instance;
  }

  public static Climber initialize(ClimberIO climberIO) {
    if (instance == null) {
      instance = new Climber(climberIO);
    }
    return instance;
  }

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  public FlipStates getFlipState() {
    return flipState;
  }

  public WinchStates getWinchState() {
    return winchState;
  }

  public Command goToStateCommand(
      Supplier<FlipStates> flipSupplier, Supplier<WinchStates> winchSupplier) {
    return new RunCommand(
        () -> {
          FlipStates flipstate = flipSupplier.get();
          WinchStates winchState = winchSupplier.get();
          climberIO.goToPosition(flipstate.setpoint);
          climberIO.runWinch(winchState.power);
        },
        this);
  }

  public Command setFlipStateCommand(FlipStates flipState) {
    return new InstantCommand(
        () -> {
          this.flipState = flipState;
        });
  }

  public Command setWinchStateCommand(WinchStates winchState) {
    return new InstantCommand(
        () -> {
          this.winchState = winchState;
        });
  }

  public void updateInputs() {
    climberIO.updateInputs(inputs);

    Logger.processInputs("Climber", inputs);
  }

  @Override
  public void periodic() {
    updateInputs();
  }
}
