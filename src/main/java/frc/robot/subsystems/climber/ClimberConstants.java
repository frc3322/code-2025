package frc.robot.subsystems.climber;

import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class ClimberConstants {
  public static final Supplier<Double> stow = () -> 0.0;
  public static final Supplier<Double> deploy = () -> 450.0;
  public static LoggedNetworkNumber climb = new LoggedNetworkNumber("/Tuning/Climb Setpoint", 9.0);

  public static Supplier<Double> getClimbSetpointMethod() {
    return () -> climb.get();
  }
}
