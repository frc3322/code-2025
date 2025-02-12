package frc.robot.subsystems.climber;

public class ClimberConstants {
  public static final int currentLimit = 60;

  public class ControllerConstraints {
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;

    public static final double ks = 0;
    public static final double kv = 0;
    public static final double kg = 0;

    public static final double velocityConstraint = 5;
    public static final double accelerationConstraint = 5;
  }

  public static enum FlipStates {
    STOW(0),
    DOWN(-1);

    public double setpoint;

    private FlipStates(double setpoint) {
      this.setpoint = setpoint;
    }
  }

  public static enum WinchStates {
    DOWN(0.1),
    UP(1),
    OFF(0);

    public double power;

    private WinchStates(double power) {
      this.power = power;
    }
  };
}
