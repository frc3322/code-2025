package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final int intakeMotorCurrentLimit = 40;
  public static final int adjustMotorCurrentLimit = 0;

  public static final class SimConstants {
    public static final double intakeV = 0.1;
    public static final double intakeA = 0.1;
    public static final double adjustV = 0.1;
    public static final double adjustA = 0.1;
  }

  public static enum IntakeStates {
    INTAKE(1),
    OUTTAKE(-0.5),
    REVERSE(-1),
    SOFTINTAKE(0.5),
    OFF(0);

    public double intakeVelocity;

    private IntakeStates(double intakeV) {
      intakeVelocity = intakeV;
    }
  }
}
