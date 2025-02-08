package frc.robot.subsystems.intake;

public class IntakeConstants {

  public static final int intakeMotorCurrentLimit = 0;
  public static final int adjustMotorCurrentLimit = 0;

  public static final class SimConstants {
    public static final double intakeV = 0.1;
    public static final double intakeA = 0.1;
    public static final double adjustV = 0.1;
    public static final double adjustA = 0.1;
  }

  public static enum IntakeStates{
    INTAKE(1, 0),
    OUTTAKE(0.1, 1),
    REVERSE(-1, 0),
    SOFTINTAKE(0.5, 0),
    OFF(0,0);

    public double intakeVelocity;
    public double adjustVelocity;

    private IntakeStates(double intakeV, double adjustV){
      intakeVelocity = intakeV;
      adjustVelocity = adjustV;
    }
  }
}
