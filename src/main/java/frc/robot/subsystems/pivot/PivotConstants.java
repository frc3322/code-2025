package frc.robot.subsystems.pivot;

public class PivotConstants {

  public static class PIDControllers {}

  public static enum PivotStates {
    STOW, // Arm straight up
    GROUND, // Coral ground intake
    AGROUND, // Algae ground intake
    L1, // L1 coral scoring
    L2, // L2 coral scoring
    L3, // L3 coral scoring
    L4, // L4 coral scoring
    REEFALGAE, // Algae from reef
    PROCESSER, // Scoring Algae in processer
    BARGE, // Scoring Algae in barge
  }
}
