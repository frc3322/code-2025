package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    
    public static class PIDControllers {
        
    }

    public static enum ElevatorStates {
        STOW, // Elevator at bottom
        GROUND, // Coral ground intake
        AGROUND, // Algae ground intake
        L1, // L1 coral scoring
        L2, // L2 coral scoring
        L3, // L3 coral scoring
        L4, // L4 coral scoring
        REEFALGAELOW, // Algae from lower reef
        REEFALGAEHIGH, // Algae from upper reef
        PROCESSER, // Scoring Algae in processer
        BARGE // Scoring Algae in barge
    }
}
