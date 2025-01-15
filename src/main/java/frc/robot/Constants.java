package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Constants {
    public static final Mode currentMode = Mode.fromState();
    
    public static enum Mode {
        /** Running on a real robot. */
        REAL,
    
        /** Running a physics simulator. */
        SIM,
    
        /** Replaying from a log file. */
        REPLAY;
    
        static Mode fromState() {
          if (Robot.isReal()) {
            return REAL;
          } else {
            return SIM;
          }
        }
      }

    public static final class OIConstants {
        public static final CommandXboxController driverController = new CommandXboxController(0);
        public static final CommandXboxController operatorController = new CommandXboxController(1);
    }

}
