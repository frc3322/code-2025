package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {

    @AutoLog
    public class WristIOInputs {
        public double motorPower = 0;
    
        public double absolutePosition = 0;
        public double absoluteVelocity = 0;
    
        public double setpoint = 0;
        public double pidOut = 0;
    
        public boolean atGoal = false;
    }
    
    public default void goToPosition(double positionRotations) {}
    
    public default void updateInputs(WristIOInputsAutoLogged inputs) {}
}