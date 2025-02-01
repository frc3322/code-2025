package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public class PivotIOInputs {
        public double leftMotorPower = 0;
        public double rightMotorPower = 0;

        public double leftMotorPosition = 0;
        public double rightMotorPosition = 0;
        public double absolutePosition = 0;

        public double leftMotorVelocity = 0;
        public double rightMotorVelocity = 0;
        public double absoluteVelocity = 0;

    }
}
