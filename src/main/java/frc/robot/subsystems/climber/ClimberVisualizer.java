package frc.robot.subsystems.climber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

public class ClimberVisualizer {
  Mechanism2d chassisMech;
  MechanismRoot2d elevatorRoot;

  public ClimberVisualizer() {
    // chassisMech = new Mechanism2d(Units.inchesToMeters(28), Units.inchesToMeters(28));
    // elevatorRoot = chassisMech.getRoot("base", Units.inchesToMeters(19), 0);
    // elevatorStageOne =
    //     elevatorRoot.append(
    //         new MechanismLigament2d("Elevator stage one", Units.inchesToMeters(40.25), 90));
    // elevatorStageTwo =
    //     elevatorRoot.append(
    //         new MechanismLigament2d("Elevator stage two", Units.inchesToMeters(6), 90));
  }

  public void update(double elevatorHeight) {
    // elevatorStageOne.setLength(elevatorHeight * (2 / 3));
    // elevatorStageTwo.setLength(elevatorHeight);

    Pose3d elevatorOnePose = new Pose3d(0, 0, elevatorHeight * .8333, new Rotation3d(0, 0, 0));
    Pose3d elevatorTwoPose =
        new Pose3d(
            0, 0, (elevatorHeight * .8333) + (elevatorHeight * .9193), new Rotation3d(0, 0, 0));
    Logger.recordOutput("ElevatorMechanism3d/ElevatorOne", elevatorOnePose);
    Logger.recordOutput("ElevatorMechanism3d/ElevatorTwo", elevatorTwoPose);
  }
}
