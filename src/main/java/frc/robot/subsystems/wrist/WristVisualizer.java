package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

public class WristVisualizer {
  Mechanism2d chassisMech;
  MechanismRoot2d wristRoot;
  MechanismLigament2d wristStageOne;
  MechanismRoot2d wristStageOneRoot;
  MechanismLigament2d wristStageTwo;

  public WristVisualizer() {
    // chassisMech = new Mechanism2d(Units.inchesToMeters(28), Units.inchesToMeters(28));
    // wristRoot = chassisMech.getRoot("base", Units.inchesToMeters(19), 0);
    // wristStageOne =
    //     wristRoot.append(
    //         new MechanismLigament2d("wrist stage one", Units.inchesToMeters(40.25), 90));
    // wristStageTwo =
    //     wristRoot.append(
    //         new MechanismLigament2d("wrist stage two", Units.inchesToMeters(6), 90));
  }

  public void update(double wristAngle, double pivotAngle, double elevatorHeight) {
    // wristStageOne.setLength(wristHeight * (2 / 3));
    // wristStageTwo.setLength(wristHeight);

    Pose3d wristPose =
        new Pose3d(
            0.005,
            -0.0023,
            (elevatorHeight * .8333) + (elevatorHeight * .9193) + .183,
            new Rotation3d(pivotAngle, 0, wristAngle));
    Logger.recordOutput("wristMechanism3d/wrist", wristPose);
  }
}
