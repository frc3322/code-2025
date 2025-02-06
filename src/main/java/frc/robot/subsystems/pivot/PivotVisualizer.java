package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import org.littletonrobotics.junction.Logger;

public class PivotVisualizer {
  Mechanism2d chassisMech;
  MechanismRoot2d pivotRoot;
  MechanismLigament2d pivotStageOne;
  MechanismRoot2d pivotStageOneRoot;
  MechanismLigament2d pivotStageTwo;

  public PivotVisualizer() {
    // chassisMech = new Mechanism2d(Units.inchesToMeters(28), Units.inchesToMeters(28));
    // pivotRoot = chassisMech.getRoot("base", Units.inchesToMeters(19), 0);
    // pivotStageOne =
    //     pivotRoot.append(
    //         new MechanismLigament2d("pivot stage one", Units.inchesToMeters(40.25), 90));
    // pivotStageTwo =
    //     pivotRoot.append(
    //         new MechanismLigament2d("pivot stage two", Units.inchesToMeters(6), 90));
  }

  public void update(double pivotAngle, double elevatorHeight) {
    // pivotStageOne.setLength(pivotHeight * (2 / 3));
    // pivotStageTwo.setLength(pivotHeight);

    Pose3d pivotPose =
        new Pose3d(
            0, 0, (elevatorHeight * .8333) + (elevatorHeight * .9193), new Rotation3d(pivotAngle, 0, 0));
    Logger.recordOutput("pivotMechanism3d/pivot", pivotPose);
  }
}
