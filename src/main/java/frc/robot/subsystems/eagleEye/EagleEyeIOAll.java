package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class EagleEyeIOAll implements EagleEyeIO {

  private String[] gamePieceNames;
  private GamePiece[] gamePieces;
  private String[] gamePiecesStrings;

  public EagleEyeIOAll() {}

  /**
   * Gets a network table entry from LimeLight.
   *
   * @param entryKey The key of the entry to get.
   * @return The network table entry.
   */
  private NetworkTableEntry getTableEntry(String entryKey) {
    return NetworkTableInstance.getDefault().getTable("GamePieces").getEntry(entryKey);
  }

  public void updateInputs(EagleEyeIOInputsAutoLogged inputs) {
    List<GamePiece> gamePiecesList = new ArrayList<>();
    gamePieceNames = getTableEntry("class_names").getStringArray(new String[0]);

    try {

      for (String gamePieceName : gamePieceNames) {
        double[] gamePieceYaws =
            getTableEntry(gamePieceName + "_yaw_angles").getDoubleArray(new double[0]);
        String[] gamePieceGlobalPositions =
            getTableEntry(gamePieceName + "_global_positions").getStringArray(new String[0]);
        String[] gamePieceLocalPositions =
            getTableEntry(gamePieceName + "_local_positions").getStringArray(new String[0]);

        for (int i = 0; i < gamePieceYaws.length; i++) {
          gamePiecesList.add(
              new GamePiece(
                  gamePieceName,
                  poseFromString(gamePieceGlobalPositions[i]),
                  poseFromString(gamePieceLocalPositions[i]),
                  gamePieceYaws[i]));
        }
      }

      gamePieces = gamePiecesList.toArray(new GamePiece[0]);

      if (gamePieces.length > 0) {
        Pose2d[] globalPositions = new Pose2d[gamePieces.length];
        for (int i = 0; i < gamePieces.length; i++) {
          globalPositions[i] = gamePieces[i].getGamePieceGlobalPosition();
        }
        inputs.globalPositions = globalPositions;
      }

      gamePiecesStrings = new String[gamePieces.length];
      for (int i = 0; i < gamePieces.length; i++) {
        gamePiecesStrings[i] = gamePieces[i].toString();
      }

      inputs.gamePieceNames = gamePieceNames;
      inputs.gamePieces = gamePiecesStrings;
    } catch (Exception e) {
      Logger.recordOutput("EagleEyeError:", e.getMessage());
    }
  }

  private Pose2d poseFromString(String poseString) {
    String[] poseArray = poseString.split(",");
    return new Pose2d(
        Double.parseDouble(poseArray[0]),
        Double.parseDouble(poseArray[1]),
        Rotation2d.fromDegrees(0));
  }
}
