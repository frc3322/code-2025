package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GamePiece {
  private String gamePieceClass;
  private Pose2d gamePieceLocalPosition;
  private Pose2d gamePieceGlobalPosition;
  private double gamePieceYaw;

  public GamePiece(
      String gamePieceName,
      Pose2d gamePieceLocalPosition,
      Pose2d gamePieceGlobalPosition,
      double gamePieceYaw) {
    this.gamePieceClass = gamePieceName;
    this.gamePieceLocalPosition = gamePieceLocalPosition;
    this.gamePieceGlobalPosition = gamePieceGlobalPosition;
    this.gamePieceYaw = gamePieceYaw;
  }

  public GamePiece(String gamePieceData) {
    String[] gamePieceDataArray = gamePieceData.split(",");
    this.gamePieceClass = gamePieceDataArray[0];
    this.gamePieceLocalPosition =
        new Pose2d(
            Double.parseDouble(gamePieceDataArray[1]),
            Double.parseDouble(gamePieceDataArray[2]),
            new Rotation2d(0));
    this.gamePieceGlobalPosition =
        new Pose2d(
            Double.parseDouble(gamePieceDataArray[3]),
            Double.parseDouble(gamePieceDataArray[4]),
            new Rotation2d(0));
    this.gamePieceYaw = Double.parseDouble(gamePieceDataArray[5]);
  }

  public String getGamePieceClass() {
    return gamePieceClass;
  }

  public Pose2d getGamePieceGlobalPosition() {
    return gamePieceGlobalPosition;
  }

  public Pose2d getGamePieceLocalPosition() {
    return gamePieceLocalPosition;
  }

  public double getGamePieceYaw() {
    return gamePieceYaw;
  }

  @Override
  public String toString() {
    return gamePieceClass
        + ","
        + gamePieceLocalPosition.getX()
        + ","
        + gamePieceLocalPosition.getY()
        + ","
        + gamePieceGlobalPosition.getX()
        + ","
        + gamePieceGlobalPosition.getY()
        + ","
        + gamePieceYaw;
  }
}
