package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GamePiece {
  private String gamePieceClass;
  private Pose2d gamePieceLocalPosition;
  private Pose2d gamePieceGlobalPosition;
  private double gamePieceYaw;
  private double gamePieceDistance;
  private double ratio;

  public GamePiece(
      String gamePieceName,
      Pose2d gamePieceLocalPosition,
      Pose2d gamePieceGlobalPosition,
      double gamePieceYaw,
      double gamePieceDistance,
      double ratio) {
    this.gamePieceClass = gamePieceName;
    this.gamePieceLocalPosition = gamePieceLocalPosition;
    this.gamePieceGlobalPosition = gamePieceGlobalPosition;
    this.gamePieceYaw = gamePieceYaw;
    this.gamePieceDistance = gamePieceDistance;
    this.ratio = ratio;
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
    this.gamePieceDistance = Double.parseDouble(gamePieceDataArray[6]);
    this.ratio = Double.parseDouble(gamePieceDataArray[7]);
  }

  /**
   * Retrieves the class of the game piece.
   *
   * @return A string representing the class of the game piece.
   */
  public String getGamePieceClass() {
    return gamePieceClass;
  }

  /**
   * Retrieves the global position of the game piece.
   *
   * @return The global position of the game piece as a Pose2d object.
   */
  public Pose2d getGamePieceGlobalPosition() {
    return gamePieceGlobalPosition;
  }

  /**
   * Retrieves the local position of the game piece to the robot.
   *
   * @return The local position of the game piece as a Pose2d object (0 rotation).
   */
  public Pose2d getGamePieceLocalPosition() {
    return gamePieceLocalPosition;
  }

  /**
   * Retrieves the yaw angle to the game piece.
   *
   * @return the yaw angle of the game piece as a double.
   */
  public double getGamePieceYaw() {
    return gamePieceYaw;
  }

  /**
   * Retrieves the distance to the game piece.
   *
   * @return the distance to the game piece as a double.
   */
  public double getGamePieceDistance() {
    return gamePieceDistance;
  }

  /**
   * Returns the width to height ratio of the game piece.
   *
   * @return the ratio as a double.
   */
  public double getRatio() {
    return ratio;
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
        + gamePieceYaw
        + ","
        + gamePieceDistance
        + ","
        + ratio;
  }
}
