// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.eagleEye;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EagleEye extends SubsystemBase {
  private static EagleEye instance;

  private final EagleEyeIOInputsAutoLogged inputs = new EagleEyeIOInputsAutoLogged();

  private EagleEyeVisualizer visualizer = new EagleEyeVisualizer();

  private GamePiece[] gamePieces;

  private EagleEyeIO eagleEyeIO;

  public static EagleEye initialize(EagleEyeIO eagleEyeIO) {
    if (instance == null) {
      instance = new EagleEye(eagleEyeIO);
    }
    return instance;
  }

  public static EagleEye getInstance() {
    return instance;
  }

  /** Creates a new EagleEye. */
  public EagleEye(EagleEyeIO eagleEyeIO) {
    this.eagleEyeIO = eagleEyeIO;
  }

  public void updateInputs() {
    eagleEyeIO.updateInputs(inputs);

    Logger.processInputs("EagleEye", inputs);

    gamePieces = new GamePiece[inputs.gamePieces.length];
    
    // Iterate through each element in the inputs.gamePieces array (string array) and convert it to a GamePiece object
    for (int i = 0; i < inputs.gamePieces.length; i++) {
      gamePieces[i] = new GamePiece(inputs.gamePieces[i]);
    }
  }

  @Override
  public void periodic() {
    updateInputs();

    visualizer.update(inputs.globalPositions);
  }

  /**
   * Sets the camera to the specified camera index.
   *
   * @param camera the index of the camera to be set
   */
  public void setCamera(int camera) {
    eagleEyeIO.setCamera(camera);
  }

  /**
   * Retrieves the global position of the closest game piece.
   *
   * @return A Pose2d object representing the global position of the closest game piece.
   */
  public Pose2d getClosestGamePiecePosition() {
    return gamePieces[0].getGamePieceGlobalPosition();
  }

  /**
   * Retrieves the yaw (rotation in degrees) of the closest game piece.
   *
   * @return the yaw of the closest game piece in degrees.
   */
  public double getClosestGamePieceYaw() {
    return gamePieces[0].getGamePieceGlobalPosition().getRotation().getDegrees();
  }

  /**
   * Returns the first GamePiece from the list of game pieces that has a width to height ratio
   * greater than the specified ratio threshold.
   *
   * @param ratioThreshold the threshold ratio to compare against
   * @return the first (closest) GamePiece with a ratio greater than the ratioThreshold,
   *         or null if no such GamePiece is found
   */
  public GamePiece getOptomalGamePiece(double ratioThreshold) {
    for (GamePiece gamePiece : gamePieces) {
      if (gamePiece.getRatio() > ratioThreshold) {
        return gamePiece;
      }
    }
    return null;
  }

  /**
   * Retrieves the optimal game piece position based on the given width to height ratio threshold.
   *
   * @param ratioThreshold the threshold width to height ratio to determine the optimal game piece.
   * @return the global position of the optimal game piece as a Pose2d object, or null if no game piece meets the criteria.
   */
  public Pose2d getOptomalGamePiecePosition(double ratioThreshold) {
    GamePiece gamePiece = getOptomalGamePiece(ratioThreshold);
    if (gamePiece != null) {
      return gamePiece.getGamePieceGlobalPosition();
    }
    return null;
  }

  /**
   * Retrieves the optimal game piece yaw based on the given width to height ratio threshold.
   *
   * @param ratioThreshold the threshold width to height ratio to determine the optimal game piece.
   * @return the yaw of the optimal game piece in degrees, or !!9999!! if no game piece meets the criteria.
   */
  public double getOptomalGamePieceYaw(double ratioThreshold) {
    GamePiece gamePiece = getOptomalGamePiece(ratioThreshold);
    if (gamePiece != null) {
      return gamePiece.getGamePieceGlobalPosition().getRotation().getDegrees();
    }
    return 9999;
  }
}
