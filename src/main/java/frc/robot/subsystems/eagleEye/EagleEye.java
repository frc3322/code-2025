// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.eagleEye;

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
  public void set_camera(int camera) {
    eagleEyeIO.set_camera(camera);
  }
}
