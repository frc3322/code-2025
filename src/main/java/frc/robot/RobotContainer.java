// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.SuperState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.drive.Simpledrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.SensorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants.PivotStates;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSpark;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Simpledrive simpledrive;

  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;
  private final Climber climber;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final APACButtonBox apacButtonBox = new APACButtonBox();

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private Trigger autoScoreTrigger;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(0),
                new ModuleIOSpark(1),
                new ModuleIOSpark(2),
                new ModuleIOSpark(3));

        elevator = Elevator.initialize(new ElevatorIOSpark());

        intake = Intake.initialize(new IntakeIOSpark(), new SensorIO() {});

        pivot = Pivot.initialize(new PivotIOSpark(), drive::getPose);

        wrist = Wrist.initialize(new WristIOSpark(), drive::getPose, this::getTargetReefPose);

        climber = new Climber();

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());

        elevator = Elevator.initialize(new ElevatorIOSim());

        intake = Intake.initialize(new IntakeIOSim(), new SensorIOSim());

        pivot = Pivot.initialize(new PivotIOSim(), drive::getPose);

        wrist = Wrist.initialize(new WristIOSim(), drive::getPose, this::getTargetReefPose);

        climber = new Climber();

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake = new Intake(new IntakeIO() {}, new SensorIO() {});

        pivot = new Pivot(new PivotIO() {}, drive::getPose);

        wrist = Wrist.initialize(new WristIO() {}, drive::getPose, this::getTargetReefPose);

        climber = new Climber();

        break;
    }
    simpledrive = new Simpledrive(drive);

    superstructure =
        new Superstructure(climber, elevator, intake, pivot, wrist, drive, simpledrive);

    // Set up named commands
    NamedCommands.registerCommand("STOW", superstructure.setSuperStateCommand(SuperState.STOW));
    NamedCommands.registerCommand(
        "L4 SCORE", superstructure.l4ScoreCommand().andThen(new WaitCommand(.45)));

    NamedCommands.registerCommand(
        "L1 SCORE", superstructure.l1ScoreCommand().andThen(new WaitCommand(.25)));

    NamedCommands.registerCommand(
        "END COMMAND",
        new SequentialCommandGroup(
            intake.setIntakeStateCommand(IntakeStates.OUTTAKE),
            pivot.setStateCommand(PivotStates.STOW)));

    NamedCommands.registerCommand(
        "LEFT GROUND INTAKE", superstructure.setSuperStateCommand(SuperState.GROUNDINTAKE));

    NamedCommands.registerCommand(
        "SOURCE INTAKE", superstructure.setSuperStateCommand(SuperState.SOURCEINTAKE));

    NamedCommands.registerCommand(
        "LEFT L1", superstructure.setSuperStateCommand(SuperState.REEFL1));

    NamedCommands.registerCommand("L4", superstructure.setTargetLevelCommand(SuperState.REEFL4));

    NamedCommands.registerCommand("L3", superstructure.setTargetLevelCommand(SuperState.REEFL3));

    NamedCommands.registerCommand("AUTON L4", superstructure.autonL4Sequence());

    NamedCommands.registerCommand(
        "AUTO ALIGN L4",
        simpledrive
            .autoDriveToReef(superstructure::getTargetReefPose, () -> SuperState.REEFL4)
            .until(
                () ->
                    Constants.FieldConstants.PoseMethods.atPose(
                        drive.getPose(), drive.getTargetReefPose(), .05, 4))
            .andThen(DriveCommands.stopCommand(drive)));

    NamedCommands.registerCommand(
        "AUTO ALIGN L3",
        simpledrive
            .autoDriveToReef(superstructure::getTargetReefPose, () -> SuperState.REEFL3)
            .until(
                () ->
                    Constants.FieldConstants.PoseMethods.atPose(
                        drive.getPose(), drive.getTargetReefPose(), .1, 5))
            .andThen(new WaitCommand(1)));

    NamedCommands.registerCommand(
        "HIGH ALGAE", superstructure.setSuperStateCommand(SuperState.ALGAEPLUCKHIGH));
    NamedCommands.registerCommand(
        "LOW ALGAE", superstructure.setSuperStateCommand(SuperState.ALGAEPLUCKLOW));

    NamedCommands.registerCommand(
        "ALGAE STOW", superstructure.setSuperStateCommand(SuperState.ALGAESTOW));

    NamedCommands.registerCommand("BARGE", superstructure.setSuperStateCommand(SuperState.BARGE));
    NamedCommands.registerCommand("BARGE SCORE", superstructure.bargeScoreCommand());

    NamedCommands.registerCommand(
        "R1", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition1));
    NamedCommands.registerCommand(
        "R2", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition2));
    NamedCommands.registerCommand(
        "R3", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition3));
    NamedCommands.registerCommand(
        "R4", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition4));
    NamedCommands.registerCommand(
        "R5", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition5));
    NamedCommands.registerCommand(
        "R6", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition6));
    NamedCommands.registerCommand(
        "R7", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition7));
    NamedCommands.registerCommand(
        "R8", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition8));
    NamedCommands.registerCommand(
        "R9", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition9));
    NamedCommands.registerCommand(
        "R10", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition10));
    NamedCommands.registerCommand(
        "R11", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition11));
    NamedCommands.registerCommand(
        "R12", superstructure.setTargetReefPoseCommand(ReefConstants.coralPosition12));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    autoScoreTrigger =
        new Trigger(
                () ->
                    Constants.FieldConstants.PoseMethods.atPose(
                        drive.getPose(), getTargetReefPose(), .02, 20))
            .and(elevator::isAtGoal);

    // Inter subsystem binfings

    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    elevator.setDefaultCommand(elevator.goToStateCommand(elevator::getElevatorState));

    pivot.setDefaultCommand(pivot.goToStateCommand(pivot::getPivotState));

    intake.setDefaultCommand(intake.goToStateCommand(intake::getState));

    climber.setDefaultCommand(climber.climberGoToPositionCommand());

    wrist.setDefaultCommand(
        wrist.goToStateCommand(wrist::getWristState, pivot::getDirectionReversed));

    // Driver Controls

    driverController
        .rightTrigger(0.1)
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.GROUNDINTAKE)))
        .onFalse(superstructure.setSuperStateCommand(SuperState.STOW));

    driverController
        .leftTrigger(0.1)
        .onTrue(
            pivot
                .setFlippedCommand(true)
                .andThen(superstructure.setSuperStateCommand(SuperState.GROUNDINTAKE)))
        .onFalse(superstructure.setSuperStateCommand(SuperState.STOW));

    driverController.a().onTrue(superstructure.setSuperStateCommand(SuperState.SOURCEINTAKE));
    driverController.x().onTrue(superstructure.setSuperStateCommand(SuperState.STOW));

    // L1 thru L4 bindings - all same button
    driverController
        .rightBumper()
        .onTrue(superstructure.setSuperStateCommand(SuperState.REEFL1))
        .onFalse(
            superstructure
                .l1ScoreCommand()
                .andThen(new WaitCommand(.5))
                .andThen(superstructure.setSuperStateCommand(SuperState.STOW)));

    driverController
        .leftBumper()
        .and(() -> superstructure.getTargetLevel() == SuperState.REEFL4)
        .and(superstructure.getSemiAutoDisabledTrigger())
        .onTrue(superstructure.l4ScoreCommand())
        .onFalse(intake.setIntakeStateCommand(IntakeStates.OFF));

    driverController
        .leftBumper()
        .and(
            () ->
                superstructure.getTargetLevel() == SuperState.REEFL3
                    || superstructure.getTargetLevel() == SuperState.REEFL2)
        .and(superstructure.getSemiAutoDisabledTrigger())
        .onTrue(superstructure.l2and3ScoreCommand());

    driverController
        .leftBumper()
        .and(() -> superstructure.getTargetLevel() == SuperState.REEFL1)
        .and(superstructure.getSemiAutoDisabledTrigger())
        .onTrue(superstructure.l1ScoreCommand());

    driverController
        .leftBumper()
        .and(superstructure.getSemiAutoEnabledTrigger())
        .whileTrue(superstructure.semiAutoScoreCommand());

    // driverController
    //     .y()
    //     .onTrue(superstructure.setSuperStateCommand(SuperState.SOURCEINTAKE))
    //     .whileTrue(
    //         superstructure.rotateToSourceCommand(
    //             () -> -driverController.getLeftY(), () -> -driverController.getLeftX()));

    driverController.povUp().onTrue(intake.setIntakeStateCommand(IntakeStates.REVERSE));

    driverController
        .povDown()
        .onTrue(elevator.lowerElevatorCommand())
        .onFalse(elevator.stopElevatorCommand().andThen(elevator.zeroElevatorCommand()));

    apacButtonBox
        .processorRight()
        .onTrue(superstructure.setSuperStateCommand(SuperState.PROCESSOR))
        .onFalse(intake.setIntakeStateCommand(IntakeStates.REVERSE));

    // Operator Controls
    // operatorController
    //     .leftTrigger(0.1)
    //     .whileTrue(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> -driverController.getLeftY() / 1.5,
    //             () -> -driverController.getLeftX() / 1.5,
    //             () -> -driverController.getRightX()));

    // operatorController.a().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL1));
    // operatorController.b().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL2));
    // operatorController.x().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL3));
    // operatorController.y().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL4));

    // // Reef Selector
    // operatorController
    //     .leftBumper()
    //     .whileTrue(
    //         superstructure.setTargetReefPoseCommand(
    //             true, operatorController::getLeftX, operatorController::getLeftY));
    // operatorController
    //     .rightBumper()
    //     .whileTrue(
    //         superstructure.setTargetReefPoseCommand(
    //             false, operatorController::getLeftX, operatorController::getLeftY));

    operatorController
        .povUp()
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.CLIMB)));

    operatorController
        .povDown()
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.CLIMBED)));

    // APAC (Button Box) controls
    apacButtonBox
        .reefOneTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition1));

    apacButtonBox
        .reefTwoTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition2));

    apacButtonBox
        .reefThreeTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition3));

    apacButtonBox
        .reefFourTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition4));

    apacButtonBox
        .reefFiveTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition5));

    apacButtonBox
        .reefSixTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition6));

    apacButtonBox
        .reefSevenTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition7));

    apacButtonBox
        .reefEightTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition8));

    apacButtonBox
        .reefNineTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition9));

    apacButtonBox
        .reefTenTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition10));

    apacButtonBox
        .reefElevenTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition11));

    apacButtonBox
        .reefTwelveTrigger()
        .onTrue(
            superstructure.setTargetReefPoseCommand(
                Constants.FieldConstants.ReefConstants.coralPosition12));

    apacButtonBox.levelOneTrigger().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL1));

    apacButtonBox.levelTwoTrigger().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL2));

    apacButtonBox
        .levelThreeTrigger()
        .onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL3));

    apacButtonBox
        .levelFourTrigger()
        .onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL4));

    apacButtonBox
        .manualTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -driverController.getLeftY() / 1.5,
                () -> -driverController.getLeftX() / 1.5,
                () -> -driverController.getRightX()));
    // .onFalse(superstructure.scoreCommand());

    apacButtonBox
        .climberUpTrigger()
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.CLIMB)));

    apacButtonBox
        .climberDownTrigger()
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.CLIMBED)));

    apacButtonBox
        .algaeHighTrigger()
        .onTrue(superstructure.setSuperStateCommand(SuperState.ALGAEPLUCKHIGH));

    apacButtonBox
        .algaeLowTrigger()
        .onTrue(superstructure.setSuperStateCommand(SuperState.ALGAEPLUCKLOW));

    apacButtonBox
        .algaeGroundLeftTrigger()
        .onTrue(
            pivot
                .setFlippedCommand(true)
                .andThen(superstructure.setSuperStateCommand(SuperState.ALGAEGROUNDINTAKE)));

    apacButtonBox
        .algaeGroundRightTrigger()
        .onTrue(
            pivot
                .setFlippedCommand(false)
                .andThen(superstructure.setSuperStateCommand(SuperState.ALGAEGROUNDINTAKE)));

    apacButtonBox
        .algaeHoldTrigger()
        .whileTrue(superstructure.setSuperStateCommand(SuperState.ALGAESTOW));

    apacButtonBox
        .bargeTrigger()
        .onTrue(superstructure.setSuperStateCommand(SuperState.BARGE))
        .onFalse(superstructure.bargeScoreCommand());

    apacButtonBox
        .manualTrigger()
        .and(
            () ->
                superstructure.getTargetLevel() == SuperState.REEFL3
                    || superstructure.getTargetLevel() == SuperState.REEFL2)
        .onFalse(superstructure.l2and3ScoreCommand());

    apacButtonBox
        .manualTrigger()
        .and(() -> superstructure.getTargetLevel() == SuperState.REEFL4)
        .onFalse(superstructure.l4ScoreCommand());

    apacButtonBox
        .manualTrigger()
        .and(() -> superstructure.getTargetLevel() == SuperState.REEFL1)
        .onFalse(superstructure.l1ScoreCommand());

    // // Lock to 0° when A button is held
    // driverController
    // .a()
    // .whileTrue(
    // DriveCommands.joystickDriveAtAngle(
    // drive,
    // () -> -driverController.getLeftY(),
    // () -> -driverController.getLeftX(),
    // () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
  }

  public Command onTeleopInitCommand() {
    return new ParallelCommandGroup(superstructure.setSuperStateCommand(SuperState.STOW));
  }

  public Command zeroSwervesCommand() {
    return drive.zeroSwervesCommand();
  }

  public Command manualAdjustDrivePoseCommand(Supplier<Pose2d> poseSupplier) {
    return drive.setTargetPoseCommand(
        simpledrive.getTargetReefPose(poseSupplier, superstructure::getTargetLevel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Pose2d getTargetReefPose() {
    return superstructure.getTargetReefPose();
  }
}
