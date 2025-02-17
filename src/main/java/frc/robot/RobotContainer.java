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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants.ReefConstants;
import frc.robot.Constants.SuperState;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSpark;
import frc.robot.subsystems.intake.SensorIO;
import frc.robot.subsystems.intake.SensorIOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSpark;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristIOSpark;
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
  private final Elevator elevator;
  private final Intake intake;
  private final Pivot pivot;
  private final Wrist wrist;
  private final Climber climber;

  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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

        climber = Climber.initialize(new ClimberIOSpark());

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

        climber = Climber.initialize(new ClimberIOSim());

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

        climber = new Climber(new ClimberIO() {});

        break;
    }

    superstructure = new Superstructure(climber, elevator, intake, pivot, wrist);

    // Set up named commands
    NamedCommands.registerCommand("STOW", superstructure.setSuperStateCommand(SuperState.STOW));

    NamedCommands.registerCommand(
        "LEFT GROUND INTAKE", superstructure.setSuperStateCommand(SuperState.GROUNDINTAKE, true));
    NamedCommands.registerCommand(
        "LEFT L1", superstructure.setSuperStateCommand(SuperState.REEFL1, false));
    NamedCommands.registerCommand(
        "LEFT L4", superstructure.setSuperStateCommand(SuperState.REEFL4, false));

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

    climber.setDefaultCommand(
        climber.goToStateCommand(climber::getFlipState, climber::getWinchState));

    wrist.setDefaultCommand(wrist.goToStateCommand(wrist::getWristState));

    // Driver Controls
    driverController
        .rightTrigger(0.1)
        .onTrue(superstructure.setSuperStateCommand(SuperState.GROUNDINTAKE))
        .onFalse(superstructure.setSuperStateCommand(SuperState.STOW));

    driverController
        .rightBumper()
        .onTrue(superstructure.setSuperStateCommand(superstructure::getTargetLevel))
        .onFalse(superstructure.setSuperStateCommand(SuperState.STOW));

    // Operator Controls
    operatorController.a().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL1));
    operatorController.b().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL2));
    operatorController.x().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL3));
    operatorController.y().onTrue(superstructure.setTargetLevelCommand(SuperState.REEFL4));

    // Manual arm direction
    operatorController
        .povRight()
        .onTrue(pivot.setDirectionBooleanCommand(false))
        .onFalse(pivot.releaseManualDirectionCommand());

    operatorController
        .povLeft()
        .onTrue(pivot.setDirectionBooleanCommand(true))
        .onFalse(pivot.releaseManualDirectionCommand());

    // Reef Selector
    operatorController
        .leftBumper()
        .whileTrue(
            superstructure.setTargetReefPoseCommand(
                true, operatorController::getLeftX, operatorController::getLeftY));
    operatorController
        .rightBumper()
        .whileTrue(
            superstructure.setTargetReefPoseCommand(
                false, operatorController::getLeftX, operatorController::getLeftY));

    // // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro to 0° when B button is pressed
    // driverController
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

  }

  public Command onTeleopInitCommand() {
    return new ParallelCommandGroup(
        pivot.releaseManualDirectionCommand(),
        superstructure.setSuperStateCommand(SuperState.STOW));
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
