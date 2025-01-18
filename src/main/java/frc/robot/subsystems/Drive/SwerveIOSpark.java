package frc.robot.subsystems.Drive;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveIOSpark implements SwerveIO{
    
    // https://docs.yagsl.com/configuring-yagsl/code-setup
    private File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    private SwerveDrive swerveDrive;

    public SwerveIOSpark() {
        /* 
        * !_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        * TELEMETRY - SET THIS TO LOW FOR DEBUGGING, AND HIGH FOR FASTER CODE
        * !_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!_!
        */
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;
    
        // Swerve init - https://docs.yagsl.com/configuring-yagsl/code-setup 
        try{
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(DriveConstants.maximumModuleSpeed);
        }
        catch (IOException e) {
            DriverStation.reportError("Swerve config not found: " + e.toString(), true);
        }

        //https://github.com/BroncBotz3481/YAGSL-Example/blob/main/src/main/java/frc/robot/subsystems/swervedrive/SwerveSubsystem.java
        //Heading correction should only be used while controlling the robot via angle.
        swerveDrive.setHeadingCorrection(false);
        //!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        swerveDrive.setCosineCompensator(false);
        //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1); 
        // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    }

    // Most important method here
    @Override
    public void drive(double translationX, double translationY, double rotation, boolean isFieldRelative, boolean isOpenLoop){
        swerveDrive.drive(new Translation2d(translationX * swerveDrive.getMaximumChassisVelocity(),
            translationY * swerveDrive.getMaximumChassisVelocity()),
            rotation * swerveDrive.getMaximumChassisAngularVelocity(),
            isFieldRelative,
            isOpenLoop // Ususally false
        );
    }	
    
    @Override
	public void setLock() {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setMotorBrake(true);
			module.setAngle(45);
		}
	}

    // Basically a fancy periodic
	@Override
	public void updateInputs(SwerveIOInputs inputs) {
		inputs.pose = swerveDrive.getPose();
		inputs.yaw = swerveDrive.getYaw();
		inputs.odometryHeading = swerveDrive.getOdometryHeading();
	}

    // Odometry and pose methods

    @Override
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

    @Override
	public void resetOdometry(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	@Override
	public void updateOdometry() {
		swerveDrive.updateOdometry();
	}

    // Chassis speeds methods

	@Override
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}

    // Heading and yaw methods

	@Override
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	@Override
	public Rotation2d getYaw() {
		return swerveDrive.getYaw();
	}

    // Configuration getters

	@Override
	public double getConfigurationRadius() {
		return swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters();
	}

	@Override
	public double getMaximumChassisVelocity() {
		return swerveDrive.getMaximumChassisVelocity();
	}

	@Override
	public double getMaximumChassisAngularVelocity() {
		return swerveDrive.getMaximumChassisAngularVelocity();
	}

    // Vision methods

    //put vision methods here. make sure to add any methods you make here to the SwerveIO interface as defaults.
    //swerveDrive.addVisionMeasurement(pose, timestamp);


    //Misc methods
    
	@Override
	public void setDriveMotorCurrentLimit(int currentLimit) {
		for (SwerveModule module : swerveDrive.getModules()) {
			module.getDriveMotor().setCurrentLimit(currentLimit);
		}
	}

	@Override
	public void setTurnMotorCurrentLimit(int currentLimit) {
		for (SwerveModule module : swerveDrive.getModules()) {
			module.getAngleMotor().setCurrentLimit(currentLimit);
		}
	}

	@Override
	public void setBrakeMode(boolean isBrakeMode) {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setMotorBrake(isBrakeMode);
		}
	}

	@Override
	public void setZero() {
		for (SwerveModule module : swerveDrive.swerveDriveConfiguration.modules) {
			module.setAngle(0.0);
		}
	}
}
