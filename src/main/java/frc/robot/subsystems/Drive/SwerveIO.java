package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface SwerveIO {
    
	@AutoLog
	public static class SwerveIOInputs {
		public Pose2d pose = new Pose2d();
		public Rotation2d yaw = new Rotation2d();
		public Rotation2d odometryHeading = new Rotation2d();
	}

    /**
	 * Updates the inputs with the current values.
	 *
	 * @param inputs The inputs to update.
	 */
	public default void updateInputs(SwerveIOInputs inputs) {};

	  /**
   * Method to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param rotation   Angular velocity setpoint
   */
    public default void drive(double translationX, double translationY, double rotation, boolean isFieldRelative, boolean isOpenLoop) {}

	/**
	 * Get the maximum swerve velocity.
	 *
	 * @return The maximum velocity
	 */
    public default double getMaximumChassisVelocity() {return 0;}

    /**
	 * Retrieves the maximum swerve angular velocity.
	 *
	 * @return The maximum angular velocity
	 */
	public default double getMaximumChassisAngularVelocity() {return 0;}

	//To-do: ELLIOT - CHANGE TO LIMELIGHTS
    //void updateEstimations(SUB_Vision vision);

    /** Updates the odometry for the swerve drive. */
	public default void updateOdometry() {}

    /**
	 * Resets the odometry to the given pose.
	 *
	 * @param pose The pose to reset the odometry to
	 */
	public default void resetOdometry(Pose2d pose) {}

	/**
	 * Retrieves the current pose of the swerve drive.
	 *
	 * @return The pose of the swerve drive
	 */
    public default Pose2d getPose() {return new Pose2d();}

	/**
	 * Retrieves the current heading of the robot from its pose.
	 *
	 * @return The current heading in a Rotation2d
	 */
    public default Rotation2d getHeading() {return new Rotation2d();}

    /**
	 * Gets the robot's velocity.
	 *
	 * @return The current swerve drive velocity in chassis speeds
	 */
	public default ChassisSpeeds getRobotVelocity() {return new ChassisSpeeds();}

    /**
	 * Sets the chassis speeds for the swerve drive.
	 *
	 * @param chassisSpeeds The desired chassis speeds in MPS
	 */
	public default void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {}

	// We dont have this struct / record. Do we need it?
    //PIDConstants getHeadingPID();

	/**
	 * Retrieves the configuration radius of the swerve drive.
	 *
	 * @return the configuration radius in meters
	 */
    public default double getConfigurationRadius() {return 0;}

    /**
	 * Sets the brake mode for all swerve modules in the swerve drive configuration.
	 *
	 * @param isBrakeMode True to enable brake mode, false to disable
	 */
	public default void setBrakeMode(boolean isBrakeMode) {}

	/** Sets the angle of all swerve modules to 0.0. */
    public default void setZero() {}

    /** Sets the lock for the swerve drive by setting the motor brake and angle */
	public default void setLock() {}

    /**
	 * Sets the current limit for each drive drive motor in the swerve.
	 *
	 * @param currentLimit The new current limit to be set
	 */
	public default void setDriveMotorCurrentLimit(int currentLimit) {}

    /**
	 * Sets the current limit for each turn motor in the swerve.
	 *
	 * @param currentLimit The new current limit to be set
	 */
	public default void setTurnMotorCurrentLimit(int currentLimit) {}

	public default Rotation2d getYaw() {return new Rotation2d();}
}
