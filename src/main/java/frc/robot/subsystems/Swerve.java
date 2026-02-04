package frc.robot.subsystems;

import java.io.File;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    /**
     * Swerve subsystem for the robot.
     * 
     * @author Max Clemetson, Jake Xie, Justin Baratta & Siddhartha Hiremath
     * @since 2025-10
     */
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController headingController = new PIDController(1, 0, 0);

    public Swerve() {
        /**
         * Swerve constructor for the robot.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        try {
            headingController.enableContinuousInput(-Math.PI, Math.PI);
            double maximumSpeed = Units.feetToMeters(4.5);

            SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

            // On the RIO, deploy files live here:
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);

            swerveDrive.setCosineCompensator(true);
            swerveDrive.setHeadingCorrection(true);

            swerveDrive.resetOdometry(new Pose2d(0, 0, Rotation2d.kZero));

            System.out.println("Swerve is initialized!");
            initialized = true;
        } catch (Exception e) {
            System.out.println("Failed to load swerve config.");
            DriverStation.reportError("Failed to load swerve configuration from /deploy/swerve: " + e.getMessage(),
                    e.getStackTrace());
        }
        swerveDrive.resetOdometry(new Pose2d());
    }

    public void periodic() {
        /**
         * Periodic method for the swerve subsystem.
         * 
         * Updates the field position of the robot.
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        if (!initialized)
            return;

        field.setRobotPose(swerveDrive.getPose());
        SmartDashboard.putData(this);
    }

    public Pose2d resetOdometry(Pose2d pose) {
        /**
         * Resets the odometry of the robot.
         * 
         * @args Pose2d pose
         * @author Max Clementson & Jake Xie
         * @since 2025-10
         * @return Pose2d
         */
        if (!initialized)
            return new Pose2d();
        swerveDrive.resetOdometry(pose);
        return swerveDrive.getPose();
    }

    // Elastic Stuff

    public Field2d getField() {
        /**
         * Gets the field position of the robot.
         * 
         * @args None
         * @author Max Clementson & Jake Xie
         * @since 2025-10
         * @return Field2d
         */
        return field;
    }

    public Pose2d getPose() {
        /**
         * Gets the pose of the robot.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return Pose2d
         */
        if (!initialized)
            return new Pose2d();
            
        return swerveDrive.getPose();
    }

    public void followTrajectory(SwerveSample sample) {
        /**
         * Follows a trajectory for the robot using elastic.
         * 
         * @args SwerveSample sample
         * @author Max Clemetson & Justin Baratta
         * @since 2025-10
         * @return void
         */

        // Get the current pose of the robot
        Pose2d pose = swerveDrive.getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
            );

        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rotation = speeds.omegaRadiansPerSecond;

        swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rotation, true, false);  
    }

    public SwerveDrive getSwerveDrive() {
        /**
         * Gets the swerve drive object.
         * 
         * @args None
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return SwerveDrive
         */
        return swerveDrive;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        /**
         * Adds a vision measurement to the swerve drive, uses standard 
         * deviations to weight the measurement along with swerve drive's odometry.
         * 
         * @args Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs
         * @author Max Clemetson & Jake Xie
         * @since 2025-10
         * @return void
         */
        if (!initialized){
            return;
        }
        
        swerveDrive.addVisionMeasurement(visionPose, timestamp, estimationStdDevs);
    }

    public double lockToPoint(double targetPointX, double targetPointY) {
        /**
         * Locks the robot's orientation to a point on the field.
         * 
         * @args double targetPointX, double targetPointY
         * @author Justin Baratta - refactored from original code written by Siddhartha Hiremath
         * @since 2025-10
         * @return double target_angle
         */
        //get position
        Pose2d currentPosition = swerveDrive.getPose();
        double currentX = currentPosition.getX();
        double currentY = currentPosition.getY();

        System.out.println("Current Position: " + currentX + " " + currentY);
        System.out.println("Target Position: " + targetPointX + " " + targetPointY);

        double target_angle = Math.atan2(targetPointY - currentY, targetPointX - currentX);

        // set the robot's target angle  - rad to deg
        return target_angle;
    }

    public void driveWhileLocked(Translation2d translation, boolean isFieldRelative, Pose2d targetPose){
        /**
         * Drives the robot while locked to a point on the field.
         * 
         * @args Translation2d translation, boolean isFieldRelative, Pose2d targetPose
         * @author Justin Baratta - refactored from original code
         * @since 2026-01-11
         * @return void
         */
        double targetAngle = lockToPoint(targetPose.getX(), targetPose.getY());
        headingController.setSetpoint(targetAngle);
        double rotation = headingController.calculate(swerveDrive.getPose().getRotation().getRadians());
        swerveDrive.drive(translation, rotation, isFieldRelative, false);
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRelative){
        /**
         * Drives the robot.
         * 
         * @args Translation2d translation, double rotation, boolean isFieldRelative
         * @author Justin Baratta
         * @since 2026-01
         * @return void
         */
        swerveDrive.drive(translation, rotation, isFieldRelative, false);
    }
}