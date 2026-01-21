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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    private final PIDController xController = new PIDController(1, 0, 0);
    private final PIDController yController = new PIDController(1, 0, 0);
    private final PIDController headingController = new PIDController(1, 0, 0);

    public Swerve() {
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
        if (!initialized)
            return;

        field.setRobotPose(swerveDrive.getPose());
    }

    public Pose2d resetOdometry(Pose2d pose) {
        if (!initialized)
            return new Pose2d();
        swerveDrive.resetOdometry(pose);
        return swerveDrive.getPose();
    }

    // Elastic Stuff

    public Field2d getField() {
        return field;
    }

    public Pose2d getPose() {
        if (!initialized)
            return new Pose2d();
            
        return swerveDrive.getPose();
    }

    public void followTrajectory(SwerveSample sample) {
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
        return swerveDrive;
    }

    public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
        if (!initialized){
            return;
        }
        
        swerveDrive.addVisionMeasurement(visionPose, timestamp, estimationStdDevs);
    }

    public double lockToPoint(double targetPointX, double targetPointY) {
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
        double targetAngle = lockToPoint(targetPose.getX(), targetPose.getY());
        headingController.setSetpoint(targetAngle);
        double rotation = headingController.calculate(swerveDrive.getPose().getRotation().getRadians());
        swerveDrive.drive(translation, rotation, isFieldRelative, false);
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRelative){
        swerveDrive.drive(translation, rotation, isFieldRelative, false);
    }
}