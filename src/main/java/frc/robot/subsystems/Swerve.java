package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;

import choreo.trajectory.SwerveSample;

public class Swerve extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public Swerve() {
        
        try {
            headingController.enableContinuousInput(-Math.PI, Math.PI);
            double maximumSpeed = Units.feetToMeters(4.5);
            // On the RIO, deploy files live here:
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(),"swerve");
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
            System.out.println("Swerve is initialized!");
            initialized = true;
        } catch (Exception e) {
            System.out.println("Failed to load swerve config.");
            DriverStation.reportError("Failed to load swerve configuration from /deploy/swerve: " + e.getMessage(), e.getStackTrace());
        }
    }

    public void periodic() {
        if (!initialized) return;

        field.setRobotPose(swerveDrive.getPose());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        if (!initialized) return;
        
        swerveDrive.drive(new edu.wpi.first.math.geometry.Translation2d(xSpeed, ySpeed), rot, fieldRelative, false);
    }

    // Elastic Stuff

    public Field2d getField(){
        return field;
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
        // Apply the generated speeds
        drive(xSpeed, ySpeed, rotation, true);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

}
