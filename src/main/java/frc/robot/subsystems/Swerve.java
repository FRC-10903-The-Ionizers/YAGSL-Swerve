package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import java.io.File;

public class Swerve extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private boolean initialized = false;
    private Field2d field = new Field2d();

    public Swerve() {
        
        try {
            // On the RIO, deploy files live here:
            double maximumSpeed = Units.feetToMeters(4.5);
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
}
