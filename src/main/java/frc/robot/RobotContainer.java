package frc.robot;

import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.util.Controller;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.io.File;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(0);
    private final RegionHandler regionHandler = new RegionHandler(
            new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
    private final Vision vision = new Vision(swerve);

    public RobotContainer() {
        SmartDashboard.putData("SwerveField", swerve.getField());

        controller.bind();
    }

    public void controllerDrive() {
        Translation2d driveTranslation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        swerve.getSwerveDrive().drive(
                driveTranslation,
                controller.getRotation(),
                controller.isFieldRelative(),
                false);
    }

    public Swerve getSwerve() {
        return swerve;
    }
}
