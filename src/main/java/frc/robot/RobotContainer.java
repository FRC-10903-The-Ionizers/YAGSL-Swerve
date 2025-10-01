package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(0);
    private final Controller.SwerveController swerveController = controller.new SwerveController();

    public RobotContainer() {
        SmartDashboard.putData("SwerveField", swerve.getField());
    }

    public void controllerDrive() {
        Translation2d driveTranslation = new Translation2d(swerveController.getDriveX(), swerveController.getDriveY());
        swerve.getSwerveDrive().drive(
            driveTranslation,
            swerveController.getRotation(),
            swerveController.isFieldRelative(),
            false
        );
    }

    public Swerve getSwerve(){
        return swerve;
    }
}
