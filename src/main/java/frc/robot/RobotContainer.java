package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(0);
    private final Controller.SwerveController swerveController = controller.new SwerveController();

    public RobotContainer() {
        // Default command that defers driving logic to SwerveController
        swerve.setDefaultCommand(
            new RunCommand(
                () -> swerve.drive(
                        swerveController.getDriveX(),
                        swerveController.getDriveY(),
                        swerveController.getRotation(),
                        swerveController.isFieldRelative()
                ),
                swerve
            )
        );
    }
}
