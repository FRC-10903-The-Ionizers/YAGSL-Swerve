package frc.robot;

import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    

    private final Controller controller = new Controller(0);
    private final Controller.SwerveController swerveController = controller.new SwerveController();
    private final RegionHandler regionHandler = new RegionHandler(new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
    private final Vision vision = new Vision(swerve);
    CommandXboxController xboxController = new CommandXboxController(0);

    public RobotContainer() {
        SmartDashboard.putData("SwerveField", swerve.getField());
        System.out.println(regionHandler.getAllRegionNames());
        // when u hit a, we start running lock to point

        xboxController.a().onTrue(new InstantCommand(() -> {swerve.turnOnLock(new Pose2d());}));
        xboxController.x().onTrue(new InstantCommand(() -> {swerve.turnOffLock();}));
        Trigger inRegion1 = new Trigger(() -> {
            boolean isInRegion = regionHandler.inRegion("region1", swerve.getPose());
            return isInRegion;
        });

        Trigger inRegion2 = new Trigger(() -> {
            boolean isInRegion = regionHandler.inRegion("region2", swerve.getPose());
            return isInRegion;
        });

        inRegion1.onTrue(Commands.runOnce(() -> System.out.println("Robot entered region1")))
                 .onFalse(Commands.runOnce(() -> System.out.println("Robot left region1")));

        inRegion2.onTrue(Commands.runOnce(() -> System.out.println("Robot entered region2")))
                 .onFalse(Commands.runOnce(() -> System.out.println("Robot left region2")));

        inRegion1.and(inRegion2).onTrue(Commands.runOnce(() -> System.out.println("Robot is in both region1 and region2")));

    }

    public void controllerDrive() {
        if (!detections.getDetectionOn()){
            Translation2d driveTranslation = new Translation2d(swerveController.getDriveX(), swerveController.getDriveY());
            swerve.getSwerveDrive().drive(
                driveTranslation,
                swerveController.getRotation(),
                swerveController.isFieldRelative(),
                false
            );
        }
    }

    public Swerve getSwerve(){
        return swerve;
    }
}
