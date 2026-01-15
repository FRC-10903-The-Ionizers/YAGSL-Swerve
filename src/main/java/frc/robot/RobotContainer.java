package frc.robot;

import frc.robot.stateSensors.RegionHandler;
import frc.robot.subsystems.ObjectDetection;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import frc.robot.subsystems.Vision;
import frc.robot.commands.DrivingCommands.TeleopDriveCommand; // Assuming this command exists for default driving
import frc.robot.commands.DrivingCommands.TeleopAimDrive; // Assuming this command exists for aiming
import frc.robot.commands.DrivingCommands.TeleopDriveToObject; // Assuming this command exists for aligning
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.File;
import java.util.Queue;

public class RobotContainer {
    
    private final Swerve swerve = new Swerve();
    private final Controller controller = new Controller(0);
    private final RegionHandler regionHandler = new RegionHandler(new File(Filesystem.getDeployDirectory(), "misc/regions.json"));
    private final Vision vision = new Vision(swerve);
    CommandXboxController xboxController = new CommandXboxController(0);
    private final ObjectDetection detections = new ObjectDetection();

    public RobotContainer() {

        configureBindings();
        SmartDashboard.putData("SwerveField", swerve.getField());
        System.out.println(regionHandler.getAllRegionNames());

    }

    // Remove controllerDrive() as it's replaced by commands

    public Swerve getSwerve(){
        return swerve;
    }
    
    public Controller getController(){
        return controller;
    }

    private void configureBindings(){
        // Bind aiming command to a button (e.g., A button)
        xboxController.a().whileTrue(new TeleopAimDrive(swerve, controller, new Pose2d(0, 0, null))); // Pass appropriate target pose

        // Bind aligning command to another button (e.g., B button)
        xboxController.b().whileTrue(new TeleopDriveToObject(swerve, controller));

        // // Region triggers for scheduling commands
        // Trigger inRegion1 = new Trigger(() -> regionHandler.inRegion("region1", swerve.getPose()));
        // Trigger inRegion2 = new Trigger(() -> regionHandler.inRegion("region2", swerve.getPose()));

        // // Example: Schedule a command when entering region1
        // inRegion1.onTrue(Commands.runOnce(() -> System.out.println("Robot entered region1")).andThen(/* Add specific command if needed */));
        // inRegion1.onFalse(Commands.runOnce(() -> System.out.println("Robot left region1")));

        // inRegion2.onTrue(Commands.runOnce(() -> System.out.println("Robot entered region2")).andThen(/* Add specific command if needed */));
        // inRegion2.onFalse(Commands.runOnce(() -> System.out.println("Robot left region2")));

        // // Combined trigger example
        // inRegion1.and(inRegion2).onTrue(Commands.runOnce(() -> System.out.println("Robot is in both region1 and region2")));
    }
}