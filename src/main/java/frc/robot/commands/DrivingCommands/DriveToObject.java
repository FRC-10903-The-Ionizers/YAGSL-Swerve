package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;
import frc.robot.Constants.Vision;
import edu.wpi.first.math.controller.PIDController;

public class DriveToObject extends Command {
    /**
     * TeleopDriveCommand command for the robot. Literally just drives the robot.
     * 
     * @author Max Clemetson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final PIDController controller;

    public DriveToObject(Swerve swerve, PIDController controller) {
        /**
         * Constructor which sets swerve and controller class variables
         * 
         * @param swerve: the swerve drive
         * @param controller: the PID controller
         */
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
    /**
     * Method that drives the swerves
     */
    System.out.println("Executing DriveToObject command");
    double[] dataFromPython = LimelightHelpers.getPythonScriptData("object_detection");

    // index 0 is x value of brightest point, we want to center that
    double angle_diff = dataFromPython[0]/Vision.kObjectCameraPixelsHorizontal * Vision.kObjectCameraFovHorizontal;
    // feed into pid controller to get rotation output
    double rotationOutput = controller.calculate(angle_diff, 0);
    swerve.drive(new Translation2d(0.05, 0), rotationOutput, false);

    }
    public boolean isFinished() {
        return false; // Runs continuously
    }
}