package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class TeleopDriveCommand extends Command {
    /**
     * TeleopDriveCommand command for the robot. Literally just drives the robot.
     * 
     * @author Max Clementson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;

    public TeleopDriveCommand(Swerve swerve, Controller controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.drive(
            new Translation2d(controller.getDriveX(), controller.getDriveY()),
            controller.getRotation(),
            controller.isFieldRelative()
        );
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously
    }
}