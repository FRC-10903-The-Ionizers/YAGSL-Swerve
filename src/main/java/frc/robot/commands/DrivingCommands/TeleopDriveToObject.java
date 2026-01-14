package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class TeleopDriveToObject extends Command {
    private final Swerve swerve;
    private final Controller controller;

    public TeleopDriveToObject(Swerve swerve, Controller controller) {
        this.swerve = swerve;
        this.controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d driveTranslation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        swerve.drive(driveTranslation, controller.getRotation(), controller.isFieldRelative());
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}