package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class TeleopAimDrive extends Command {
    /**
     * TeleopAimDrive command for the robot, used for aiming the robot to a point (represented by a Pose2d)
     * 
     * @author Max Clementson
     * @since 2026-01-15
     */
    private final Swerve swerve;
    private final Controller controller;
    private final Pose2d targetPose;

    public TeleopAimDrive(Swerve swerve, Controller controller, Pose2d targetPose) {
        this.swerve = swerve;
        this.controller = controller;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d driveTranslation = new Translation2d(controller.getDriveX(), controller.getDriveY());
        swerve.driveWhileLocked(driveTranslation, controller.isFieldRelative(), targetPose);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}