package frc.robot.commands.ShootingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class AimAtPoint extends Command{

    private final Swerve swerve;
    private final Controller controller;
    private final Pose2d targetPose;

    public AimAtPoint(Swerve swerve, Controller controller, Pose2d targetPose) {
        this.swerve = swerve;
        this.controller = controller;
        this.targetPose = targetPose;
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
        return false;
    }
}
