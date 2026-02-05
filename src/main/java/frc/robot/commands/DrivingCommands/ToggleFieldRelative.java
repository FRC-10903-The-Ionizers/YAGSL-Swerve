package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class ToggleFieldRelative extends Command {
    /**
     * Shifter command for the robot, used for shifting gears
     * 
     * @author Siddhartha Hiremath
     * @since 2026-01-15
     */
    private final Controller controller;
    public ToggleFieldRelative(Controller controller) {
        this.controller = controller;
    }

    @Override
    public void initialize() {
        controller.fieldRelative = !controller.fieldRelative;
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}