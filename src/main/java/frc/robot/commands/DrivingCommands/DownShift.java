package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Controller;

public class DownShift extends Command {
    /**
     * Shifter command for the robot, used for shifting gears
     * 
     * @author Siddhartha Hiremath
     * @since 2026-01-15
     */

    public DownShift(Swerve swerve) {
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        if (Swerve.currentGear-1.0 > Swerve.LOW_GEAR_MULTIPLIER) {
            Swerve.currentGear -= 1.0;
            System.out.println("Shifted to LOW gear");
        }
        else {
            Swerve.currentGear = Swerve.LOW_GEAR_MULTIPLIER;
            System.out.println("Shifted to LOW gear");
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}