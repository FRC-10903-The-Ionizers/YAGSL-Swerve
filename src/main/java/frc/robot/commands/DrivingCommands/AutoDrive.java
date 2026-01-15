package frc.robot.commands.DrivingCommands;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.Controller;

public class AutoDrive extends Command{
    
    private Swerve swerve;

    public AutoDrive(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        AutoFactory autofactory = new AutoFactory(
            swerve::getPose,
            swerve::resetOdometry,
            swerve::followTrajectory,
            false,
            swerve
        );
        Auto auto = new Auto(autofactory);

        auto.getAutoCommand().schedule();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}
