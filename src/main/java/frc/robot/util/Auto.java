package frc.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import choreo.auto.AutoFactory;

public class Auto {
    private AutoFactory autoFactory;

    public Auto(AutoFactory factory) {
        this.autoFactory = factory;
    }

    public AutoRoutine genericAutoTest() {
        AutoRoutine routine = autoFactory.newRoutine("Auto");

        // Load the routine's trajectories
        AutoTrajectory trajectory = routine.trajectory("Test.traj");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd()
            )
        );

        return routine;
    }
}
