package frc.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Auto {
    private AutoFactory autoFactory;
    private final AutoChooser autoChooser;

    public Auto(AutoFactory factory) {
        this.autoFactory = factory;
        // Make something to display on the GUI that allows the user to select the autonomous they want to run
        autoChooser = new AutoChooser();

        // I added the dummy command just to test things. For real autos, please make routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        autoChooser.addRoutine("Test Routine", this::genericAutoTest);

        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Choreo Chooser", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public AutoRoutine genericAutoTest() {
        AutoRoutine routine = autoFactory.newRoutine("Auto");

        // Load the routine's trajectories
        AutoTrajectory trajectory = routine.trajectory("Forward.traj");

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd()
            )
        );
        return routine;
    }

    public Command getAutoCommand(){
        //return Commands.runOnce(() -> chooser.getSelected().schedule());
        return autoChooser.selectedCommand();
    }

    public AutoFactory getAutoFactory(){
        return autoFactory;
    }
}
