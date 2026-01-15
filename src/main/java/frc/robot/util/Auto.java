package frc.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Auto {
    private AutoFactory autoFactory;
    private SendableChooser<Command> chooser;

    public Auto(AutoFactory factory) {
        this.autoFactory = factory;
        // Make something to display on the GUI that allows the user to select the autonomous they want to run
        chooser = new SendableChooser<Command>();

        // I added the dummy command just to test things. For real autos, please make routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        chooser.addOption("dummyCommand", Commands.runOnce(() -> System.out.println("Autonomous Started!")));
        chooser.addOption("Routine", genericAutoTest().cmd());
        
        chooser.setDefaultOption("dummyCommand", Commands.runOnce(() -> System.out.println("Autonomous Started!")));

        SmartDashboard.putData("Auto Chooser", chooser);
    }

    public AutoRoutine genericAutoTest() {
        AutoRoutine routine = autoFactory.newRoutine("Auto");

        // Load the routine's trajectories
        AutoTrajectory trajectory = routine.trajectory("TestChoreoPath.traj");

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
        return chooser.getSelected();
    }

    public AutoFactory getAutoFactory(){
        return autoFactory;
    }
}
