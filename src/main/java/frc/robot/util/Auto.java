package frc.robot.util;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Auto {
    private final AutoChooser autoChooser;

    public Auto(AutoFactory factory) {
        // Make something to display on the GUI that allows the user to select the autonomous they want to run
        autoChooser = new AutoChooser();

        // I added the dummy command just to test things. For real autos, please make routines and use their commands.
        // NEVER SET "getAutoCommand()" AS AN OPTION FOR AN AUTONOMOUS

        autoChooser.addRoutine("T", this::tRoutine);
        autoChooser.addRoutine("T then Loop", this::forwardThenLoopRoutine);
        autoChooser.addRoutine("Loop", this::loopRoutine);
        // Put the auto chooser on the dashboard
        SmartDashboard.putData("Choreo Chooser", autoChooser);

        // Schedule the selected auto during the autonomous period
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    public AutoRoutine tRoutine() {
        AutoRoutine routine = AutoConstants.autoFactory.newRoutine("Auto");

        // Load the routine's trajectories
        AutoTrajectory trajectory = routine.trajectory("T.traj");

        double totalTime = trajectory.getRawTrajectory().getTotalTime();
        
        trajectory.atTime(totalTime*0.5).onTrue(Commands.print("Halftime!"));

        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                trajectory.resetOdometry(),
                trajectory.cmd()
            )
        );


        return routine;
    }

    public AutoRoutine loopRoutine(){
        AutoRoutine routine = AutoConstants.autoFactory.newRoutine("Loop");

        AutoTrajectory loopTrajectory = routine.trajectory("Loop.traj");

        routine.active().onTrue(
            Commands.sequence(
                loopTrajectory.resetOdometry(),
                loopTrajectory.cmd()
            )
        );

        return routine;
    }

    public AutoRoutine forwardThenLoopRoutine(){
        AutoRoutine routine = AutoConstants.autoFactory.newRoutine("ForwardThenLoop");

        AutoTrajectory forwardTrajectory = routine.trajectory("Forward.traj");
        AutoTrajectory loopTrajectory = routine.trajectory("Loop.traj");

        routine.active().onTrue(
            Commands.sequence(
                forwardTrajectory.resetOdometry(),
                forwardTrajectory.cmd(),
                loopTrajectory.cmd()
            )
        );

        return routine;
    }

    public Command getAutoCommand(){
        //return Commands.runOnce(() -> chooser.getSelected().schedule());
        return autoChooser.selectedCommand();
    }
}
