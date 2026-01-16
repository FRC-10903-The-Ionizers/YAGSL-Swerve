package frc.robot;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import choreo.auto.AutoFactory;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DrivingCommands.TeleopDriveCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.Auto;
import frc.robot.util.Controller;

public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private final Timer timer = new Timer();
    private AutoFactory autoFactory;
    private Auto auto;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        Swerve swerve = robotContainer.getSwerve();

        autoFactory = new AutoFactory(
            swerve::getPose, // A function that returns the current robot pose
            swerve::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerve::followTrajectory, // T`he drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerve // The drive subsystem
        );

        auto = new Auto(autoFactory);
        RobotModeTriggers.autonomous().onTrue(auto.getAutoCommand());
        // Optionally disable the joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true);

        // Starts file server to host the Elastic layout to be fetched by client from robot
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        System.out.println("Auto started...");
        timer.restart();
        // if (autoCommand != null) {
        //     autoCommand.schedule();
        // }
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopInit() {

        Swerve swerve = robotContainer.getSwerve();
        Controller controller = robotContainer.getController();
        TeleopDriveCommand teleopDriveCommand = new TeleopDriveCommand(swerve, controller);

        swerve.setDefaultCommand(teleopDriveCommand);
    }

    @Override
    public void teleopPeriodic() {
        
    }

    @Override
    public void teleopExit() {
        Swerve swerve = robotContainer.getSwerve();
        swerve.removeDefaultCommand();
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
