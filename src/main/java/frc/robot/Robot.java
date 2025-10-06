package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    // Loads a swerve trajectory, alternatively use DifferentialSample if the robot is tank drive
    private final Optional<Trajectory<SwerveSample>> trajectory = Choreo.loadTrajectory("Test.traj");
    private final Timer timer = new Timer();
    private AutoFactory autoFactory;
    private Swerve swerve;
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
        swerve = robotContainer.getSwerve();

        autoFactory = new AutoFactory(
            swerve::getPose, // A function that returns the current robot pose
            swerve::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerve::followTrajectory, // T`he drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerve // The drive subsystem
        );

        // Optionally disable the joystick connection warning
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {

        // if (trajectory.isPresent()) {
        //     System.out.println("Initial Pose Read");
        //     // Get the initial pose of the trajectory
        //     Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

        //     if (initialPose.isPresent()) {
        //         // Reset odometry to the start of the trajectory
        //         robotContainer.getSwerve().getSwerveDrive().resetOdometry(initialPose.get());
        //     }
        // }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
    }

    @Override
    public void autonomousPeriodic() {
        if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<SwerveSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
                robotContainer.getSwerve().followTrajectory(sample.get());
            }
        }
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.controllerDrive();
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }
}
