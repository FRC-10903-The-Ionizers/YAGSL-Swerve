package frc.robot.commands.DrivingCommands;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.controller.PIDController;

public class MovementOriented extends Command {
  /**
   * GoToPoint command for the robot.
   * 
   * @since 2026-01-20
   */

  private final Swerve swerve;
  private final Controller controller;
  // heading controller
  private final PIDController headingController = new PIDController(Constants.DriveConstants.kHeadingP, 0, 0);

  public MovementOriented(Swerve subsystem, Controller controller) {
    /**
     * GoToPoint constructor for the robot.
     * 
     * @args Swerve subsystem, Pose2d targetPose
     * @since 2026-01-21
     * @return void
     */
    swerve = subsystem;
    this.controller = controller;
    addRequirements(swerve);
  }

  public void initialize() {
    /**
     * @todo Remove this unless needed
     */
  }
  
  public void execute() {
    /**
     * Executes the GoToPoint command.
     * Periodically updates the translation and heading distance to the target pose by subtracting the current pose from the target pose.
     * @args None
     * @since 2026-01-21
     * @return void
     */

      //rotation oriented to movement direction
      // get movement direction
      double movementDirection = Math.atan2(controller.t, translation.getX());
      // set the robot's target angle to the movement direction
      headingController.setSetpoint(movementDirection);
      double rotationOutput = headingController.calculate(swerveDrive.getPose().getRotation().getRadians());
      swerveDrive.drive(translation.times(currentGear), rotationOutput, isFieldRelative, false);
  }
    System.out.println("Movement oriented mode enabled");
  }

  @Override
  public void end(boolean interrupted) {
    swerve.setMovementOriented(false);
  }
}