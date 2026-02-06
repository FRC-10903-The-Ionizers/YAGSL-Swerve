package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class GoToPoint extends Command {
  /**
   * GoToPoint command for the robot.
   * 
   * @author Justin Baratta - refactored from original code
   * @since 2026-01-20
   */
  private final Swerve swerve;
  private final Pose2d targetPose;
  private final double minDistance = Constants.DriveConstants.kMinimumDistanceToStop;

  private final PIDController xController = new PIDController(
    Constants.DriveConstants.GoToPointConstants.kXP, 
    Constants.DriveConstants.GoToPointConstants.kXI, 
    Constants.DriveConstants.GoToPointConstants.kXD
  );
  private final PIDController yController = new PIDController(
    Constants.DriveConstants.GoToPointConstants.kYP, 
    Constants.DriveConstants.GoToPointConstants.kYI, 
    Constants.DriveConstants.GoToPointConstants.kYD
  );
  private final PIDController headingController = new PIDController(
    Constants.DriveConstants.GoToPointConstants.kHeadingP, 
    Constants.DriveConstants.GoToPointConstants.kHeadingI, 
    Constants.DriveConstants.GoToPointConstants.kHeadingD
  );

  public GoToPoint(Swerve subsystem, Pose2d targetPose) {
    /**
     * GoToPoint constructor for the robot.
     * 
     * @args Swerve subsystem, Pose2d targetPose
     * @author Justin Baratta - refactored from original code
     * @since 2026-01-21
     * @return void
     */
    swerve = subsystem;
    this.targetPose = targetPose;
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
     * @author Justin Baratta - refactored from original code
     * @since 2026-01-21
     * @return void
     */
    Pose2d currentPose = swerve.getPose();
    double xCalculated = xController.calculate(currentPose.getX(), targetPose.getX());
    double yCalculated = yController.calculate(currentPose.getY(), targetPose.getY());
    double headingCalculated = headingController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
    
    swerve.drive(new Translation2d(xCalculated, yCalculated), headingCalculated, true);
  }

  @Override
  public boolean isFinished(){
    /**
     * Checks if the GoToPoint command is finished based on the minimum distance to stop.
     * 
     * @args None
     * @author Justin Baratta - refactored from original code
     * @since 2026-01-21
     * @return boolean
     */
    Pose2d currentPose = swerve.getPose();
    double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    System.out.println(distance);
    if(distance < minDistance){
      return true;
    } else {
      return false;
    }
  }
}