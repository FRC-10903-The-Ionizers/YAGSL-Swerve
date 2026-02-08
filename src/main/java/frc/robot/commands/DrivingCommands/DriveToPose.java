package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPose extends Command {
  /**
   * GoToPoint command for the robot.
   * 
   * @author Justin Baratta - refactored from original code
   * @since 2026-01-20
   */
  private final Swerve swerve;
  private final Pose2d targetPose;

  public DriveToPose(Swerve subsystem, Pose2d targetPose) {
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
    swerve.driveToPose(targetPose);
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
    
    if(distance < Constants.DriveConstants.kMinimumDistanceToStop){
      return true;
    } else {
      return false;
    }
  }
}