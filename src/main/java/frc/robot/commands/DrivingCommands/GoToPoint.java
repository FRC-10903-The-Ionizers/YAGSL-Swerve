package frc.robot.commands.DrivingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

public class GoToPoint extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve swerve;
  private final Pose2d targetPose;
  private final double minDistance = Constants.DriveConstants.kMinimumDistanceToStop;

  public GoToPoint(Swerve subsystem, Pose2d targetPose) {
    swerve = subsystem;
    this.targetPose = targetPose;
  }

  public void initialize() {

  }
  
  public void execute() {
    Pose2d currentPose = swerve.getPose();

    double x_distance = targetPose.getX() - currentPose.getX();
    double y_distance = targetPose.getY() - currentPose.getY();

    Translation2d translation = new Translation2d(x_distance, y_distance);

    double heading_distance = targetPose.getRotation().getDegrees() - currentPose.getRotation().getDegrees();
    
    swerve.drive(translation, heading_distance, true);
  }

  @Override
  public boolean isFinished(){

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