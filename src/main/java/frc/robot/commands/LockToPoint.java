package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import java.lang.Math;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
/** An example command that uses an example subsystem. */
public class LockToPoint extends Command {
  //lock rotation to target point
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve swerve;
  
  private final double targetPointX;
  private final double targetPointY;
  private double target_angle;

  public LockToPoint (Swerve subsystem, double targetPointX, double targetPointY) {
    swerve = subsystem;
    this.targetPointX = targetPointX;
    this.targetPointY = targetPointY;
  }

  public void execute() {
    //get position
    Pose2d currentPosition = swerve.getPose();
    double currentX = currentPosition.getX();
    double currentY = currentPosition.getY();
    // log current position
    System.out.println("Current X: " + currentX);
    System.out.println("Current Y: " + currentY);
    
    target_angle = Math.atan2(targetPointY - currentY, targetPointX - currentX);
    System.out.println(target_angle);

    // set the robot's target angle  - rad to deg
    swerve.setTargetAngle(Math.toDegrees(target_angle));
  }
}