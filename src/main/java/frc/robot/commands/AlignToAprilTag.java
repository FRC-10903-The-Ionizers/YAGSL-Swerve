package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** An example command that uses an example subsystem. */
public class AlignToAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve swerve;
  private final Vision vision;
  private double turn;

  public AlignToAprilTag(Swerve swerve, Vision vision) {
    this.swerve = swerve;
    this.vision = vision;
  }
  public void initialize() {
    // not much neeeded here
  }
  public void execute() {
    // Get the closest AprilTag
    PhotonTrackedTarget closestTarget = vision.getClosestAprilTagYaw();
    if (closestTarget == null) {
        turn = 0;
        return;
    }
    // get the yaw of the closest target
    double closestTargetYaw = closestTarget.getYaw();

    turn = -1.0 * closestTargetYaw * DriveConstants.kMaxTurnRate;
    swerve.getSwerveDrive().drive(new ChassisSpeeds(0, 0, turn));
  }
}