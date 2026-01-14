 package frc.robot.subsystems;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.numbers.N1;
 import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.Constants.Vision.kRobotToCam;

import java.util.List;
 import java.util.Optional;

 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Constants;

 public class ObjectDetection extends SubsystemBase {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
     private Swerve swerve;
     private double yaw;
     private boolean objectDetectionOn = false;
     public ObjectDetection(Swerve swerve) {
        this.swerve = swerve;
        camera = new PhotonCamera(Constants.Vision.kCameraName);

        photonEstimator =
                new PhotonPoseEstimator(Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
     }

     public void objectDetectionToggle() {
        objectDetectionOn = !objectDetectionOn;
     }
     public void periodic() {
      if (objectDetectionOn) {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         var result = camera.getLatestResult();
         boolean hasTargets = result.hasTargets();
         //System.out.print(result);
         System.out.print("object detection activated");
         if (hasTargets) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            double yaw = bestTarget.getYaw();
            System.out.println("Yaw: " + yaw);
         }
         // yaw relative to the robot
         swerve.setTargetAngle(yaw + swerve.getPose().getRotation().getDegrees());
      }
      }
 }