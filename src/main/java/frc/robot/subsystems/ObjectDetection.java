package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.Constants.Vision.kRobotToCam;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants;

public class ObjectDetection extends SubsystemBase {
   private final PhotonCamera camera;
   private final PhotonPoseEstimator photonEstimator;
   private double yaw;
   private boolean currentlyTracking = false;
   private boolean objectDetectionOn = false;
   public ObjectDetection() {
      camera = new PhotonCamera(Constants.Vision.objectCameraName);

      photonEstimator =
               new PhotonPoseEstimator(Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
   }
   public void periodic() {
      var result = camera.getAllUnreadResults().get(0);
      boolean hasTargets = result.hasTargets();
      //System.out.print(result);
      //System.out.println("object detection activated");
      if (hasTargets) {
         PhotonTrackedTarget bestTarget = result.getBestTarget();
         yaw = bestTarget.getYaw();
         // swerve.setRelativeTargetAngle(yaw);
         // SmartDashboard.putNumber("Target Angle", yaw);
         currentlyTracking = true;
      }
      else {
         currentlyTracking = false;
      }
   }

   public double getYaw() {
      if (!currentlyTracking){
         return 10903;
      }
      return yaw;
   }

   public boolean getDetectionOn(){
      return objectDetectionOn;
   }
}