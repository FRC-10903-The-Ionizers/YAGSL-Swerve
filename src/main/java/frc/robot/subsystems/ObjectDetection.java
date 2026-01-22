package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.Constants.Vision.kRobotToCam;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants;

public class ObjectDetection extends SubsystemBase {
   /**
    * ObjectDetection subsystem for detecting objects using PhotonVision.
    * 
    * This subsystem is used to detect objects in the field of view of the camera.
    * It uses the PhotonVision library to detect objects and estimate their position.
    * Used to align the robot to detected objects.
    * 
    * @filename ObjectDetection.java
    * @author Siddhartha Hiremath
    * @since 2026-01-21
    * @todo Still need to implement movement side of object detection,
    *       also grouping targets together for better throughput.
    */

   private final PhotonCamera camera;
   private final PhotonPoseEstimator photonEstimator;
   private double yaw;
   private boolean currentlyTracking = false;
   private boolean objectDetectionOn = false;
   public ObjectDetection() {
      camera = new PhotonCamera(Constants.Vision.kObjectCameraName);

      photonEstimator =
               new PhotonPoseEstimator(Constants.Vision.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
      photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
   }
   public void periodic() {
      /**
       * Periodic method for detecting objects using PhotonVision.
       * 
       * @args None
       * @author Siddhartha Hiremath
       * @return void
       */
      var result = camera.getLatestResult();
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
      /**
       * Get the yaw of the detected object.
       * 
       * @args None
       * @return double yaw
       */
      if (!currentlyTracking){
         return 10903;
      }
      return yaw;
   }

   public boolean getDetectionOn(){
      /**
       * Get the detection status of the object.
       * 
       * @args None
       * @return boolean objectDetectionOn
       */
      return objectDetectionOn;
   }
}