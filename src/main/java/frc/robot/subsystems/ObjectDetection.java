package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonTrackedTarget;
import static frc.robot.Constants.Vision.kRobotToCam;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.TimedRobot;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.core.CvType;

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


   private double absoluteTargetYaw;
   private boolean currentlyTracking = false;
   private boolean objectDetectionOn = false;
   private Swerve swerve;
   Thread m_visionThread;

   public ObjectDetection(Swerve swerve) {
      this.swerve = swerve;
      m_visionThread =
      new Thread(
          () -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setResolution(640, 480);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            CvSource densitySource = CameraServer.putVideo("Density-Camera", 640, 480);
            CvSource binarySource = CameraServer.putVideo("Binary-Camera", 640, 480);
            CvSource transformSource = CameraServer.putVideo("Transform Source", 640, 480);
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("ObjectVision-Camera", 640, 480);

            // Mats are very memory expensive. Lets reuse this Mat.
            Mat mat = new Mat();

            // This cannot be 'true'. The program will never exit if it is. This
            // lets the robot stop this thread when restarting robot code or
            // deploying.
            while (!Thread.interrupted()) {
              // Tell the CvSink to grab a frame from the camera and put it
              // in the source mat.  If there is an error notify the output.
              if (cvSink.grabFrame(mat) == 0) {
                // Send the output the error.
                outputStream.notifyError(cvSink.getError());
                // skip the rest of the current iteration
                continue;
              }
              System.out.println("Processing Frame for Object Detection");
              // filter for yellow objects using HSV color space
              Mat hsvMat = new Mat();
              Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_BGR2HSV);
              Mat yellowMask = new Mat();
              Core.inRange(hsvMat, new Scalar(20, 100, 100), new Scalar(30, 255, 255), yellowMask);
              binarySource.putFrame(yellowMask);

              Mat invertcolormatrix = new Mat(yellowMask.rows(), yellowMask.cols(), yellowMask.type(), new Scalar(255, 255, 255));
              Mat invertedYellowMask = new Mat();
              Core.bitwise_not(yellowMask, invertedYellowMask);

              // Distance transform - gives distance from each pixel to nearest yellow pixel
              Mat distanceTransform = new Mat();
              Imgproc.distanceTransform(invertedYellowMask, distanceTransform, Imgproc.DIST_L2, 5);

              // Create density map by applying box filter
              Mat densityMap = new Mat();
              Imgproc.boxFilter(yellowMask, densityMap, CvType.CV_32F, new Size(101, 101));

              // Normalize both maps to 0-1 range for proper weighting
              Core.normalize(distanceTransform, distanceTransform, 0, 1, Core.NORM_MINMAX);
              Core.normalize(densityMap, densityMap, 0, 1, Core.NORM_MINMAX);

              // Invert distance transform so closer to yellow = higher value
              Mat ones = new Mat(distanceTransform.rows(), distanceTransform.cols(), CvType.CV_32F, new Scalar(1.0));
              Core.subtract(ones, distanceTransform, distanceTransform);
              ones.release();

              // Combine: 60% proximity (inverted distance), 40% density
              Mat weightedMap = new Mat();
              Core.addWeighted(distanceTransform, 0.6, densityMap, 0.4, 0, weightedMap);

              // Find brightest point in heatmap
              Core.MinMaxLocResult mmr = Core.minMaxLoc(weightedMap);

              // Convert display mats to CV_8U for output
              Mat displayTransform = new Mat();
              Mat displayDensity = new Mat();
              Mat displayWeighted = new Mat();
              Core.normalize(distanceTransform, displayTransform, 0, 255, Core.NORM_MINMAX);
              Core.normalize(densityMap, displayDensity, 0, 255, Core.NORM_MINMAX);
              Core.normalize(weightedMap, displayWeighted, 0, 255, Core.NORM_MINMAX);
              displayTransform.convertTo(displayTransform, CvType.CV_8U);
              displayDensity.convertTo(displayDensity, CvType.CV_8U);
              displayWeighted.convertTo(displayWeighted, CvType.CV_8U);

              // Output frames to dashboard
              transformSource.putFrame(displayTransform);
              densitySource.putFrame(displayDensity);
              outputStream.putFrame(displayWeighted);

               // get coordinates of brightest point
               Point brightestPoint = mmr.maxLoc;
               // convert to angle in radians
               double fovHorizontal = Constants.Vision.kObjectCameraFovHorizontal;
               double relativeYaw = (brightestPoint.x - mat.width() / 2.0) / mat.width() * fovHorizontal;
               //setYaw(relativeYaw);

              // Release Mats to prevent memory leaks
              hsvMat.release();
              yellowMask.release();
              invertedYellowMask.release();
              distanceTransform.release();
              densityMap.release();
              weightedMap.release();
              displayTransform.release();
              displayDensity.release();
              displayWeighted.release();
            }
          });
  m_visionThread.setDaemon(true);
  m_visionThread.start();
   }
   public void periodic() {
      /**
       * Periodic method for detecting objects using PhotonVision.
       * 
       * @args None
       * @author Siddhartha Hiremath
       * @return void
       */
      // get current yaw of robot, and calculate relative yaw to target
      if (objectDetectionOn) {
         double currentYaw = swerve.getPose().getRotation().getRadians();
         System.out.println("Current Yaw: " + currentYaw);
         System.out.println("Target Yaw: " + absoluteTargetYaw);

         // drive robot to face target
         //Commands.schedule(() -> {
         // swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), currentYaw - Math.toRadians(absoluteTargetYaw), false);
         //}, swerve);
         Command cmd = Commands.runOnce(() -> {
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), -(currentYaw - Math.toRadians(absoluteTargetYaw)), false);
         }, swerve);
         CommandScheduler.getInstance().schedule(cmd);
      }
   }

   public void setYaw(double relativeYaw){
      /**
       * Set the target angles for the detected object.
       * 
       * @args double relativeYaw - The relative yaw of the detected object.
       * @author Siddhartha Hiremath
       * @return void
       */
      // calculate absolute yaw
      absoluteTargetYaw = relativeYaw + swerve.getPose().getRotation().getRadians();

   }


   public void toggleObjectDetection(){
      /**
       * Get the detection status of the object.
       * 
       * @args None
       * @return boolean objectDetectionOn
       * @author Siddhartha Hiremath
       */
      objectDetectionOn = !objectDetectionOn;
   }
}