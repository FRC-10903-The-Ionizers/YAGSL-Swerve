package frc.robot.subsystems;

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
import org.opencv.imgproc.Imgproc;

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


   private double yaw;
   private boolean currentlyTracking = false;
   private boolean objectDetectionOn = false;
   Thread m_visionThread;

   public ObjectDetection() {
      m_visionThread =
      new Thread(
          () -> {
            // Get the UsbCamera from CameraServer
            UsbCamera camera = CameraServer.startAutomaticCapture();
            // Set the resolution
            camera.setResolution(640, 480);

            // Get a CvSink. This will capture Mats from the camera
            CvSink cvSink = CameraServer.getVideo();
            // Setup a CvSource. This will send images back to the Dashboard
            CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

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
              // Put a rectangle on the image
              Imgproc.rectangle(
                  mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
              // Give the output stream a new image to display
              outputStream.putFrame(mat);
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

   }

   public double getYaw() {
      /**
       * Get the yaw of the detected object.
       * 
       * @args None
       * @author Siddhartha Hiremath
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
       * @author Siddhartha Hiremath
       */
      return objectDetectionOn;
   }
}