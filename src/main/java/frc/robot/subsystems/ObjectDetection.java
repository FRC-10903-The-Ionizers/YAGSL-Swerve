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
      // Ensure OpenCV native library is loaded before creating any Mats to avoid UnsatisfiedLinkError
      try {
         System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      } catch (UnsatisfiedLinkError e) {
         System.err.println("Failed to load OpenCV native library: " + e.getMessage());
      }

      this.swerve = swerve;
      // create an nxn Gaussian kernel (normalized). Adjust n as needed (odd n gives symmetric kernel)
      int kernelSize = 67;
      int downsampleFactor = 8;
      Mat gaussianKernel = new Mat(kernelSize, kernelSize, CvType.CV_32F);
      double sigma = kernelSize / 6.0; // ~3 sigma covers most of the kernel
      double mean = (kernelSize - 1) / 2.0;
      double sum = 0.0;

      for (int r = 0; r < kernelSize; r++) {
         for (int c = 0; c < kernelSize; c++) {
            double dx = c - mean;
            double dy = r - mean;
            double v = Math.exp(-(dx * dx + dy * dy) / (2.0 * sigma * sigma));
            gaussianKernel.put(r, c, v);
            sum += v;
         }
      }

      // normalize so the kernel sums to 1
      for (int r = 0; r < kernelSize; r++) {
         for (int c = 0; c < kernelSize; c++) {
            double[] vv = gaussianKernel.get(r, c);
            gaussianKernel.put(r, c, vv[0] / sum);
         }
      }
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
            CvSource transformSource = CameraServer.putVideo("Transform Source", 640, 480);
            CvSource yellowSource = CameraServer.putVideo("Yellow Source", 640, 480);

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
              //System.out.println("Processing Frame for Object Detection");
              // filter for yellow objects using HSV color space
              Mat hsvMat = new Mat();
              Imgproc.cvtColor(mat, hsvMat, Imgproc.COLOR_BGR2HSV);
              Mat yellowMask = new Mat();
              Core.inRange(hsvMat, new Scalar(20, 100, 100), new Scalar(30, 255, 255), yellowMask);
               yellowSource.putFrame(yellowMask);
            // downscale yellow mask for performance
            // downscale yellow mask for performance (divide spatial dimensions by n; channels remain unchanged)
               Size origSize = yellowMask.size();
               Size downSize = new Size(origSize.width / downsampleFactor, origSize.height / downsampleFactor);
               Imgproc.resize(yellowMask, yellowMask, downSize, 0, 0, Imgproc.INTER_AREA);
              Mat invertcolormatrix = new Mat(yellowMask.rows(), yellowMask.cols(), yellowMask.type(), new Scalar(255, 255, 255));
              Mat invertedYellowMask = new Mat();
            

              // create density map by applying box filter
              Mat densityMap = new Mat();
              Imgproc.boxFilter(yellowMask, densityMap, -1, new Size(21, 21));
              Imgproc.filter2D(yellowMask, densityMap, -1, gaussianKernel);

              // Convert all mats to CV_32F for arithmetic operations
              Mat invertFloat = new Mat();
              invertcolormatrix.convertTo(invertFloat, CvType.CV_32F);
              densityMap.convertTo(densityMap, CvType.CV_32F);

              // Compute distance transform inversion
              //Core.subtract(invertFloat, distanceTransform, distanceTransform);
              invertFloat.release();

              // create 60-40 split of distance transform and density map
              Mat weightedMap = new Mat();

              // find brightest point in heatmap
              Core.MinMaxLocResult mmr = Core.minMaxLoc(weightedMap);

              // Normalize and convert all display mats to CV_8U for output
              Mat displayTransform = new Mat();
              Mat displayWeighted = new Mat();
              Core.normalize(densityMap, densityMap, 0, 255, Core.NORM_MINMAX);
              // convert weighted map to heatmap
              densityMap.convertTo(displayTransform, CvType.CV_8U);

              // Output frames to dashboard
              transformSource.putFrame(displayTransform);
              densitySource.putFrame(densityMap);
              outputStream.putFrame(densityMap);

               // get coordinates of brightest point
               Point brightestPoint = mmr.maxLoc;
               // convert to angle in radians
               double fovHorizontal = Constants.Vision.kObjectCameraFovHorizontal;
               double relativeYaw = (brightestPoint.x - mat.width() / 2.0) / mat.width() * fovHorizontal;
               setYaw(relativeYaw);

              // Release Mats to prevent memory leaks
              hsvMat.release();
              yellowMask.release();
              invertcolormatrix.release();
              invertedYellowMask.release();
              displayTransform.release();
              densityMap.release();
              weightedMap.release();
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

      // if (objectDetectionOn) {
      //    Command command = getCommand();
      //    command.schedule();
      // }
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

   public Command getCommand(){
      /**
       * Get the command to align the robot to the detected object.
       * 
       * @args None
       * @author Justin Baratta, refactored some code from Siddhartha Hiremath
       * @return Command
       * @since 2026-01-24
       */
      System.out.println("Get Command called");
         if(objectDetectionOn == false){
            return Commands.none();
         }
         double currentYaw = swerve.getPose().getRotation().getRadians();
         System.out.println("Current Yaw: " + currentYaw);
         System.out.println("Target Yaw: " + absoluteTargetYaw);

         // drive robot to face target
         //Commands.schedule(() -> {
         // swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), currentYaw - Math.toRadians(absoluteTargetYaw), false);
         //}, swerve);

         Command command = Commands.run(() -> {
            double updatedYaw = swerve.getPose().getRotation().getRadians();
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), -(updatedYaw - Math.toRadians(absoluteTargetYaw)), false);
            }, swerve).until(
               this::isFinished
            ).finallyDo((interrupted) -> {
               swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, false);
               System.out.println("Object Detection Alignment Complete");
         });

         return command;
   }

   public boolean isFinished(){
      double updatedYaw = swerve.getPose().getRotation().getRadians();
      double error = Math.abs(updatedYaw - Math.toRadians(absoluteTargetYaw));
      return error < Math.toRadians(0.5); 
   }
}