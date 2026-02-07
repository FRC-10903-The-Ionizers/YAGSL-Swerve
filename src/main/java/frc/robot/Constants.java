package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.Auto;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class Constants {
    /**
     * Constants class for the robot.
     * 
     * @author Max Clemetson, Jake Xie, Siddhartha Hiremath, Justin Baratta
     * @since 2025-10
     */

    public static class DriveConstants {
        /**
         * Drive constants for the robot.
         * 
         * @author Max Clemetson, Jake Xie, Siddhartha Hiremath, Justin Baratta
         * @since 2025-10
         */
        public static final double kControllerDriveMultiplier = 1.0;
        public static final double kControllerRotationMultiplier = 2.0;
        public static final double kAutoDriveMultiplier = 4.5;
        public static final double kLOW_GEAR_MULTIPLIER = 1.0;
        public static final double kHIGH_GEAR_MULTIPLIER = 2.0;
        
        public static class GoToPoseConstants {
            public static final double kMinimumDistanceToStop = 0.1;

            public static final double kXP = 1.0;
            public static final double kXI = 0.0;
            public static final double kXD = 0.0;

            public static final double kYP = 1.0;
            public static final double kYI = 0.0;
            public static final double kYD = 0.0;
            
            public static final double kHeadingP = 4.0;
            public static final double kHeadingI = 0.0;
            public static final double kHeadingD = 0.0;

        }
    }
    public static class Vision {
        /**
         * Vision constants for the robot. Make sure to set the camera names in case u have different cameras.
         * 
         * @author Max Clemetson, Jake Xie, SIddhartha Hiremath
         * @since 2025-10
         */
        public static String kAprilCameraName = "Arducam_OV9281_USB_Camera";
        public static String kObjectCameraName = "Arducam_OV9782_USB_Camera";
        public static Double kObjectCameraFovHorizontal = 70.0;
        public static Transform3d kRobotToCam = new Transform3d();
        public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);;

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutoConstants {
        /**
         * Auto constants for the robot.
         * 
         * @author Max Clementson, Jake Xie, Siddhartha Hiremath, Justin Baratta
         * @since 2026-01
         */
        public static AutoFactory kAutoFactory;
        public static final double kDelayTime = 0.08;
        public static final Angle kAutoAngleEpsilon = Units.Degrees.of(1.0);
    }
}
