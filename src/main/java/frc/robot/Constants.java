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

public class Constants {

    public static class DriveConstants {
        public static final double kControllerDriveMultiplier = 1.0;
        public static final double kControllerRotationMultiplier = 2.0;
        public static final double autoDriveMultiplier = 4.5;
    }
    public static class Vision {
        public static String aprilCameraName = "Arducam_OV9281_USB_Camera";
        public static String objectCameraName = "Arducam_OV9782_USB_Camera";
        public static Transform3d kRobotToCam = new Transform3d();
        public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);;

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutoConstants {
        public static AutoFactory autoFactory;
    }
}
