package frc.robot.commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.ObjectDetection;
/** An example command that uses an example subsystem. */
public class ObjectDetectionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ObjectDetection objectDetection;
  
  public ObjectDetectionCommand(ObjectDetection objectDetection) {
    this.objectDetection = objectDetection;
  }

  public void execute() {
    System.out.println("wow");
    objectDetection.ObjectDetectionToggle();
  }
}