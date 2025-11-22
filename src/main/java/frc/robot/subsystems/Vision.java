package frc.robot.subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("GENERAL_WEBCAM");

    public void tryToSeeTag(){
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results){    
            for(PhotonTrackedTarget target : result.getTargets()){
                System.out.println(target.getBestCameraToTarget());
            }
        }

        
    }
}
