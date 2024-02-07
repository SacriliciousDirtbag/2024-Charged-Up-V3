package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;
//PHOTONVISION IMPORTS
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.*;



import java.util.HashSet;
import java.util.List;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class photonSubsystem extends SubsystemBase{
  PhotonCamera camera;
  PhotonPipelineResult result;
  Transform3d pose;



   
    public photonSubsystem(){
      camera = new PhotonCamera("photoncamera1");
      //var result = camera.getLatestResult();
      // PhotonTrackedTarget target = result.getBestTarget();
      // pose = target.getBestCameraToTarget();

      
    }
  
  @Override
  public void periodic(){
    
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();
    if(hasTargets) {
      ArrayList<PhotonTrackedTarget> targets = new ArrayList<PhotonTrackedTarget>();
      for(PhotonTrackedTarget target : targets){
        targets.add(target);
      }

      pose = targets.get(0).getBestCameraToTarget(); 
      SmartDashboard.putNumber("X-Pos", pose.getX());
      SmartDashboard.putNumber("Y-Pos", pose.getY());
      SmartDashboard.putNumber("Z-Pos", pose.getZ());
    }

  }

  public double getRotation()
  {
    
  }

  public double getDistance()
  {

  }
}
