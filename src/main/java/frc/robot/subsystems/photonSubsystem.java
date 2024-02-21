package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.photonvision.targeting.PhotonPipelineResult;
//PHOTONVISION IMPORTS
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.*;

import java.io.IOException;
import java.nio.channels.Pipe;
import java.sql.Driver;
import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


public class photonSubsystem extends SubsystemBase{
  PhotonCamera camera;
  PhotonPipelineResult lf; //latest frame
  PhotonTrackedTarget ct; //current target
  Transform3d pti; //idk what this is arlen
  final double chog = Units.inchesToMeters(20.25); //camera height off ground
  final double thog = Units.inchesToMeters(34); //target height off ground
  final double cpr = 0; //camera pitch radians
  double distance;
  
  public photonSubsystem()
  {
    camera = new PhotonCamera( "PhotonCamera1");

  }

  @Override
  public void periodic()
  {
    lf = camera.getLatestResult();
    if(lf.hasTargets())
    {
      ct = lf.getBestTarget();
      distance = PhotonUtils.calculateDistanceToTargetMeters(
        chog, 
        thog, 
        cpr, 
        Units.degreesToRadians(ct.getYaw()));
        pti = ct.getBestCameraToTarget();
        SmartDashboard.putNumber("z", pti.getZ());
        SmartDashboard.putNumber("x", pti.getX());
        SmartDashboard.putNumber("y", pti.getY());
        SmartDashboard.putNumber("THEBIGANGLE", ct.getYaw());
        
    }
  }

  public double getYaw()
  {
    if(ct != null)
    {
      return ct.getYaw();
    }
    return 0.0;
  }

  public double getDistance()
  {
    if(ct != null)
    {
      return distance;
    }
    return 0.0;
  }

  public double GXP()
  {
    if(ct != null)
    {
      return ct.getBestCameraToTarget().getX();
    }
    return 0;
  }

  public Transform3d getTransform3d()
  {
    return ct.getBestCameraToTarget();
  }
}
