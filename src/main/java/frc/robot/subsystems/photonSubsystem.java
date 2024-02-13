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
  Swerve s_Swerve;
  PhotonPipelineResult result; 
  Transform3d pose;
  double range; 
  AprilTagFieldLayout aprilTagFieldLayout;
  ArrayList<PhotonTrackedTarget> targets;



 /**
   * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  

  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

  private SwerveDrivePoseEstimator poseEstimator;

  private final Field2d field2d = new Field2d();

  private double previousPipelineTimestamp = 0;
  
    public photonSubsystem(PhotonCamera photonCamera, Swerve s_Swerve){
      this.camera = photonCamera;
      this.s_Swerve = s_Swerve;
      AprilTagFieldLayout layout;
      try{
        layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        var alliance = DriverStation.getAlliance();
        //sets field to resource
        this.aprilTagFieldLayout = layout;
      } catch(IOException e){
        DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      }
      

      poseEstimator = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics, 
        s_Swerve.getYaw(), 
        s_Swerve.getModulePositions(), 
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);
    }
  
  @Override
  public void periodic(){
    //update pose estimator
    if(camera != null){
    var piplelineResult = camera.getLatestResult();
    SmartDashboard.putBoolean("THe Truth", true);

    if(piplelineResult.hasTargets()) {
    var target = piplelineResult.getBestTarget();
    var fiducialId = target.getFiducialId();

    //get tag from pose from field layout
      Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);

    if(target.getPoseAmbiguity() <=  .2 && fiducialId >= 0 && tagPose.isPresent()){
      var targetPose = tagPose.get(); 
    }
  }
  }

  }

  public double getYaw()
  {
    if(targets == null)
    {
      return 0.0;
    }
    else
    {
      return targets.get(0).getYaw();
    }
  }

  public double getDistance()
  {
    if(targets == null)
    {
      return 0.0;
    }
    else
    {
      return range;
    }
  }
}
