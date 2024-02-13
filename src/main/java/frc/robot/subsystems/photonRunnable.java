// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.RobotState;


// // import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
// // import static frc.robot.Constants.VisionConstants.APRILTAG_CAMERA_TO_ROBOT;
// // import static frc.robot.Constants.VisionConstants.FIELD_LENGTH_METERS;
// // import static frc.robot.Constants.VisionConstants.FIELD_WIDTH_METERS;

// import java.io.IOException;
// import java.util.concurrent.atomic.AtomicReference;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;

// import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.RobotState;

// public class photonRunnable implements Runnable {
// private final PhotonPoseEstimator photonPoseEstimator;
//   private final PhotonCamera photonCamera;
//   private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

//   public photonRunnable() {
//     this.photonCamera = new PhotonCamera("photonvision1");
//     PhotonPoseEstimator photonPoseEstimator = null;
//     this.photonPoseEstimator = photonPoseEstimator;
//   }

//     @Override
//     public void run() {      
//     // Get AprilTag data
//     if (photonPoseEstimator != null && photonCamera != null && !RobotState.isTeleop()) {
//       var photonResults = photonCamera.getLatestResult();


//       if (photonResults.hasTargets()) {
//         photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
//           var estimatedPose = estimatedRobotPose.estimatedPose;


//           // Make sure the measurement is on the field
//           if (estimatedPose.getX() > 0.0 && estimatedPose.getY() > 0.0 ) {
//             atomicEstimatedRobotPose.set(estimatedRobotPose);
//           }


//         });
//       }
//     }  
// }

// public EstimatedRobotPose grabLastedEstimatedRobotPose()
// {
//     return atomicEstimatedRobotPose.getAndSet(null);
// }


// }


