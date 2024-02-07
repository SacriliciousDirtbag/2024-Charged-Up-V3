package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.util.HashSet;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.Config;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Rotation2d;


import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class cameraSubsystem extends SubsystemBase{
    static Thread m_visionThread;
    static double czp = 0;
    static Transform3d position = new Transform3d(); 
    static AprilTagPoseEstimator poseEst;
    static Rotation3d tagRotation;
    static Rotation2d tagRotation2d = new Rotation2d();


   
    public cameraSubsystem(){
    m_visionThread =
    new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
          // Set the resolution
          camera.setResolution(640, 480); //640x480

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", 192, 144);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();
          Mat grayMat = new Mat();

          Point pt0 = new Point();
          Point pt1 = new Point();
          Point pt2 = new Point();
          Point pt3 = new Point();
          Point center = new Point();
          Scalar red = new Scalar(0, 0, 255);
          Scalar green = new Scalar(0, 255, 0);


          AprilTagDetector aprilTagDetector = new AprilTagDetector();
          AprilTagDetector.Config config = aprilTagDetector.getConfig();
          config.quadSigma = 0.8f;
          aprilTagDetector.setConfig(config);
          AprilTagDetector.QuadThresholdParameters quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
          quadThreshParams.minClusterPixels = 50;
          quadThreshParams.criticalAngle *= 5; // default is 10
          quadThreshParams.maxLineFitMSE *= 1.5;
          aprilTagDetector.setQuadThresholdParameters(quadThreshParams);
          aprilTagDetector.addFamily("tag16h5");


          AprilTagPoseEstimator.Config PestConfig = new AprilTagPoseEstimator.Config(0.1524,699.3778103158814,677.7161226393544,345.6059345433618,207.12741326228522);
          AprilTagPoseEstimator poseEst = new AprilTagPoseEstimator(PestConfig);


          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            if (cvSink.grabFrame(mat) == 0) {
              outputStream.notifyError(cvSink.getError());
              continue;
            }


            Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

            AprilTagDetection results[] = aprilTagDetector.detect(grayMat);

            var set = new HashSet<>();

            for (var result: results) {
             
              if(result.getId() != 3)
              {
                continue;
               
              }

              pt0.x = result.getCornerX(0);
              pt1.x = result.getCornerX(1);
              pt2.x = result.getCornerX(2);
              pt3.x = result.getCornerX(3);

              pt0.y = result.getCornerY(0);
              pt1.y = result.getCornerY(1);
              pt2.y = result.getCornerY(2);
              pt3.y = result.getCornerY(3);

              center.x = result.getCenterX();
              center.y = result.getCenterY();

              set.add(result.getId());

              position = poseEst.estimateHomography(result);

              czp = position.getZ();

              Imgproc.line(mat, pt0, pt1, red, 5);
              Imgproc.line(mat, pt1, pt2, red, 5);
              Imgproc.line(mat, pt2, pt3, red, 5);
              Imgproc.line(mat, pt3, pt0, red, 5);


              Imgproc.putText(mat,String.valueOf(czp), pt1, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);
              Imgproc.circle(mat, center, 4, green);
              Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2, green, 7);

              SmartDashboard.putNumber("Tag Z Position", position.getZ());
              SmartDashboard.putNumber("Tag X Position", position.getX());
              SmartDashboard.putNumber("Tag Y Position", position.getY());

              tagRotation = new Rotation3d(position.getRotation().getQuaternion());
              tagRotation2d = new Rotation2d(tagRotation.toRotation2d().getRadians());
              SmartDashboard.putNumber("Tag Angle", tagRotation.getAngle());
              SmartDashboard.putNumber("Tag Degrees", tagRotation2d.getDegrees());

              SmartDashboard.putNumber("CAMERA CZP", czp);

            }

            outputStream.putFrame(mat);
          }
          aprilTagDetector.close();

        }); 
            
        if(!m_visionThread.isAlive())
        {
            m_visionThread.setDaemon(true);
            m_visionThread.start();
        }
    }
    
    public void startCamera() {
        m_visionThread.start();
    }

    public void stopCamera() {
        m_visionThread.interrupt();
    }
  
  public static double getZVal() {
    return position.getZ();
  }

  public double getXVal() {
    return position.getX();
  }

  public static double getDegrees() {
    return tagRotation2d.getDegrees();
  }
  
  public Double getCZP(){
    return czp;

  }

  @Override
  public void periodic(){
    SmartDashboard.putNumber("czp", czp);
  }
}