package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.Swerve.*;


public class AutoBalance extends CommandBase {

    public void exampleAuto(Swerve s_Swerve){
        double x = s_Swerve.getPose().getX();
        double z = s_Swerve.getPose().getRotation().getDegrees();
        
        //Periodically called
        if(s_Swerve.gyro.getRoll() < -3){
            s_Swerve.drive(new Translation2d(x + 0.1,0), 0, true, true);
            }
            
        if(s_Swerve.gyro.getRoll() > 3){
            s_Swerve.drive(new Translation2d(x - 0.1,0), 0, true, true); 
            }

        s_Swerve.drive(new Rotation2d(), z, true, true);
        
        /*if(s_Swerve.gyro.getRoll() == -0.1){
            s_Swerve.drive(new Translation2d(x,0), 0, true, true); 
            s_Swerve.drive(new Rotation2d(), z + 2.5, true, true);

            }*/
        
        /*if(s_Swerve.isTiltedBackwards() == true){
            s_Swerve.drive(new Translation2d(0.15,0), 0, true, true);
        }*/
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }
}