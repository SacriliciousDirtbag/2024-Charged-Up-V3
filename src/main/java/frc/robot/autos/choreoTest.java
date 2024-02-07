package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class choreoTest extends SequentialCommandGroup {
    private BooleanSupplier isBlueField;
    
    public choreoTest(Swerve s_Swerve){

        // An example trajectory to follow.  All units in meters.
        ChoreoTrajectory exampleTrajectory = Choreo.getTrajectory("Trajectory.1");
                
       
 
        var thetaController =
            new PIDController(
                Constants.AutoConstants.kPThetaController, 0, 0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        ChoreoControlFunction choreoControl = Choreo.choreoSwerveController(new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), thetaController);

        Command choreoSwerveCommand = Choreo.choreoSwerveCommand(
            exampleTrajectory, 
            s_Swerve::getPose,
            choreoControl,
            // new PIDController(Constants.AutoConstants.kPXController, 0.0, 0.0), // 
            // new PIDController(Constants.AutoConstants.kPYController, 0.0, 0.0), // 
            // thetaController, // 
            (ChassisSpeeds speeds) -> // 
                s_Swerve.drive(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), speeds.omegaRadiansPerSecond, false, false),
            isBlueField, //Whether or not to mirror the path based on alliance (this assumes the path is created for the blue alliance)
            s_Swerve 
        );


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            choreoSwerveCommand

        );
    }
}