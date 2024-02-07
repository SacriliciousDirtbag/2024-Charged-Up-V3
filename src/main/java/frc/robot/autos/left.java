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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class left extends SequentialCommandGroup {
    public left(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =

            // Wrap around charging station //

            //RIGHT SIDE POSITION
            /*TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making a curved path
                List.of(new Translation2d(-3.5, 0), new Translation2d(-2, -1.5), new Translation2d(-0.25, -1.5)), 
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-0.20, -1.5, new Rotation2d(0)),
                config);*/

            //MIDDLE POSITION
            /*TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making a curved path
                List.of(new Translation2d(-1, 0), new Translation2d(-1.5, 0.1)), 
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2.5, 0.1, new Rotation2d(0)),
                config);*/



            //LEFT SIDE POSITION
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making a curved path
                List.of(new Translation2d(-3.5, 0), new Translation2d(-2, 1.5), new Translation2d(-1.5, 1.5), new Translation2d(-0.25, 1.5)), 
                // End 3 meters straight ahead of where we started, facing forward
                    new Pose2d(-0.50, 1.5, new Rotation2d(0)),
                    config);

            
            // Drive on to charging station //
            /*TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making a straight path
                List.of(new Translation2d(-2, 0), new Translation2d(-1.5, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1.25, 0, new Rotation2d(0)),
                config);*/
                
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            swerveControllerCommand
        );
    }
}