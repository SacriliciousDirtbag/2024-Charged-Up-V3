package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.IntakeCone;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.subsystems.Swerve.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class swerveDriveDrive extends SequentialCommandGroup {
    public swerveDriveDrive(Swerve s_Swerve, ElevatorSubsystem s_ElevatorSubsystem, IntakeSubsystem s_IntakeSubsystem){
        double x = s_Swerve.getPose().getX();
        double z = s_Swerve.getPose().getRotation().getDegrees();
        
    addCommands(
        //new middle(s_Swerve),

        //new IntakeCone(s_ElevatorSubsystem, s_IntakeSubsystem),

        new driveDistance(2, 0, 0, s_Swerve)


    ); 
    
    
    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    }
}
