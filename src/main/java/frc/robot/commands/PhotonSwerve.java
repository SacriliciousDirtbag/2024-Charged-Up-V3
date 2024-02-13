package frc.robot.commands;

import java.util.ArrayList;

import com.fasterxml.jackson.core.TreeNode;

import frc.robot.Constants;
import frc.robot.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;


public class PhotonSwerve extends Command{
    PIDController movementController;
    photonSubsystem camera;
    SwerveModule[] swerveModules = new SwerveModule[4];

    SwerveModule[] frontModules = new SwerveModule[2];
    SwerveModule[] backModules = new SwerveModule[2];

    PIDController pid = new PIDController(Constants.AutoConstants.kPXController, Constants.AutoConstants.kPYController, Constants.AutoConstants.kPThetaController);
    

    double currentPosition;
    double currentAngle;
    double cyp;
    Swerve s_Swerve;
    Translation2d translation; 

    public PhotonSwerve(photonSubsystem camera, Swerve s_Swerve)  //, SwerveModule[] frontModules, SwerveModule[] backModules
    {
        //this.movementController = movementController;
        this.camera = camera;
        this.s_Swerve = s_Swerve;
        // for(int i = 0; i < frontModules.length; i++)
        // {
        //     frontModules[i] = swerveModules[i] ; // every odd
        //     backModules[i] = swerveModules[i+1]; // every 
        // }
    }

    @Override
    public void initialize() 
    {
        // movementController.setSetpoint(0);
        // movementController.setTolerance(0.5);
        currentAngle = camera.getYaw();
        cyp = camera.GXP();
        currentPosition = camera.getDistance();
        translation = new Translation2d(currentPosition, cyp);
        //new Rotation2d(Units.degreesToRadians(currentAngle)));
    }

    @Override 
    public void execute()
    {
        s_Swerve.drive(translation, currentAngle, false,false);
        currentAngle -= camera.getYaw();
        cyp = camera.GXP();
        currentPosition = camera.getDistance();
        translation = new Translation2d(currentPosition,cyp);
        // new Rotation2d(Units.degreesToRadians(currentAngle)));
    
        // currentAngle = -1 * (currentAngle - camera.getYaw());                                               
        // currentPosition = camera.getDistance();

        // movementController.calculate(currentPosition);
        // SwerveModuleState[] mo = {new SwerveModuleState(movementController.calculate(currentPosition),new Rotation2d(Units.degreesToRadians(currentAngle)))};

        // SwerveDriveKinematics.desaturateWheelSpeeds(mo, Constants.Swerve.maxSpeed);
    
        // int counter = 0; 
        // for(SwerveModule i : swerveModules)
        // {
        //     if(counter == 2)
        //     {
        //         mo[0] = new SwerveModuleState(
        //             movementController.calculate(currentPosition), new Rotation2d(Units.degreesToRadians((currentAngle * 3) % 360)));
        //     }
        //     i.setDesiredState(mo[0], false);
        // }
    }

    @Override
    public void end(boolean isTrue)
    {
        SwerveModuleState stop = new SwerveModuleState();

        for(SwerveModule i : swerveModules)
        {
            i.setDesiredState(stop, false);
        }
    }

    @Override
    public boolean isFinished()
    {
        if(currentPosition < 0.5 && currentAngle < -0.5 && currentAngle > 0.5)
        {
             return true;
        }
        return false;
    }
}
