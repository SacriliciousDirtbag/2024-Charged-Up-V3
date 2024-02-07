package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.Constants;
import frc.robot.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;


public class PhotonSwerve extends Command{
    PIDController movementController;
    photonSubsystem camera;
    SwerveModule[] swerveModules = new SwerveModule[4];

    double currentPosition;
    double currentAngle;

    public PhotonSwerve(PIDController movementController, photonSubsystem camera, SwerveModule[] frontModules,
    SwerveModule[] backModules) 
    {
        this.movementController = movementController;
        this.camera = camera;
        for(int i = 0; i < frontModules.length; i++)
        {
            swerveModules[i] = frontModules[i];
            swerveModules[i+2] = backModules[i];
        }
    }

    @Override
    public void initialize() 
    {
        movementController.setSetpoint(0);
        movementController.setTolerance(0.5);
        currentAngle = 0;
    }

    @Override 
    public void execute()
    {
        currentAngle = -1 * (currentAngle - camera.getYaw());
        currentPosition = camera.getDistance();

        movementController.calculate(currentPosition);
        SwerveModuleState[] mo = {new SwerveModuleState(movementController.calculate(currentPosition),new Rotation2d(Units.degreesToRadians(currentAngle)))};

        SwerveDriveKinematics.desaturateWheelSpeeds(mo, Constants.Swerve.maxSpeed);
    
        int counter = 0; 
        for(SwerveModule i : swerveModules)
        {
            if(counter == 2)
            {
                mo[0] = new SwerveModuleState(
                    movementController.calculate(currentPosition), new Rotation2d(Units.degreesToRadians((currentAngle * 3) % 360)));
            }
            i.setDesiredState(mo[0], false);
        }
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
        if(movementController.atSetpoint() && currentAngle < -0.5 && currentAngle > 0.5)
        {
            return true;
        }
        return false;
    }
}
