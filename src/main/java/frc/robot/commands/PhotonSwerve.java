package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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
            swerveModules[i + 2] = backModules[i];
        }
    }

    @Override
    public void initialize() 
    {
        movementController.setSetpoint(0);
        movementController.setTolerance(0.5);
    }

    @Override 
    public void execute()
    {
        currentAngle = camera.getYaw();
        currentPosition = camera.getDistance();

        movementController.calculate(currentPosition);
        SwerveModuleState movement = new SwerveModuleState(movementController.calculate(currentPosition),
        new Rotation2d(Units.degreesToRadians(currentAngle)));

        for(SwerveModule i : swerveModules)
        {
            if(i.equals(2))
            {
                i.setDesiredState(movement, false);
            }
        }
    }

    @Override
    public void end(boolean isTrue)
    {
        
    }

    @Override
    public boolean isFinished()
    {
        if(movementController.atSetpoint() && currentAngle < -1 && currentAngle > 1)
        {
            return true;
        }
        return false;
    }
}
