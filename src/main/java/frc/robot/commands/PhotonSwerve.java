package frc.robot.commands;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.sim.ChassisReference;
import com.fasterxml.jackson.core.TreeNode;

import frc.robot.Constants;
import frc.robot.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.geometry.Transform3d;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.photonSubsystem;


public class PhotonSwerve extends Command{
    photonSubsystem camera;
    Swerve s_Swerve;

    TrapezoidProfile.Constraints theataTrap = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
    ProfiledPIDController theataPidController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, theataTrap);
    PIDController xPidController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    //PIDController theataPidController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    PIDController yPidController = new PIDController(Constants.AutoConstants.kPThetaController, 0, 0);
    Transform3d currentTransform3d;
    ChassisSpeeds chassisSpeeds;
    SwerveModuleState[] swerveModuleStates;

    double xCalculation;
    double yCalculation;
    double theataCaluclation;
    double angle;


    public PhotonSwerve(photonSubsystem camera, Swerve s_Swerve)
    {
        this.camera = camera;
        this.s_Swerve = s_Swerve;

        xPidController.setTolerance(0.1);
        yPidController.setTolerance(0.1);
        theataPidController.enableContinuousInput(-Math.PI, Math.PI);
        theataPidController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void initialize() 
    {
        theataPidController.reset(s_Swerve.getYaw().getRadians());

        xPidController.setSetpoint(1);
        yPidController.setSetpoint(0);
        s_Swerve.zeroGyro();
        theataPidController.setGoal(0);
    }

    @Override 
    public void execute()
    {
        currentTransform3d = camera.getTransform3d();
        angle = camera.getYaw(); 
        theataCaluclation = -theataPidController.calculate(s_Swerve.gyro.getYaw()  % 360- angle); //Units.degreesToRadians(angle - s_Swerve.getYaw().getDegrees())
        //if camera > gyro
        if((Math.abs(s_Swerve.gyro.getYaw()) - Math.abs(angle)) < 0){ //Added Math.abs to both so neither are impacted by negatives
            theataCaluclation = -theataPidController.calculate(-(s_Swerve.gyro.getYaw() % 360 - angle));
        }

        xCalculation = -xPidController.calculate(currentTransform3d.getX());
        yCalculation = -yPidController.calculate(currentTransform3d.getY()); 
        chassisSpeeds = new ChassisSpeeds(xCalculation,yCalculation,theataCaluclation);
        swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.AutoConstants.kMaxSpeedMetersPerSecond);
        s_Swerve.setModuleStates(swerveModuleStates);
    }

    @Override
    public void end(boolean isTrue)
    {
        SwerveModuleState[] stop = new SwerveModuleState[4];
        for(int i = 0; i < stop.length; i++)
        {
            stop[i] = new SwerveModuleState();
        }
        s_Swerve.setModuleStates(stop);
    }

    @Override
    public boolean isFinished()
    {
        if(yPidController.atSetpoint())
        {
            yPidController.reset();
        }
        if(xPidController.atSetpoint())
        {
            xPidController.reset();
        }
        // if(theataPidController.atSetpoint())
        // {
        //     theataPidController.reset();
        // }
        return false;
    }
}
