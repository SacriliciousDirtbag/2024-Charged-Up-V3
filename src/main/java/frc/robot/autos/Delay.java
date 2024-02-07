package frc.robot.autos;

import java.util.List;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

public class Delay extends CommandBase {
    public Timer clock;
    double start;
    double current;
    double seconds;

    public Delay (double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void initialize() {
        start = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        current = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        if(current < start + seconds)
        {
            return false;
        } 
        return true;
    }

}
