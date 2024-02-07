package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;


public class Pose2dMovement extends CommandBase {
    private SwerveModule[] mSwerveMods;
    private Translation2d translation;
    private Swerve s_Swerve;
    private boolean fieldRelative;
    private double rotation;
    private boolean isOpenLoop;
    private Pose2d startPose; 
    private Pose2d currentPose; 
    private Translation2d currentTranslation;

    public Pose2dMovement(Swerve s_Swerve, Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        this.startPose = new Pose2d();
        this.s_Swerve = s_Swerve;
        this.translation = translation;
        this.rotation = rotation;
        this.fieldRelative = fieldRelative;
        this.isOpenLoop = isOpenLoop;
        this.startPose = s_Swerve.getPose();
    }

    @Override
    public void initialize() {
        s_Swerve.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    @Override
    public void execute() {
        currentPose = s_Swerve.getPose();
        currentTranslation = new Translation2d(currentPose.getX(), currentPose.getY());
    }

    @Override
    public void end(boolean isFinished) {
        for(SwerveModule swerveModule : mSwerveMods)
        {
            swerveModule.setDesiredState(new SwerveModuleState(), false);
        }
    }

    @Override
    public boolean isFinished() {
        if(translation.equals(currentTranslation)) {
            return true;
        } else {
            return false;
        }
    }
}    


