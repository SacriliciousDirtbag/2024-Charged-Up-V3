package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {

        

        public static final int pigeonID = 13;

        //BALANCE VALUES
        public static final double balanceKp = 0.5;
        public static final double balanceGoal = 8.25;
        public static final double balanceTolerance = 10;
        

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L1);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.5);
        public static final double wheelBase = Units.inchesToMeters(24.5);
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.2; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.01;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.3; //was 4
        /** Radians per Second */
        public static final double maxAngularVelocity = 2.5; //was 10

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
         public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(243.63); /*197.22 */
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(152.57); /*266.83 */
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(93.60); /*303.52 */
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(44.30); /*161.58, was 28.30 */
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profiled robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    
    public static final class Elevator{

        public static final class ElevatorRise {
            public static final int ElevatorMotorID = 15; //Talon
            //Uncomment if any other motors are to be called for this system
            //public static final int angleMotorID = ___;
            //public static final int canCoderID = ___;
        }

        public static final class ElevatorExtend {
            public static final int extendMotorID = 16; //Falcon
            //Uncomment if any other motors are to be called for this system
            //public static final int angleMotorID = ____;
            //public static final int canCoderID = ____;

        }

        /*public static final int forwardMotorID = 0;
        public static final int reverseMotorID = 0;
        public static final int extendMotorID = 0;
        public static final int riseMotorID = 0;*/

    }

    public static final class IntakeSystem {
        public static final class IntakeFlip {
            public static final int flipMotorID = 17; //Talon
            //Uncomment if any other motors are to be called for this system
            //public static final int angleMotorID = ____;
            //public static final int canCoderID = ____;
        }

        public static final class IntakeWheel {
            public static final int inMotorID = 18; //Talon
            //Uncomment if any other motors are to be called for this system
            //public static final int angleMotorID = ____;
            //public static final int canCoderID = ____;
        } 
    } 






    public static final class autoConfigs {
        //Adjustment
        public static final Translation2d[] backupLocations = {new Translation2d(-1,0.1)};
        public static final Pose2d backupEndLocation = new Pose2d(-2, 0.1, Rotation2d.fromDegrees(0)); //y was 0 

        //Route1 (Near 2nd Cube)
        public static final Translation2d[] route1Locations = {new Translation2d(-2,0)};
        //                                                                                          DO NOT CHANGE 
        public static final Pose2d route1EndLocation = new Pose2d(-2.3, 0.1, Rotation2d.fromDegrees(179.8)); //y was 180 //gyro offset 182

          //Pickup cube (move towards cube)
          public static final Translation2d[] pickupcubeLocations = {new Translation2d(-3.5,0.1)};
          public static final Pose2d pickupcubeLocation = new Pose2d(-2.5, 0.15, Rotation2d.fromDegrees(0));

        //Route2 (Scoring Hub)
        public static final Translation2d[] route2Locations = {new Translation2d(-4.8,0.1)};
        public static final Pose2d route2EndLocation = new Pose2d(0, 0.1, Rotation2d.fromDegrees(180));

        //Route3 (Balance)
        public static final Translation2d[] route3Locations = {new Translation2d(2.5, 0.15), new Translation2d(2.6, 0.1), new Translation2d(2.6, 1), new Translation2d(2.6, 2)};
        public static final Pose2d route3EndLocation = new Pose2d(0.3, 2, Rotation2d.fromDegrees(0));
    }
}

