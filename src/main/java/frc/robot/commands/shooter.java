package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreConeLow;
import frc.robot.commands.RampCone;

import frc.robot.commands.IntakeCube;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.ScoreCubeLow;
import frc.robot.commands.RampCube;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.LEDSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class shooter extends Command {
    Swerve s_Swerve;
    ElevatorSubsystem s_ElevatorSubsystem;
    IntakeSubsystem s_IntakeSubsystem;
    LEDSubsystem s_lightSubsystem;
    RobotContainer s_RobotContainer;

    IntakeCone s_IntakeCone;
    IntakeCube s_IntakeCube;
    RampCone s_RampCone = s_RobotContainer.s_RampCone;
    RampCube s_RampCube;
    ScoreConeHigh s_ScoreConeHigh;
    ScoreConeLow s_ScoreConeLow;
    ScoreCubeHigh s_ScoreCubeHigh;

    public Timer m_Timer;

    public shooter(){
        m_Timer = new Timer();
        m_Timer.reset();
        m_Timer.start();

        new InstantCommand(() -> s_RampCone.initialize());
        
        /*while(Timer.getFPGATimestamp() != 2){
            new InstantCommand(() -> s_ElevatorSubsystem.carriageExtend()); //Nudge
            new InstantCommand(() -> s_RampCone.initialize()); //Arm Score High
            if(Timer.getFPGATimestamp() == 2){
                new InstantCommand(() -> s_IntakeSubsystem.wheelForward()); 
                break;
            }
        }*/

    }
}

