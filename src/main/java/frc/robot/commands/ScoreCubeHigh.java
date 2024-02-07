package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.State.*;

public class ScoreCubeHigh extends CommandBase {
    // private Swerve s_Swerve;
    private ElevatorSubsystem s_ElevatorSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;

    public ScoreCubeHigh(/*Swerve swerve,*/ ElevatorSubsystem elevator, IntakeSubsystem intake) {
        // s_Swerve = swerve;
        s_ElevatorSubsystem = elevator;
        s_IntakeSubsystem = intake;

        // addRequirements(swerve);
        //addRequirements(elevator);
        //addRequirements(intake);
    }

    @Override
    public void initialize() {
        // s_Swerve.?
        //s_IntakeSubsystem.goSState(SState.STOP);
        //s_ElevatorSubsystem.goVState(VState.SCORE_CUBE_HIGH);
        //s_ElevatorSubsystem.goHState(HState.SCORE_CUBE_HIGH);
        s_IntakeSubsystem.goFState(FState.SCORE_CUBE_HIGH);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}