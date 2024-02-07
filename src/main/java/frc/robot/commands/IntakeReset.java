package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.State.*;

public class IntakeReset extends CommandBase {
    // private Swerve s_Swerve;
    private ElevatorSubsystem s_ElevatorSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;

    public IntakeReset(/*Swerve swerve,*/ ElevatorSubsystem elevator, IntakeSubsystem intake) {
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
        //s_ElevatorSubsystem.goHState(HState.LOAD_CONE);
        //s_ElevatorSubsystem.goVState(VState.HOME);
        s_IntakeSubsystem.goFState(FState.FLIPPED);
        //s_IntakeSubsystem.goSState(SState.OUT);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
