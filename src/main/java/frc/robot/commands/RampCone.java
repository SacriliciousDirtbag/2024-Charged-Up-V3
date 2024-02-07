package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.State.*;

public class RampCone extends CommandBase {
    // private Swerve s_Swerve;
    private ElevatorSubsystem s_ElevatorSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;

    public RampCone(/*Swerve swerve,*/ ElevatorSubsystem elevator, IntakeSubsystem intake) {
        // s_Swerve = swerve;
        s_ElevatorSubsystem = elevator;
        s_IntakeSubsystem = intake;

        // addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // s_Swerve.?
        //s_IntakeSubsystem.goSState(SState.STOP);
        //s_ElevatorSubsystem.goVState(VState.RAMP_CONE);
        //s_ElevatorSubsystem.goHState(HState.RAMP_CONE);
        s_IntakeSubsystem.goFState(FState.RAMP_CONE);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}