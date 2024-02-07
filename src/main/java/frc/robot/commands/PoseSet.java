package frc.robot.commands;
// The poses needed to implement team 1056's strategy
// Safe
// IntakeCube
// IntakeCone
// ScoreCubeLow
// ScoreConeLow
// ScoreCubeHigh
// ScoreConeHigh
// RampCone
// RampCube
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.Swerve;
import frc.robot.State.*;


public class PoseSet extends CommandBase {
    // private Swerve s_Swerve;
    private ElevatorSubsystem s_ElevatorSubsystem;
    private IntakeSubsystem s_IntakeSubsystem;

    public void Safe(/*Swerve swerve,*/ ElevatorSubsystem elevator, IntakeSubsystem intake) {
        // s_Swerve = swerve;
        s_ElevatorSubsystem = elevator;
        s_IntakeSubsystem = intake;

        // addRequirements(swerve);
        addRequirements(elevator);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // s_Swerve.?
        s_ElevatorSubsystem.goHState(HState.HOME);
        s_ElevatorSubsystem.goVState(VState.HOME);
        s_IntakeSubsystem.goFState(FState.HOME);
        s_IntakeSubsystem.goSState(SState.STOP);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
