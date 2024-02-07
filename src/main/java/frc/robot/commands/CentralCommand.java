package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.State.FState;
import frc.robot.State.HState;
import frc.robot.State.SState;
import frc.robot.State.VState;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//import java.util.function.BooleanSupplier;
//import java.util.function.DoubleSupplier;
//import edu.wpi.first.wpilibj.Timer; Needs Implementation

public class CentralCommand extends Command {
  //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

   
    //private Timer m_Timer; /* TODO: use later, possibly */

/**
* @param subsystem The subsystem used by this command.
*/
  public ElevatorSubsystem s_ELEVATORSUBSYSTEM;
  public IntakeSubsystem s_INTAKESUBSYSTEM;


  /*public CentralCommand(
    ElevatorSubsystem p_ELEVATORSUBSYSTEM, BooleanSupplier p_toggleExtend, BooleanSupplier p_toggleLower, BooleanSupplier p_toggleRise, BooleanSupplier p_toggleRetract,
    IntakeSubsystem p_INTAKESUBSYSTEM, BooleanSupplier p_toggleFlipOut, BooleanSupplier p_toggleFlipIn, BooleanSupplier p_toggleWheelForward, BooleanSupplier p_toggleWheelReverse) {
      
      s_ESubsystem = p_ELEVATORSUBSYSTEM;
      m_toggleExtend = p_toggleExtend;
      m_toggleLower = p_toggleLower;
      m_toggleRise = p_toggleRise;
      m_toggleRetract = p_toggleRetract;

      s_IntakeSubsystem = p_INTAKESUBSYSTEM;
      m_toggleFlipIn = p_toggleFlipIn;
      m_toggleFlipOut = p_toggleFlipOut;
      m_toggleWheelForward = p_toggleWheelForward;
      m_toggleWheelReverse = p_toggleWheelReverse;
      

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(p_ELEVATORSUBSYSTEM);
      addRequirements(p_INTAKESUBSYSTEM);
    }*/


  @Override
  public void initialize(){
    //s_ELEVATORSUBSYSTEM.goVState(VState.HOME);
    //s_ELEVATORSUBSYSTEM.goHState(HState.HOME);
    //s_INTAKESUBSYSTEM.goFState(FState.HOME);
  }

  @Override
  public void end(boolean isFinished){

  }
}