// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.State.FState;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class HighCubeAuto extends CommandBase {

  private Timer tim = new Timer();
  private final IntakeSubsystem is;
  private final ElevatorSubsystem es;

  /** Creates a new HighCubeAuto. */
  public HighCubeAuto(IntakeSubsystem IS, ElevatorSubsystem ES) {
    is = IS;
    es = ES;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tim.reset();
    tim.start();
    is.goFState(FState.RAMP_CUBE);
  }

  private final double waitTime = 1;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double t = tim.get();
    if (t < waitTime) {
      es.m_RiseMotor.set(ControlMode.PercentOutput, 0.8);
    } else if (t > waitTime  && t < 2) {
      is.wheelReverse();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    is.goFState(FState.FLIPPED);
    is.resetAll();
    es.m_RiseMotor.set(ControlMode.PercentOutput, 0.25);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tim.get() > 2;
  }
}
