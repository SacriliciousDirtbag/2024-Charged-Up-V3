// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class NewAutoBalance extends CommandBase {
  private Swerve swerve;
  double roll;
  double error;
  /** Creates a new NewAutoBalance. */
  public NewAutoBalance(Swerve swerve) {
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    roll = swerve.gyro.getRoll();
    error = frc.robot.Constants.Swerve.balanceGoal - roll;

    double drivePower = -Math.min( frc.robot.Constants.Swerve.balanceKp * error, 1);

    if(Math.abs(drivePower) > 0.5) {
      drivePower = Math.copySign(0.5, drivePower);
    }

    SmartDashboard.putNumber("Drive power", drivePower);

    swerve.drive(new Translation2d(drivePower, 0), 0, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0, 0), 0, true, true);
    swerve.drive(new Rotation2d(), 45, true, true);  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) < frc.robot.Constants.Swerve.balanceTolerance;
  }
}
