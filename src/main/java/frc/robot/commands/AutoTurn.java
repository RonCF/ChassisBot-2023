// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurn extends CommandBase {
  DriveSubsystem drive;

  double turn_angle;

  /** Creates a new AutoTurn. */
  public AutoTurn(DriveSubsystem driveSub, double angle) {
    addRequirements(driveSub);
    drive = driveSub;
    
    turn_angle = angle;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.gyro.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(turn_angle > 0){
      drive.arcadeDrive(0, .3);
    }
    else{
      drive.arcadeDrive(0, -.3);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(drive.gyro.getYaw());
    return Math.round(Math.abs(drive.gyro.getRotation2d().getDegrees())) >= turn_angle;
  }
}
