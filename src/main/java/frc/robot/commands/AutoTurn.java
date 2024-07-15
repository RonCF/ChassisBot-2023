// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTurn extends CommandBase {
  DriveSubsystem drive;

  double turn_angle;
  boolean turn_direction;

  /** Creates a new AutoTurn. */
  public AutoTurn(DriveSubsystem driveSub, double angle, boolean direction) {
    addRequirements(driveSub);
    drive = driveSub;
    
    turn_angle = angle;
    turn_direction = direction; //true = clockwise false = counterclockwise
    
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
    if(turn_direction){
      drive.tankDrive(-0.25, 0.25);
    }
    else{
      drive.tankDrive(0.25, -0.25);
    }

    if(isFinished()){
      end(false);
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
    return Math.abs(drive.gyro.getYaw()) >= turn_angle - 1;
  }
}
