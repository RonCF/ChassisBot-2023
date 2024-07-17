// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ShapeAuto extends CommandBase {
  DriveSubsystem drive;
  Timer timer;

  int n;
  int current_side = 0;
  int angle;

  double speed;

  /** Creates a new ShapeAuto. */
  public ShapeAuto(DriveSubsystem driveSub, int sides, double drive_speed) {
    addRequirements(driveSub);
    this.drive = driveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    n = sides;
    angle =  360/n;

    timer = new Timer();

    speed = drive_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.gyro.reset();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(current_side < n){
      if(timer.get() >= 1){
        double turn_sped = Math.min(1, 2*speed);

        drive.arcadeDrive(0, turn_sped);

        if(Math.abs(drive.gyro.getYaw()) >= angle){
          drive.tankDrive(0, 0);
          timer.reset();
          current_side++;
          drive.gyro.reset();
        }
      }
      else{
        if(drive.gyro.getYaw() > 0){
          drive.tankDrive(speed / 2 ,speed);
        }
        else if(drive.gyro.getYaw() > 0){
          drive.tankDrive(speed, speed / 2);
        }
        else{
          drive.tankDrive(speed, speed);
        }
      }

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.tankDrive(0, 0);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return current_side == n;
  }
}
