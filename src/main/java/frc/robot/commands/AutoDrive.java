// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDrive extends CommandBase {
  DriveSubsystem drive;
  Timer timer;
  double drive_time;

  /** Creates a new AutoDrive. */
  public AutoDrive(DriveSubsystem driveSub, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSub);
    drive = driveSub;
    drive_time = time;
    timer = new Timer();
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.tankDrive(.25,.25);
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
    return timer.hasElapsed(drive_time);
  }
}
