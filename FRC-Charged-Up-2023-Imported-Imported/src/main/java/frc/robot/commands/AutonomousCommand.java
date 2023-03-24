// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutonomousCommand extends CommandBase {
  DriveSubsystem m_subsystem;
  PneumaticsSubsystem m_pneu;
  double step = 0;
  /** Creates a new DriveRotate. */
  public AutonomousCommand(DriveSubsystem subsystem, PneumaticsSubsystem pneu) {
    m_subsystem = subsystem;
    m_pneu = pneu;
    
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.driveToPoint(10*12, 0, 0, 5, 5);    
    if (step == 0 && m_subsystem.timeDrive(0.5, -0.48, false)) {
      step++;
    } else if (step == 1 && m_subsystem.timedProfiledDrive(4.2, 0.48, true)) { // Time 3.1 or 6.2
      step++;
    } else if (step == 2) {
      if(Constants.gyro.getPitch()>=70){
        step++;
      }
       m_subsystem.diffDrive.arcadeDrive(0.75, 0);  
    } else if(step==3){
      if(Constants.gyro.getPitch()<=60){
        m_subsystem.stopDrive();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
