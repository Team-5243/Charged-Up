// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  public double drive90Help= Constants.gyro.getYaw();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.resetPos(); 
    Constants.gyro.zeroYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Constants.LEFTJOY.getRawButton(6)) {
      m_subsystem.driveToPoint(12*10, 0, 0, 6, 5);
    // } else if(Constants.LEFTJOY.getRawButtonPressed(11)){
    //     while(m_subsystem.getDriveError()>5){
    //       m_subsystem.jadenSmithPlus(drive90Help+90);
    //     }
    // }else if(Constants.LEFTJOY.getRawButtonReleased(11)){
    //    drive90Help+=90;
    //    if(drive90Help>=360){
    //     drive90Help-=360;
    //   } else if(drive90Help<=0){
    //     drive90Help+=360;
    //   }
    } else {
      if(Constants.LEFTJOY.getRawButtonPressed(11)){
        drive90Help+=90;
        if((drive90Help)>=180){
          drive90Help*=-1;
          drive90Help+=180;
        }
      } else if(Constants.LEFTJOY.getRawButton(11)){
        m_subsystem.jadenSmithPlus(drive90Help);
      } else{
        m_subsystem.arcadeDrive(); 
      }
    }

      SmartDashboard.putString("yaw", Constants.gyro.getYaw()+"");
      SmartDashboard.putString("pitch", Constants.gyro.getPitch()+"");
      SmartDashboard.putString("roll", Constants.gyro.getRoll()+"");
      SmartDashboard.putString("error", m_subsystem.getDriveError()+"");
      SmartDashboard.putString("targetAngleDrive", drive90Help+"");
      SmartDashboard.putString("targetDrive", m_subsystem.getTestDrive()+"");
      
      // }
    //m_subsystem.armMoveBetter();
    //m_subsystem.leftPivot();
    // SmartDashboard.putNumber("X", m_subsystem.getX());
    // SmartDashboard.putNumber("Y", m_subsystem.getY());
    // SmartDashboard.putNumber("T", m_subsystem.getTDeg());
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
