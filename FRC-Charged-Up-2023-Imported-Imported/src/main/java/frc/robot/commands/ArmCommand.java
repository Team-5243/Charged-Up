// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  private final ArmSubsystem m_subsystem;

  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.EncZeroer();
    m_subsystem.setArmPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //m_subsystem.PIDArmController();
    m_subsystem.extendController();
    m_subsystem.armController();
    // m_subsystem.retractedReset();
    // m_subsystem.extendedReset();
    SmartDashboard.putNumber("Arm Degree Position", m_subsystem.getArmDegPos()*42./360);
    SmartDashboard.putNumber("Arm Target Pos", m_subsystem.getArmTarget());
    SmartDashboard.putNumber("Arm Position", m_subsystem.getArmPos());
    //SmartDashboard.putNumber("Extension Degree Position", m_subsystem.getExtendDegPos());
    SmartDashboard.putNumber("Extension Position", m_subsystem.getExtendPos());
    SmartDashboard.putNumber("Power Set to Arm", m_subsystem.getArmPower());
    SmartDashboard.putNumber("kP", Constants.KP_ARM);
    SmartDashboard.putNumber("kI", Constants.KI_ARM);
    SmartDashboard.putNumber("kD", Constants.KD_ARM);
    SmartDashboard.putNumber("Error", m_subsystem.getError());
    m_subsystem.livePIDTuner();
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
