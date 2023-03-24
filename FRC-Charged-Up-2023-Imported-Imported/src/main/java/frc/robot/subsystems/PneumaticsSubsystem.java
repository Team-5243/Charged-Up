// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.PneumaticsCommand;

public class PneumaticsSubsystem extends SubsystemBase {
  double count = 0;
  public Compressor pcmCompressor;
  public DoubleSolenoid exampleDoublePCM;
  public PneumaticsSubsystem() {
    pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    pcmCompressor.enableDigital();
    exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.PENUMATIC_IN, Constants.PENUMATIC_OUT);
    exampleDoublePCM.set(Value.kOff);
  }

  public void extend(){
    if(Constants.LEFTJOY.getRawButtonPressed(5)) {
      exampleDoublePCM.set(Value.kForward);
      count++;}
  }

  public void retract(){
    if(Constants.LEFTJOY.getRawButtonPressed(3)) {
      exampleDoublePCM.set(Value.kReverse);
      count++;}
  }
  public void toggle(){
    if(Constants.LEFTJOY.getRawButtonPressed(4)){
      exampleDoublePCM.toggle();
      count++;}
  }

  public void manageCounter() {
    if (pcmCompressor.isEnabled()) {
      count = 0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new PneumaticsCommand(this));
    //System.out.print("Actuations: " + count);
    manageCounter();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getCount() {
      return count;
  }

  public void setCount(double count) {
      this.count = count;
  }

}