// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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
    exampleDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 1);
    exampleDoublePCM.set(Value.kOff);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Penumatics");
    builder.addDoubleProperty("Actuations", this::getCount, this::setCount);
  }

  public void extend(){
    if(Constants.LEFTJOY.getRawButtonPressed(2)) {
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