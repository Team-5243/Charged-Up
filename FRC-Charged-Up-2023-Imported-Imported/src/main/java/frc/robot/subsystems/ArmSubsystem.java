package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.ArmCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm;
  private RelativeEncoder m_encoder;

  public ArmSubsystem() {
    arm = new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    // arm.setInverted(true);
    m_encoder = arm.getEncoder(); 
}

  public void armController(){
    if(Constants.LEFTJOY.getTrigger()){
      arm.set(0.1);
    }
    else if(Constants.RIGHTJOY.getTrigger()){
      arm.set(-0.1);
    } else {
      arm.set(0);
    }
  }

  public double getDegPos() {
    return m_encoder.getPosition() * 360.0;
  }


  @Override
  public void periodic() {  
   setDefaultCommand(new ArmCommand(this));
  }


  @Override
  public void simulationPeriodic() {}

}