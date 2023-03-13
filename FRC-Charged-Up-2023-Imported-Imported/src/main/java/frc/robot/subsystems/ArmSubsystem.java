package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.ArmCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm;
  private CANSparkMax extend;
  private RelativeEncoder m_encoder;

  public ArmSubsystem() {
    arm = new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    extend= new CANSparkMax(Constants.EXTENDCAN, MotorType.kBrushless);
    // arm.setInverted(true);
    m_encoder = arm.getEncoder(); 
    m_encoder.setPosition(0);
}

  public void armController(){
    if(Constants.LEFTJOY.getTrigger()){
      arm.set(0.30);
    }
    else if(Constants.RIGHTJOY.getTrigger()){
      arm.set(-0.30);
    } else {
      arm.set(0);
    }
  }

  public void extendController(){
    extend.set(Constants.RIGHTJOY.getY());
  }

  public double getDegPos() {
    return m_encoder.getPosition()*(360/42);
  }

  public void perfect90(){
    if(Constants.LEFTJOY.getRawButton(7)){
      if(getDegPos()>50){
        arm.set(-.1);
      } else if( getDegPos()<40){
        arm.set(.4);
      } else{
        arm.set(0.2*Math.sin(getDegPos()*Math.PI/180));
      }
    }
  }



  @Override
  public void periodic() {  
   setDefaultCommand(new ArmCommand(this));
   System.out.println(m_encoder.getPosition());
   System.out.println(m_encoder.getVelocity());
  }


  @Override
  public void simulationPeriodic() {}

}