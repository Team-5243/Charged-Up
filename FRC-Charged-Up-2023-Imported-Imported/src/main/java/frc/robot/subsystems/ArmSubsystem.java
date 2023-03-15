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
  double targetPos = 0;

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

  public void perfect45(){
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

  public void PIDArm(double degPos, double tolerance) {
    double e = degPos - getDegPos();
    if (Math.abs(e) < tolerance) {
      arm.set(Constants.KP_ARM*e + Constants.KS_ARM*Math.sin(Math.toRadians(getDegPos())));
    }
  }

  public void PIDArmController() {
    targetPos += Constants.RIGHTJOY.getY()*Constants.ARM_CONTROL_SCALAR;
    targetPos = Math.max(5, Math.min(targetPos, 85));
    PIDArm(targetPos, Constants.ARM_DEG_TOL);
  }

  @Override
  public void periodic() {  
   setDefaultCommand(new ArmCommand(this));
  }


  @Override
  public void simulationPeriodic() {}

}