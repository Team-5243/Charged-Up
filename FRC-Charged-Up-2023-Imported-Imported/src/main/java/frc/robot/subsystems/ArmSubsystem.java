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
  private RelativeEncoder m_ArmEncoder;
private RelativeEncoder m_ExtendEncoder;
  double targetPosARM = 0;
  double targetPosEXTEND = 0;

  public ArmSubsystem() {
    arm = new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    extend= new CANSparkMax(Constants.EXTENDCAN, MotorType.kBrushless);
     extend.setInverted(true);
    m_ArmEncoder = arm.getEncoder(); 
    m_ArmEncoder.setPosition(0);

    m_ExtendEncoder= extend.getEncoder();
    m_ExtendEncoder.setPosition(0);
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

  public double getDegPos(RelativeEncoder Enc) {
    return Enc.getPosition()*(360/42);
  }

  public void perfect45(){
    if(Constants.LEFTJOY.getRawButton(7)){
      if(getDegPos(m_ArmEncoder)>50){
        arm.set(-.1);
      } else if( getDegPos(m_ArmEncoder)<40){
        arm.set(.4);
      } else{
        arm.set(0.2*Math.sin(getDegPos(m_ArmEncoder)*Math.PI/180));
      }
    }
  }

  public void PIDArm(double degPos, double tolerance) {
    double e = degPos - getDegPos(m_ArmEncoder);
    if (Math.abs(e) < tolerance) {
      arm.set(Constants.KP_ARM*e + Constants.KS_ARM*Math.sin(Math.toRadians(getDegPos(m_ArmEncoder))));
    }
  }


  public void PIDArmController() {   
    /*if (Constants.LEFTJOY.getTrigger()) {
      targetPosARM += Constants.ARM_CONTROL_SCALAR; 
    }
    if (Constants.RIGHTJOY.getTrigger()) {
      targetPosARM -= Constants.ARM_CONTROL_SCALAR; 
    }*/
    targetPosARM = Math.max(5, Math.min((1 -Constants.RIGHTJOY.getRawAxis(0))/2, Constants.ARM_LIMIT));
    PIDArm(targetPosARM, Constants.ARM_DEG_TOL);
  }

  public void PIDExtendController() {
    targetPosEXTEND += Constants.RIGHTJOY.getY()*Constants.EXTEND_CONTROL_SCALAR;
    targetPosEXTEND = Math.max(0, Math.min(targetPosEXTEND, Constants.EXTENT_LIMIT));
    PIDExtend(targetPosEXTEND, Constants.EXTEND_DEG_TOL);
  }

  public void PIDExtend(double degPos, double tolerance) {
    double e = degPos - getDegPos(m_ExtendEncoder);
    if (Math.abs(e) < tolerance) {
      arm.set(Constants.KP_EXTEND*e /*+ Constants.KS_EXTEND*/);
    }
  }

  @Override
  public void periodic() {  
   setDefaultCommand(new ArmCommand(this));
  }


  @Override
  public void simulationPeriodic() {}

  public double getArmDegPos() {return getDegPos(m_ArmEncoder);}

  public double getExtendDegPos() {return getDegPos(m_ExtendEncoder);}

}