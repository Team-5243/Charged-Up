package frc.robot.subsystems;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.commands.ArmCommand;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ArmSubsystem extends SubsystemBase {

  private CANSparkMax arm;
  private CANSparkMax extend;
  private RelativeEncoder m_ArmEncoder;
private RelativeEncoder m_ExtendEncoder;
  double targetPosARM = 0;
  double targetPosEXTEND = 0;
  //PIDController armPIDController;
  double armPosition = 0;
  SparkMaxPIDController armPIDController;
  public double EXT_LENGTH;

  public double p, i, d;

  public ArmSubsystem() {
    arm = new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    extend= new CANSparkMax(Constants.EXTENDCAN, MotorType.kBrushless);
    targetPosARM= 0;
    m_ArmEncoder = arm.getEncoder(); 
    m_ArmEncoder.setPosition(0);
    extend.setInverted(false);
    m_ExtendEncoder= extend.getEncoder();
    m_ExtendEncoder.setPosition(0);
   // armPIDController= new PIDController(Constants.KP_ARM, Constants.KI_ARM, Constants.KD_ARM);
    armPIDController= arm.getPIDController();
    armPIDController.setP(Constants.KP_ARM);
    armPIDController.setI(Constants.KI_ARM);
    armPIDController.setD(Constants.KD_ARM);
    armPIDController.setIZone(Constants.ARM_KIZ);
    armPIDController.setFF(Constants.ARM_KFF);
    armPIDController.setOutputRange(Constants.ARM_KMIN, Constants.ARM_KMAX);

    int smartMotionSlot = 0;
    armPIDController.setSmartMotionMinOutputVelocity(-5, smartMotionSlot);
    armPIDController.setSmartMotionMaxVelocity(400, smartMotionSlot); 
    armPIDController.setSmartMotionMaxAccel(3000,smartMotionSlot);
    armPIDController.setSmartMotionAllowedClosedLoopError(150, smartMotionSlot);
    armPIDController.setReference(armPosition, CANSparkMax.ControlType.kPosition, 0);
}

  public double getArmTarget() {
    return targetPosARM;
  }

  public void EncZeroer(){
    m_ArmEncoder.setPosition(0);
    m_ExtendEncoder.setPosition(0);
  }
  public void armController(){
    // if(0 <= getDegPos(m_ArmEncoder) && getDegPos(m_ArmEncoder) <= 90){
      if(Constants.LEFTJOY.getTrigger()){
        armPosition-=.05;
        armPIDController.setReference(armPosition, CANSparkMax.ControlType.kPosition, 0);
        //armPIDController.set
      } 
      else if(Constants.RIGHTJOY.getTrigger()){
        armPosition+=.05;
        armPIDController.setReference(armPosition, CANSparkMax.ControlType.kPosition, 0);
      }
      targetPosARM = armPosition;
      armPIDController.setReference(armPosition, CANSparkMax.ControlType.kPosition, 0);
  }

  public double getError() {
    return targetPosARM - m_ArmEncoder.getPosition();
  }
  public void setArmPosition(double target){
    armPosition=target;
  }

/* 
  public void verticalConstancy(){
    if(Constants.LEFTJOY.getRawButton(8)){
      

        if(extend.getEncoder()<){

        }
    }
    //armPosition += 0.5
  }*/

  public double verticalConstancyAngleHelp(double height){
    return Math.atan(height/23.0)+Math.atan(38.0/23.0);
  }

  public double verticalConstancyLengthHelp(double height){
    return Math.sqrt(23.0*23.0+height*height);
  }

  public double tickToDeg(double ticks){
    return ticks*(2*Math.PI);
  }

  // public void retractedReset() {
  //   if (Constants.RIGHTJOY.getRawButton(8)) {
  //     m_ExtendEncoder.setPosition(Constants.EXT_MIN*42/360);
  //   }
  // }

  // public void extendedReset() {
  //   if (Constants.RIGHTJOY.getRawButton(9)) {
  //     m_ExtendEncoder.setPosition(Constants.EXT_MIN*42/360);
  //   }
  // }

  public void extendController(){
    // if((-0.5*Constants.RIGHTJOY.getY()<0 && getExtendDegPos()<=Constants.EXT_MIN) || 
    //   (-0.5*Constants.RIGHTJOY.getY()>0 && getExtendDegPos()>=Constants.EXT_MAX)){
    //   extend.set(0);
    // } else{
      extend.set(-0.5*Constants.RIGHTJOY.getY());
    ///}
  }

  public double getDegPos(RelativeEncoder Enc) {
    return Enc.getPosition()*(360./42);
  }

  public double getArmPos(){
    return m_ArmEncoder.getPosition();
  }

  public void liftPause(){
    if(Constants.LEFTJOY.getRawButton(7)){
        arm.set(Constants.ARM_KS*Math.sin(getDegPos(m_ArmEncoder)*Math.PI/180)*getDegPos(m_ExtendEncoder)); 
    }
  }

  public double getArmPower(){
    return Constants.ARM_KS*Math.sin((getExtendDegPos()*Math.PI/180)+(Math.PI/4))*getDegPos(m_ExtendEncoder);
  }



  public void PIDArm(double degPos, double tolerance) {
    double e = degPos - getDegPos(m_ArmEncoder);
    if (Math.abs(e) < tolerance) {
      arm.set(Constants.KP_ARM*e + Constants.KS_ARM/*Math.sin(Math.toRadians(getDegPos(m_ArmEncoder)))*/);
      SmartDashboard.putNumber("Power Set to Arm", Constants.KP_ARM*e + Constants.KS_ARM);
      SmartDashboard.putNumber("Arm Power", arm.get());
    }
  }


  public void PIDArmController() {   
    /*if (Constants.LEFTJOY.getTrigger()) {
      targetPosARM += Constants.ARM_CONTROL_SCALAR; 
    }
    if (Constants.RIGHTJOY.getTrigger()) {
      targetPosARM -= Constants.ARM_CONTROL_SCALAR; 
    }*/
    targetPosARM = Math.max(5, Math.min((1 - Constants.RIGHTJOY.getRawAxis(0))/2, Constants.ARM_LIMIT));
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

  public double getExtendPos() {return m_ExtendEncoder.getPosition(); }

  public void livePIDTuner(){
    p= SmartDashboard.getNumber("kP", Constants.KP_ARM);
    i= SmartDashboard.getNumber("kI", Constants.KI_ARM);
    d= SmartDashboard.getNumber("kd", Constants.KD_ARM);

    if(p!=Constants.KP_ARM){
      armPIDController.setP(p);
    }
    if(i!=Constants.KI_ARM){
      armPIDController.setI(i);
    }
    if(d!=Constants.KD_ARM){
      armPIDController.setD(d);
    }
  }

}