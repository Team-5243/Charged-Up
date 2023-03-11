// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax fr, fl, br, bl,ARM;
  public RelativeEncoder flEnc, frEnc, blEnc, brEnc, arm;
  public double s = 0, rot = 0, a = 0.5;
  public DifferentialDrive diffDrive;
  public double x = 0, y = 0, t = 0, fl_0 = 0, fr_0 = 0, bl_0 = 0, br_0 = 0, dt = 0;
  public Timer time;
/*fdshfhdjshfhgtjvddsgsjfhdisjgdfgsdjglsfhdsjfshfsdjiotfds System.Out.Println("L") */
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    fl = new CANSparkMax(Constants.FLCAN, MotorType.kBrushless);
    fl.setInverted(true);
    fr = new CANSparkMax(Constants.FRCAN, MotorType.kBrushless);
    //fr.setInverted(true);
    br = new CANSparkMax(Constants.BRCAN, MotorType.kBrushless);
    //br.setInverted(true);
    bl = new CANSparkMax(Constants.BLCAN, MotorType.kBrushless);
    //ARM= new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    bl.setInverted(true);
    //ARM.set(0);
    

    // fl = new PWMSparkMax(Constants.FLCAN);
    // //fl.setInverted(false);
    // fr = new PWMSparkMax(Constants.FRCAN);
    // //fr.setInverted(false);
    // br = new PWMSparkMax(Constants.BRCAN);
    // //br.setInverted(false);
    // bl = new PWMSparkMax(Constants.BLCAN);

    MotorControllerGroup left= new MotorControllerGroup(fl, bl);
    MotorControllerGroup right= new MotorControllerGroup(fr, br);    

    flEnc = fl.getEncoder();
    frEnc = fr.getEncoder();
    blEnc = bl.getEncoder();
    brEnc = br.getEncoder();    

    diffDrive= new DifferentialDrive(left, right);
    
    time = new Timer();
  }

  public double lowPassFilter(double lpf, double f, double i) {
    return f*lpf + i*(1-lpf);
  }

  
  public void arcadeDrive(){
    diffDrive.arcadeDrive(Constants.LEFTJOY.getY(),Constants.LEFTJOY.getX());
    
    //fr.set(Constants.LEFTJOY.getY());
  }
  /*public void armMoveBetter(){
    if(Constants.LEFTJOY.getTrigger()){
      ARM.set(0.1);
    }
    else if(Constants.RIGHTJOY.getTrigger()){
      ARM.set(-0.1);
    } else{
      ARM.set(0);
    }
  }*/
  

  public void LParcadeDrive(){
    diffDrive.arcadeDrive(
      -a*Constants.LEFTJOY.getX() + (a-1)*rot, 
      a*Constants.LEFTJOY.getY() + (a-1)*s);
      s = Constants.LEFTJOY.getY();
      rot = Constants.LEFTJOY.getX();
  }

  public void tankDrive(){
    diffDrive.tankDrive(-Constants.LEFTJOY.getY(), -Constants.LEFTJOY.getX());
  }

  public void leftPivot(){
    if(Constants.LEFTJOY.getTrigger()){
        br.set(1);
        bl.set(0.6);
        fr.set(1);
        fl.set(0.6);
    }
  }

  public boolean encDrive(double dist, double pow) {
    if (x < dist) {
      setMotors(pow, pow);
      return false;
    } else {
      stopDrive();
      return true;
    }
  }

  public void setMotors(double left, double right) {
    diffDrive.tankDrive(left, right);
  }

  public void stopDrive() {
    diffDrive.arcadeDrive(0, 0);
}

public void stateUpdater() {
  dt = 0.01; //time.get();
  double fl_1 = (lowPassFilter(1, flEnc.getPosition(), fl_0) - fl_0)/dt;
  double fr_1 = (lowPassFilter(1, frEnc.getPosition(), fr_0) - fr_0)/dt;
  double bl_1 = (lowPassFilter(1, blEnc.getPosition(), bl_0) - bl_0)/dt;
  double br_1 = (lowPassFilter(1, brEnc.getPosition(), br_0) - br_0)/dt;

  double x_dot = 2*3.14*Constants.DRIVE_R*(fl_1 + bl_1 + br_1 + fr_1) / 4;
  double y_dot = 0; //Constants.DRIVE_R*(fl_1 - bl_1 + br_1 - fr_1) / 4;
  double a_dot = Constants.DRIVE_R*(bl_1 + fl_1 - fr_1 - br_1) / (3.14 * Constants.DRIVE_TRACT);

  t += a_dot*dt;

  double xd = x_dot*Math.cos(t) - y_dot*Math.sin(t);
  double yd = x_dot*Math.sin(t) + y_dot*Math.cos(t);

  x_dot = xd;
  y_dot = yd;

  x += x_dot*dt;
  y += y_dot*dt;

  fl_0 += fl_1*dt;
  fr_0 += fr_1*dt;
  bl_0 += bl_1*dt;
  br_0 += br_1*dt;
  time.reset();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new DriveCommand(this));
    //stateUpdater();
  }

  public double getX() {
      return x;
  }

  public double getY() {
      return y;
  }

  public double getT() {
      return t;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
