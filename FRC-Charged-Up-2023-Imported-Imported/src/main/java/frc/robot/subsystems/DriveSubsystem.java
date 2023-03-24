// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveCommand;

public class DriveSubsystem extends SubsystemBase {
  public CANSparkMax fr, fl, br, bl,ARM;
  public RelativeEncoder flEnc, frEnc, blEnc, brEnc, arm;
  public double s = 0, rot = 0, a = 0.5;
  public DifferentialDrive diffDrive;
  public double x = 0, y = 0, t = 0, fl_0 = 0, fr_0 = 0, bl_0 = 0, br_0 = 0, dt = 0;
  public Timer time, auto;
  RelativeEncoder[] Encs;
  int drivePhase = 0;
  double p= 0.03;
  double i= 0.009;
  double d= 0.01;
  double e;
  public PIDController drivePIDController;
  public double TestDrive;
 
/*fdshfhdjshfhgtjvddsgsjfhdisjgdfgsdjglsfhdsjfshfsdjiotfds System.Out.Println("L") */
  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    fl = new CANSparkMax(Constants.FLCAN, MotorType.kBrushless);
    fr = new CANSparkMax(Constants.FRCAN, MotorType.kBrushless);
    //fr.setInverted(true);
    br = new CANSparkMax(Constants.BRCAN, MotorType.kBrushless);
    //br.setInverted(true);
    bl = new CANSparkMax(Constants.BLCAN, MotorType.kBrushless);
    Constants.gyro.calibrate();

    drivePIDController= new PIDController(p, i, d);
    //ARM= new CANSparkMax(Constants.ARMCAN, MotorType.kBrushless);
    //br.setInverted(true);
    //fr.setInverted(true);
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

    left.setInverted(true);

    

    flEnc = fl.getEncoder();
    frEnc = fr.getEncoder();
    blEnc = bl.getEncoder();
    brEnc = br.getEncoder();
    
    RelativeEncoder[] EncsA = {flEnc, frEnc, blEnc, brEnc};

    Encs = EncsA;

    for (RelativeEncoder enc : Encs) {
      enc.setPosition(0);
      //enc.setPositionConversionFactor(-1);
    }

    /*flEnc.setPositionConversionFactor(-1);
    flEnc.setVelocityConversionFactor(-1);
    blEnc.setPositionConversionFactor(-1);
    blEnc.setVelocityConversionFactor(-1);*/

    diffDrive= new DifferentialDrive(left, right);
    
    time = new Timer();
    time.start();

  }

  public boolean timeDrive(double time, double pow, boolean end) {
    if (auto == null) {
      auto = new Timer();
      auto.start();
      return false;
    } else if (auto.get() < time) {
      diffDrive.arcadeDrive(-pow, 0);
      return false;
    } else {
      stopDrive();
      if (!end) {auto = null;}
      return true;
    }
  }

  public boolean timedProfiledDrive(double time, double pow, boolean end) {
    if (auto == null) {
      auto = new Timer();
      auto.start();
      return false;
    } else if (auto.get() < time) {
      diffDrive.arcadeDrive(-pow * (1 - (auto.get() / (3 * time))), 0);
      return false;
    } else {
      stopDrive();
      if (!end) {auto = null;}
      return true;
    }
  }

  public boolean expProfiledDrive(double time, double pow, boolean end) {
    if (auto == null) {
      auto = new Timer();
      auto.start();
      return false;
    } else if (auto.get() < time) {
      diffDrive.arcadeDrive(-pow * 1.5 * (1 - (auto.get() / (3 * time))), 0);
      return false;
    } else {
      stopDrive();
      if (!end) {auto = null;}
      return true;
    }
  }





  public void resetPos() {
    x = 0; y = 0; t = 0;
    for (RelativeEncoder enc : Encs) {
      enc.setPosition(0);
      //enc.setPositionConversionFactor(-1);
    }
  }

  public void driveToPoint(double xf, double yf, double tf, double tol, double atol) {
    double dy = yf - getY();
    double dx = xf - getX();
    double ti = Math.atan(dy/dx);
    atol = Math.toRadians(atol);
    double e = ti - getT();
    radAngWrap(e);
    SmartDashboard.putNumber("DP", drivePhase);
    if (drivePhase == 0) {
      if (Math.abs(dy) > tol || Math.abs(dx) > tol || Math.abs(e) > atol) {
        diffDrive.arcadeDrive(clip(-0.5, 0.5, -Math.hypot(dx, dy)*Constants.KP_DRIVE_X), 
          clip(-0.15, 0.15, -e*Constants.KP_DRIVE_R));
        SmartDashboard.putNumber("X", Math.abs(dy));
      } else {
        stopDrive();
        SmartDashboard.putNumber("X", Math.abs(dy));
        drivePhase++;
      }
    } else if (drivePhase == 1) {
      if (rotateToPoint(tf, Math.toDegrees(atol))) {
        stopDrive();
        drivePhase = 0;
      }
    }
  }

  public double clip(double min, double max, double i) {
    return Math.min(min, Math.max(max, i));
  }

  public double radAngWrap(double a) {
    while (a < -Math.PI) {
      a += 2*Math.PI;
    }

    while (a > Math.PI) {
      a -= 2*Math.PI;
    }

    return a;
  }

  public boolean rotateToPoint(double setT, double tolerance) {
    setT = Math.toRadians(setT);
    tolerance = Math.toRadians(tolerance);
    e = setT - getT();
    e = radAngWrap(e);
    SmartDashboard.putNumber("E", e);
    if (Math.abs(e) > tolerance) {
      diffDrive.arcadeDrive(0, clip(-0.4, 0.4, e*Constants.KP_DRIVE_R));
      SmartDashboard.putNumber("A", e*Constants.KP_DRIVE_R);
      return false;
    } else {
      SmartDashboard.putNumber("A", 1);
      stopDrive();
      return true;
    }
  }

  public double lowPassFilter(double lpf, double f, double i) {
    return f*lpf + i*(1-lpf);
  }

  
  public void arcadeDrive(){
    double p = 1;
    if (Constants.LEFTJOY.getRawButton(2)) {
      p = 0.5;
    }
    diffDrive.arcadeDrive(p*Constants.LEFTJOY.getY(), p*Constants.LEFTJOY.getX());
    
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
    diffDrive.tankDrive(-Constants.LEFTJOY.getY(), -Constants.RIGHTJOY.getY());
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
  dt = time.get();
  double fl_1 = (lowPassFilter(1, flEnc.getPosition(), fl_0) - fl_0)/dt;
  double fr_1 = (lowPassFilter(1, frEnc.getPosition(), fr_0) - fr_0)/dt;
  double bl_1 = (lowPassFilter(1, blEnc.getPosition(), bl_0) - bl_0)/dt;
  double br_1 = (lowPassFilter(1, brEnc.getPosition(), br_0) - br_0)/dt;

  // SmartDashboard.putNumber("FL", flEnc.getPosition());
  // SmartDashboard.putNumber("FR", frEnc.getPosition());
  // SmartDashboard.putNumber("BL", blEnc.getPosition());
  // SmartDashboard.putNumber("BR", brEnc.getPosition());

  double x_dot = Constants.DRIVE_InTk*((fl_1 + bl_1) - (br_1 + fr_1)) / (4);
  //double y_dot = 0; //Constants.DRIVE_R*(fl_1 - bl_1 + br_1 - fr_1) / 4;
  double a_dot = Constants.DRIVE_InTk*((bl_1 + fl_1) + (fr_1 + br_1)) / (2*Constants.DRIVE_TRACT);

  // SmartDashboard.putNumber("Xd", x_dot);
  // SmartDashboard.putNumber("Ad", a_dot);

  t += a_dot*dt;

  t = radAngWrap(t);

  double xd = x_dot*Math.cos(t);
  double yd = x_dot*Math.sin(t);

  x_dot = xd;
  double y_dot = yd;

  x += x_dot*dt;
  y += y_dot*dt;

  fl_0 = flEnc.getPosition();
  fr_0 = frEnc.getPosition();
  bl_0 = blEnc.getPosition();
  br_0 = brEnc.getPosition();
  time.restart();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setDefaultCommand(new DriveCommand(this));
    stateUpdater();
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

  public double getTDeg() {
    return Math.toDegrees(t);
}

public void jadenSmithPlus(double target){
  // double kP = 0.1;
  // double e = target - Constants.gyro.getYaw();
  // if(Math.abs(e) > 5){ 
  //   diffDrive.arcadeDrive(0, kP*e);
  // } else {
  //   diffDrive.arcadeDrive(0, 0);
  // }

 TestDrive = MathUtil.clamp(drivePIDController.calculate(Constants.gyro.getYaw(), target), -0.5, 0.5);
  diffDrive.arcadeDrive(0, TestDrive);
  
}

public double getDriveError(){
  return e;
}

public double getTestDrive(){
  return TestDrive;
}


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
