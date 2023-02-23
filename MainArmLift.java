package frc.robot;

import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class MainArmLift {
  AnalogTrigger m_sensor; 
  Counter m_sensor_Count;
  VictorSP m_motor;

  double kP =.1135;
  double kI = 0;
  double kD =0.004;
  PIDController MALPID = new PIDController(kP, kI, kD);
  double MALPID_setpoint = 50;

  MainArmLift(int motor_port){
    m_sensor = new AnalogTrigger(3);
    m_sensor_Count = new Counter(m_sensor);
    //m_sensor_Count.setExternalDirectionMode();
    m_sensor.setLimitsVoltage(3.0, 3.9);
    m_motor = new VictorSP(motor_port);
  }    

  public double CalculateMALSpeed(){
    double speed=0;
    speed = MathUtil.clamp(MALPID.calculate(m_sensor_Count.get(), MALPID_setpoint),-0.5,0.5);
    return(speed);
  }

  public int ReadCount(){
    return(m_sensor_Count.get());
  }

  public void ResetCount(){
    m_sensor_Count.reset();
  }

  public double ReadRate(){
    return(m_sensor_Count.getRate());    
  }

  public void Move_ArmLift(int position, double speed){
    SmartDashboard.putNumber("MoveArmLift", position);
    if (speed > 0){
        m_sensor_Count.setReverseDirection(false);
    }
    else{
        m_sensor_Count.setReverseDirection(true);
    }
    SmartDashboard.putBoolean("Direction", m_sensor_Count.getDirection());
    m_motor.set(speed);
  }

  public void Stop_ArmLift(){
    m_motor.set(0);
  }
}
