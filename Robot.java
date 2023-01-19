
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

  Joystick joystick = new Joystick(1);

  TalonFX fldrive = new TalonFX(4);
  TalonFX frdrive = new TalonFX(6);
  TalonFX bldrive = new TalonFX(1);
  TalonFX brdrive = new TalonFX(2);

  TalonFX flsteer = new TalonFX(0);
  TalonFX frsteer = new TalonFX(5);
  TalonFX blsteer = new TalonFX(7);
  TalonFX brsteer = new TalonFX(3);

  CANCoder flcancoder = new CANCoder(0); 
  CANCoder frcancoder = new CANCoder(0); 
  CANCoder blcancoder = new CANCoder(0); 
  CANCoder brcancoder = new CANCoder(0); 

CANCoderConfiguration config = new CANCoderConfiguration();

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    fldrive.set(ControlMode.PercentOutput, joystick.getRawAxis(1));
    frdrive.set(ControlMode.PercentOutput, joystick.getRawAxis(1));
    bldrive.set(ControlMode.PercentOutput, joystick.getRawAxis(1));
    brdrive.set(ControlMode.PercentOutput, joystick.getRawAxis(1));

    flsteer.set(ControlMode.PercentOutput, joystick.getRawAxis(4));
    frsteer.set(ControlMode.PercentOutput, joystick.getRawAxis(4));
    blsteer.set(ControlMode.PercentOutput, joystick.getRawAxis(4));
    brsteer.set(ControlMode.PercentOutput, joystick.getRawAxis(4));

    double flPosition = flcancoder.getPosition();
    double frPosition = frcancoder.getPosition();
    double blPosition = blcancoder.getPosition();
    double brPosition = brcancoder.getPosition();
    
    System.out.println("FL Position: " + flPosition);
    System.out.println("FR Position: " + frPosition);
    System.out.println("BL Position: " + blPosition);
    System.out.println("BR Position: " + brPosition);
    
    // if(joystick.getRawButton(1)){
    //   frdrive.set(ControlMode.PercentOutput, 0.50);
    // } else {
    //   frdrive.set(ControlMode.PercentOutput, 0.0);
    // }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
