package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;

public class Climber {

  // Even though we used multiple double solenoids on the robot, what worked for me is instantiating
  // two Solenoid objects for each setting on the double solenoid instead of instantiating
  // a DoubleSolenoid object. Each side of the solenoid is either true or false.
  // Both sides of the solenoid cannot have the same value.

  // each solenoid setting has a Solenoid object
  Solenoid level2ClimberUp;
  Solenoid level2ClimberDown;

  Solenoid traverseClimberActivate;
  Solenoid traverseClimberDeactivate;

  // declaration and initialization of traverse climber motor
  CANSparkMax traverseClimberMotor = new CANSparkMax(8, MotorType.kBrushless);

  // useful but unnecessary true or false variables that keep track of solenoid settings
  boolean isLevel2ClimberDown = true;
  boolean isTraverseClimberActivated = false;

  public void climberInit() {
    // make solenoids in your init method like here. make them from a PneumaticHub object
    // as shown here. use makeSolenoid(), with the one argument being
    // the index found on the pneumatic hub on the robot, which is a positive integer.
    level2ClimberUp = Robot.ph.makeSolenoid(13);
    level2ClimberDown = Robot.ph.makeSolenoid(11);
    traverseClimberActivate = Robot.ph.makeSolenoid(8);
    traverseClimberDeactivate = Robot.ph.makeSolenoid(9);
  }

  public void putLevel2ClimberDown() {
    // set one side false before setting the other true
    // I also use multiple useful but unnecessary boolean variables to keep track of the settings.
    // In this case, it's isLevel2ClimberDown. The following 3 methods have similar formats
    level2ClimberUp.set(false);
    level2ClimberDown.set(true);
    isLevel2ClimberDown = true;
  }

  public void putLevel2ClimberUp() {
    level2ClimberDown.set(false);
    level2ClimberUp.set(true);
    isLevel2ClimberDown = false;
  }

  public void activateTraverseClimber() {
    traverseClimberDeactivate.set(false);
    traverseClimberActivate.set(true);
    isTraverseClimberActivated = true;
  }

  public void deactivateTraverseClimber() {
    traverseClimberActivate.set(false);
    traverseClimberDeactivate.set(true);
    isTraverseClimberActivated = false;
  }

  // this method runs all the time during teleop mode.
  public void climberTeleop() {

    // if button 7 on the xbox controller is pressed, toggle the level 2 climber
    if (Constants.xbox.getRawButtonPressed(7)) {
      if (isLevel2ClimberDown) {
        putLevel2ClimberUp();
      } else {
        putLevel2ClimberDown();
      }
    }

    // same as above, but it's button 8 on xbox and toggle the traverse climber
    if(Constants.xbox.getRawButtonPressed(8)){
      if(isTraverseClimberActivated){
        deactivateTraverseClimber();
      } else {
        activateTraverseClimber();
      }
    }

    // initialize speed value based on axis 5 on the xbox controller
    double traverseClimberArmSpeed = Constants.xbox.getRawAxis(5);
    // the speed value is squared, so this variable keeps track of whether it was originally negative
    boolean isTraverseSpeedNegative = false;

    // sees if the speed is negative and sets above variable to true if it is.
    if(traverseClimberArmSpeed < 0.0){
      isTraverseSpeedNegative = true;
    }
    // threshold, similar to drive train one
    if((traverseClimberArmSpeed > -0.2) && (traverseClimberArmSpeed < 0.2)){
      traverseClimberArmSpeed = 0.0;
    }
    // if negative, set traverse climb motor to negative squared speed
    // else, set it to squared speed, which is always positive
    if(isTraverseSpeedNegative){
      traverseClimberMotor.set(-1.0 * traverseClimberArmSpeed*traverseClimberArmSpeed);
    } else {
      traverseClimberMotor.set(traverseClimberArmSpeed*traverseClimberArmSpeed);
    }
    
  }

}