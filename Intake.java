package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake {

    // two motor declarations and initializations for intake and indexer motors
    CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorIndex, MotorType.kBrushed);
    CANSparkMax indexerMotor = new CANSparkMax(Constants.indexerMotorIndex, MotorType.kBrushless);

    // two Solenoid object for each setting of the intake double solenoid on the robot.
    // further explanation in Climber class
    Solenoid intakeUpSolenoid;
    Solenoid intakeDownSolenoid;

    // an unnecessary but useful true or false value that keeps track of the intake setting
    static boolean isIntakeDown = false;

    // only one I2C port on the robo rio, and this is how you declare and initialize it
    I2C.Port i2cPort = I2C.Port.kOnboard;
    // Rev color sensor declaration and initialization using the I2C port
    ColorSensorV3 indexerColorSensor = new ColorSensorV3(i2cPort);

    // true or false value that keeps track of whether the intake motor is running
    static boolean intaking = false;

    // true or false value that keeps track of whether the color sensor is being used
    // for intaking (can be disabled by the press of one button via code).
    static boolean intakingByColor = true;

    public void intakeInit() {
        // initialize intaking to false. Start out not running intake
        intaking = false;
        
        // voltage compensation for intake and indexer motor. explained in other classes
        intakeMotor.enableVoltageCompensation(12.5);
        indexerMotor.enableVoltageCompensation(12.5);

        // smart current limit for intake motor. explained in other classes
        intakeMotor.setSmartCurrentLimit(35, 40);

        // initialization of Solenoid objects. explanation in other classes
        intakeUpSolenoid = Robot.ph.makeSolenoid(12);
        intakeDownSolenoid = Robot.ph.makeSolenoid(10);
    }

    // true or false variable that keeps track of whether a ball is in the intake when using color sensor for intaking
    static boolean ballPresent = false;
    // since indexing with the color sensor is time-based, there is a Timer object to keep track of the time
    static Timer indexingTimer = new Timer();

    // method that contains the logic for indexing with the color sensor. Always running
    public void indexByColor() {
        // creates Color object for the color read by the color sensor
        Color detectedColor = indexerColorSensor.getColor();
        // System.out.println("red: " + detectedColor.red + ", blue: " + detectedColor.blue + ", green: " + detectedColor.green);

        // if in the process of shooting, all indexing needs to be controlled by the shooting methods, so this first
        // if statement makes sure the robot is not shooting.
        // fyi, "!" is used for booleans, and it means the opposite.
        // for example, if I made a new boolean like this:
        // boolean newBoolean = !false;
        // it's value would be true. The opposite of false is true.
        // so here, as long as we're not shooting, dumpShot, lowDumpShot, or hangarShot, we can go past the first if statement
        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot && !Robot.shooter.lowDumpShot && !Robot.shooter.hangarShot) {
            // these are value ranges that the color sensor will have if it encounters a ball
            if(((detectedColor.red > 0.30) && (detectedColor.red < 0.40)) || ((detectedColor.blue > 0.30) && (detectedColor.blue < 0.50))){
                // if a ball is not already in the indexer
                if(!ballPresent){
                    ballPresent = true; // set variable to true, indicating there is now a ball in the indexer
                    indexingTimer.reset(); // set timer to 0.0 s
                    indexingTimer.start(); // start timer
                }
            } else {
                // indexer is always running unless a ball is present and 0.65 seconds or more have passed
                indexerMotor.set(-0.2);
            }

            // if there's a ball in the indexer and more than 0.65 seconds have passed, stop the indexer motor.
            // what this logic allows is for the indexer to run for 0.65 seconds after it sees a ball, which puts
            // it farther into the indexer and makes the ball closer to the shooter, thus reducing the time to shoot
            if(ballPresent && (indexingTimer.get() > 0.65)){
                indexerMotor.set(0.0);
            }
        }
    }

    // put intake up. self-explanatory. similar method explanation in Climber
    public void intakeUp(){
        intakeDownSolenoid.set(false);
        intakeUpSolenoid.set(true);
        isIntakeDown = false;
    }

    // same as above
    public void intakeDown(){
        intakeUpSolenoid.set(false);
        intakeDownSolenoid.set(true);
        isIntakeDown = true;
    }

    // runs all the time during teleop
    public void intakeTeleop() {

        // one press of a button turns off intaking by color for the rest of the match
        if(Constants.xbox.getRawButtonPressed(9)){
            intakingByColor = false;
        }

        // resets the ballPresent variable to false. axis 3 on the xbox controller is used to shoot,
        // so after it is pressed (it is a trigger), a ball should no longer be in the indexer
        if(Constants.xbox.getRawAxis(3) > 0.8){
            ballPresent = false;
        }

        // displays true or false on smart dashboard for whether the intake is running
        SmartDashboard.putBoolean("Intake", intaking);

        // if the intake is down
        if (isIntakeDown) {
            // run intake backwards press and hold
            if (Constants.stick.getRawButton(10)) {
                intakeMotor.set(0.6);
                intaking = false;
            // button 1 on the joy stick toggles the intake running
            } else if (Constants.stick.getRawButtonPressed(1)) {
                intaking = !intaking; // toggle is here
                // if now true, run the intake
                if (intaking) {
                    intakeMotor.set(-0.8);
                }
            } else {
                // if intaking is false, stop the intake
                if (!intaking) {
                    intakeMotor.set(0.0);
                }
            }
        // intake never runs if it is up
        } else {
            intakeMotor.set(0.0);
        }

        // as long as the robot is not in the process of shooting
        if (!Robot.shooter.shooting && !Robot.shooter.dumpShot && !Robot.shooter.lowDumpShot && !Robot.shooter.hangarShot) {
            // evaluate the D-Pad on the xbox controller.
            if (Constants.xbox.getPOV() == 90) {
                indexerMotor.set(-0.3); // index a ball in with manual control
            } else if (Constants.xbox.getPOV() == 270) {
                indexerMotor.set(0.3); // index a ball out with manual control
            } else {
                if(intakingByColor){ // if intaking/indexing by color and the variable is not false, index by color
                    indexByColor();
                } else { // if it is false, stop the indexer motor.
                    indexerMotor.set(0.0);
                }
            }
        }

        // if button 2 on the xbox controller is pressed, toggle the intake.
        // similar logic in climber
        if(Constants.xbox.getRawButtonPressed(2)){
            if(isIntakeDown){
                intakeUp();
            } else {
                intakeDown();
            }
        }
    }
    
}