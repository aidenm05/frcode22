package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

public class DriveTrain {

    // 4 drive motor objects. FL, BL, FR, BR
    // two arguments. first argument = CAN index. second argument = type i.e. brushed vs brushless motor type
    CANSparkMax flMotor = new CANSparkMax(Constants.flMotorIndex, MotorType.kBrushless);
    CANSparkMax blMotor = new CANSparkMax(Constants.blMotorIndex, MotorType.kBrushless);
    CANSparkMax frMotor = new CANSparkMax(Constants.frMotorIndex, MotorType.kBrushless);
    CANSparkMax brMotor = new CANSparkMax(Constants.brMotorIndex, MotorType.kBrushless);
    // MecanumDrive object that combines and controls are four drive train motors in an easy way
    MecanumDrive mecanumDrive = new MecanumDrive(flMotor, blMotor, frMotor, brMotor);

    // this variable did not end up being used
    boolean direction = false; // false: intake is front. true: shooter in front

    // controls deadband for the different axes on the joystick.
    // a joystick axis must be pushed to at least 0.2 to be activated.
    // deadband is used to avoid interference between axes and allows greater precision the driver
    // final means the value of the variable cannot be changed anywhere else in the code but here
    final double motorSpeedThresholdTeleop = 0.2;
    // coefficient that controls the absolute maximum power able to be given to drive train motors.
    // no greater than 87%.
    // this variable is not used in the code. driveSpeedCoefficient performs the exact same function
    final double maxMotorSpeedTeleop = 0.87;
    // min and max here describe the speed parabola that driveTrainByInches uses when revving up
    // and revving down the drive train motors. the values here can affect how accurate the method is.
    // the max is not "final" because it is changed some duringa some of the utonomous modes
    final double minMotorSpeedEncoders = 0.1;
    double maxMotorSpeedEncoders = 0.5;

    final static double ENCODERS_PER_REV = 42.0; // encoder counts per revolution of the motor
    final static double GEAR_RATIO = 12.75; // inches // motor spins 12.75 times for wheel to spin once
    final static double wheelRadius = 4.0; // inches
    final static double driveTrainInchesOffset = 0.0; // inches

    // coefficient that controls the absolute maximum power able to be given to drive train motors.
    // no greater than 87%.
    static double driveSpeedCoefficient = 0.75;

    public void driveTrainInit() { // make the drive train ready to be used
        // changes what positive and negative values mean for speed and encoders
        frMotor.setInverted(true);
        brMotor.setInverted(true);
        blMotor.setInverted(false);
        flMotor.setInverted(false);

        resetDriveTrainEncoders(); // sets encoder values on drive train motors to 0.0

        // multiples motor encoder values by the argument in the setPositionEncoderFactor() method.
        // the getEncoder() method for the motor provides access to the encoder object
        // that every motor has. Some methods like setPositionEncoderFactor() are only
        // accessible this way.
        // setPositionEncoderFactor() is useful for directly relating encoders, which are a measurement
        // of how much a motor turns, to how much a wheel turns in inches, for example
        flMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        frMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        blMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);
        brMotor.getEncoder().setPositionConversionFactor(ENCODERS_PER_REV);

        // enableVoltageCompensation means that it doesn't matter
        // if battery voltage drops. voltage compensation means that
        // the same % power that a motor is set to is the same at any voltage
        // relative to 12.5 volts. argument = voltage
        flMotor.enableVoltageCompensation(12.5);
        frMotor.enableVoltageCompensation(12.5);
        blMotor.enableVoltageCompensation(12.5);
        brMotor.enableVoltageCompensation(12.5);

        // setSmartCurrentLimit is about limiting the power draw of motors so they don't burn out.
        // first argument = stall limit in amps. second argument = free limit in amps.
        // the same is true of methods as with constructors. To see a brief description of the method
        // and its required arguments, hold the control key and hover your mouse over it
        flMotor.setSmartCurrentLimit(40, 55);
        frMotor.setSmartCurrentLimit(40, 55);
        blMotor.setSmartCurrentLimit(40, 55);
        brMotor.setSmartCurrentLimit(40, 55);
    }

    // method for regular teleop driving.
    // takes 3 arguments, which are the values of the axes from the joystick
    public void driveTrainByControls(double ySpeed, double xSpeed, double zSpeed) {
        
        // while button 4 on the joy stick is being held, turbo mode is activated.
        // driveSpeedCoefficient is increased
        if (Constants.stick.getRawButton(4)) {
            driveSpeedCoefficient = 0.85;
        } else {
            driveSpeedCoefficient = 0.75;
        }

        // values are squared below, meaning that negative can become positive
        // this code keeps track of whether the values were negative or positive
        // before squaring and then re-adds the negative below, if applicable
        double yDirectionMaintainer = 1.0;
        double xDirectionMaintainer = 1.0;
        double zDirectionMaintainer = 1.0;
        if (ySpeed < 0.0) {
            yDirectionMaintainer = -1.0;
        }
        if (xSpeed < 0.0) {
            xDirectionMaintainer = -1.0;
        }
        if (zSpeed < 0.0) {
            zDirectionMaintainer = -1.0;
        }

        // deadband. any value within the deadband is set to 0.00.
        // deadband is different for some axes.
        if ((ySpeed > -0.2) && (ySpeed < 0.2)) {
            ySpeed = 0.00;
        }
        if ((xSpeed > -0.2) && (xSpeed < 0.2)) {
            xSpeed = 0.00;
        }
        if ((zSpeed > -0.4) && (zSpeed < 0.4)) {
            zSpeed = 0.00;
        }

        // driveCartesian is a method in the MecanumDrive class created by first.
        // it takes 3 arguments in a specific order which are the 3 axes of movement.
        // each value is squared and multiplied by the driveSpeedCoefficient as well as
        // the direction maintainers described above. some axes, such as the y axis, are inverted
        // by the controller, so there is a negative sign in front one of the factors.
        // there are also a few "+ 0.1"s added to the coefficent values for axes that needed an extra 10% power.
        mecanumDrive.driveCartesian(driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                (driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer,
                (driveSpeedCoefficient + 0.1) * zSpeed * zSpeed * zDirectionMaintainer); // intake is front
    }

    public void driveTrainByInches(double inches, int direction) { // INTAKE ALWAYS FORWARD.
        // 0 = forward. 1 = back. 2 = left. 3 = right. 4 = turn left. 5 = turn right

        // uses quadraticPositionAndSpeed, which creates a parabola that maxes out
        // at the value of maxMotorSpeedEncoders. motor revs up and revs down very quickly
        // so that the motor arrives at its target number of encoders accurately and does not go over.
        // first argument = min speed for parabola. must be greater than about 0.075 for it to start.
        // the parabola relates position (x axis) and speed (y axis). causes the need for an offset
        // second argument = max speed for parabola.
        // third argument = encoder target
        // four argument = current position of the front left motor. Should be about the same
        // for all four motors. the getPosition() method of an encoder object of a motor provides this value
        if (direction == 0) { // forward
            mecanumDrive.driveCartesian(Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders,
                    maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if (direction == 1) { // back
            mecanumDrive.driveCartesian(-Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders,
                    maxMotorSpeedEncoders, inchesToEncoders(inches), -flMotor.getEncoder().getPosition()), 0.0, 0.0);
        }

        if (direction == 2) { // left
            mecanumDrive.driveCartesian(0.0, Constants.quadraticPositionAndSpeed(-minMotorSpeedEncoders,
                    -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()), 0.0);
        }

        if (direction == 3) { // right
            mecanumDrive.driveCartesian(0.0, Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders,
                    maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()), 0.0);
        }

        if (direction == 4) { // turn left
            mecanumDrive.driveCartesian(0.0, 0.0, Constants.quadraticPositionAndSpeed(-minMotorSpeedEncoders,
                    -maxMotorSpeedEncoders, inchesToEncoders(inches), frMotor.getEncoder().getPosition()));
        }

        if (direction == 5) { // turn right
            mecanumDrive.driveCartesian(0.0, 0.0, Constants.quadraticPositionAndSpeed(minMotorSpeedEncoders,
                    maxMotorSpeedEncoders, inchesToEncoders(inches), flMotor.getEncoder().getPosition()));
        }

    }

    ///////////////////////////////////////////////////////////////////////
    // useful drivetrain functions

    // function that sets all drive train motor encoder values to 0.
    public void resetDriveTrainEncoders() {
        flMotor.getEncoder().setPosition(0);
        frMotor.getEncoder().setPosition(0);
        blMotor.getEncoder().setPosition(0);
        brMotor.getEncoder().setPosition(0);
    }


    // pretty self-explanatory. converts inches to encoders.
    // the word that comes before the name of the method in the method definition is the type of the method.
    // in this case, it is double, meaning that the method returns a value of type double when it is
    // done processing. If I wanted, for example, create a new variable like so:
    // double newDouble = inchesToEncoders(5.0);
    // the method would process and then set my new variable equal to the return value of the method.
    // methods can have a type and thus a return value or they can have no type and just perform its function.
    // methods that have no return value say void where the type would be in the method definiton.
    // a great example is the method above, resetDriveTrainEncoders(). There's no reason to have a return value
    // for this method. It just does it's job of setting all drive train motor encoder values to 0.
    // methods can return a primitive type i.e. int, float, char, double, boolean.
    // methods can also return objects of classes i.e. String, DriveTrain, Robot, CANSparkMax

    // Also, whoever is reading this, I would do some research on the rules of mathematical operations in Java
    // and also casting. Mathematical operations can be weird in Java. Occasionally, numbers can be truncated
    // to the lower integer, which can be 0 in some cases, which will obviously mess up calculations.
    // To avoid problems with operations, I make basically all numbers decimals, which will cause no problems.
    // there's some exceptions to this, like when I am trying to increase efficiency by calculating with
    // numbers and variables of lower bit sizes.
    // fyi all decimal numbers are, by default, of type double.

    // Casting is when variables or numbers are converted from one type to another. this can be useful
    // when performing mathematical operations (very relevant to FRC) or dealing with inheritance
    // or polymorphism (not super relevant to FRC).
    // a basic example would be with floats and doubles. A double is a 64-bit variable, whereas
    // a float is a 32-bit variable that could, in theory, be more efficient in large calculations.
    // Here's a basic example:
    // double newDouble = 2.0;
    // float newFloat = (float) newDouble;
    // it's converted simply by using the (float) in parentheses. I use casting in the Shooter class
    public static double inchesToEncoders(double inches) {
        return (((Math.abs(inches) - driveTrainInchesOffset) / (2.0 * Math.PI * wheelRadius)) * ENCODERS_PER_REV
                * GEAR_RATIO);
    }

    // very self-explanatory. Converts encoders to inches based on values found above.
    // takes one double argument of encoders and returns a double value of inches
    public static double encodersToInches(double encoders) {
        return ((((encoders / GEAR_RATIO) / ENCODERS_PER_REV) * (2.0 * Math.PI * wheelRadius))
                + driveTrainInchesOffset);
    }

    // self-explanatory. sets all drive train motors to 0% output
    public void stopMotors() {
        flMotor.set(0.0);
        blMotor.set(0.0);
        frMotor.set(0.0);
        brMotor.set(0.0);
    }
    /////////////////////////////////////////////////////////////////////
}
