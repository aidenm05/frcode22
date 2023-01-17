package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    // note the classes that are used here. WPI_TalonFX, CANSparkMax, PWMSparkMax.
    // there are classes with many similar names throughout the First and non-First libraries.
    // be mindful of which one you are using because not all of them work.
    // everything here worked for us. WPI_TalonFX is used for the CTRE Falcon 500 motors.
    WPI_TalonFX masterShooterMotor = new WPI_TalonFX(Constants.masterShooterMotorIndex);
    WPI_TalonFX slaveShooterMotor = new WPI_TalonFX(Constants.slaveShooterMotorIndex);
    CANSparkMax hoodMotor = new CANSparkMax(Constants.hoodMotorIndex, MotorType.kBrushless);
    // the one argument in this method is the CAN index number
    PWMSparkMax LEDsSparkMax = new PWMSparkMax(0);

    // four true or false variables indicating which shooting process the robot is in.
    // all initialized to false. Code does not allow for any more than one of them
    // to be true at once.
    boolean shooting = false;
    boolean dumpShot = false;
    boolean lowDumpShot = false;
    boolean hangarShot = false;

    // voltage used for voltage compensation methods in the init method below.
    final double saturationVoltage = 12.5;

    // void method for setting hood encoders to 10.5 because this is approximately
    // the degree the hood is at when it's all the way down on the robot.
    public void resetHoodEncoders() {
        hoodMotor.getEncoder().setPosition(10.5);
    }

    public void shooterRobotInit() {
        // the robot has two talon motors for the shooter. Both motors need to be
        // doing the exact same thing, so one is called master and the other slave.
        // with this follow() method, the slave copies everything that the master does,
        // so you only have to set one of them to spin, for example.
        slaveShooterMotor.follow(masterShooterMotor);
        // inverts which direction is positive or negative for encoders and power.
        hoodMotor.setInverted(true);
        // the conversion factor calculated for encoders of the hood motor and degrees
        hoodMotor.getEncoder().setPositionConversionFactor(0.65222);
        // voltage compensation for talonFX shooter motors. Voltage compensation explained
        // in DriveTrain class. Requires two methods to enable voltage compensation
        // for the talonFX motors. configVoltageCompSaturation() and enableVoltageCompensation().
        // configVoltageCompSaturation() takes one double argument, the value in volts of the compensation
        masterShooterMotor.configVoltageCompSaturation(saturationVoltage);
        slaveShooterMotor.configVoltageCompSaturation(saturationVoltage);
        // enableVoltageCompensation() takes one boolean argument, whether voltage compensation is enabled
        masterShooterMotor.enableVoltageCompensation(true);
        slaveShooterMotor.enableVoltageCompensation(true);

        //
        // I honestly do not understand what this section means. This has to do with PID,
        // which I never really learned. This is code that I copy-and-pasted from somewhere else
        // that allowed setting the shooter motors by velocity instead of power.
        masterShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        masterShooterMotor.setSensorPhase(true);

        masterShooterMotor.configNominalOutputForward(0, 30);
        masterShooterMotor.configNominalOutputForward(0, 30);
        masterShooterMotor.configPeakOutputForward(1, 30);
        masterShooterMotor.configPeakOutputReverse(-1, 30);

        masterShooterMotor.config_kF(0, 0.34, 30);
        masterShooterMotor.config_kP(0, 0.2, 30);
        masterShooterMotor.config_kI(0, 0, 30);
        masterShooterMotor.config_kD(0, 0, 30);
        //

        // explained above
        resetHoodEncoders();
    }

    // method I made for myself that sets all shooting variables to false.
    public void shooterInit() {
        shooting = false;
        dumpShot = false;
        lowDumpShot = false;
        hangarShot = false;
        // this method also turns on the limelight.
        // the same method except for the ending setDouble(1.0) instead of 3.0 turns the limelight off.
        // the same information can be found in the limelight documentation online.
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(3.0);
    }

    // some constants in inches that I used in calculations
    // notice that they are all final, static floats.
    // their values can't be changed, they don't need to be accessed using an object, and they are
    // half as many bits as doubles. Everything is in the name and refers
    // to horizontal distance
    final static float distanceFromTapeToCenterOfHub = 26.6875f;
    final static float distanceFromFenderToCenterOfHub = 33.875f;
    final static float distanceFromFenderToTape = distanceFromFenderToCenterOfHub - distanceFromTapeToCenterOfHub;
    // final static float horizontalDistanceFromLimeLightToShooter = 5.375f;
    // final static float horizontalDistanceFromShooterToFrontOfRobot = 7.25f;
    // final static float horizontalDistanceFromLimelightToFrontOfRobot =
    // horizontalDistanceFromShooterToFrontOfRobot
    // - horizontalDistanceFromLimeLightToShooter;
    final static float horizontalDistanceFromFrontOfRobotToLimelight = 15.0f;
    final static float horizontalDistanceFromFrontOfRobotToShooter = 7.25f;
    final static float horizontalDistanceFromLimelightToShooter = horizontalDistanceFromFrontOfRobotToLimelight
            - horizontalDistanceFromFrontOfRobotToShooter;

    // gravity in in/s^2
    final static float gravity = 386.103f;
    // height of upper hub
    final static float y = 104.0f;
    // height of shooter
    final static float y0 = 22.5f;
    // all of these numbers below speed up the calculated velocity calculations because they
    // reduce the number of necessary iterations.
    // minimum calculated velocity needed in in/s shooting from anywhere on the field
    final static float minimumVelocity = 267.7f;
    // maximum calculated velocity needed in in/s shooting from anywhere on the field
    final static float maximumVelocity = 405.5f;
    // minimum calculated velocity needed in radians shooting from anywhere on the field
    final static float minimumAngle = 0.804f;
    // minimum calculated velocity needed in radians shooting from anywhere on the field
    final static float maximumAngle = 1.387f;

    // camera height from ground
    private static float cameraHeight = 40.0f;
    // camera base angle in degrees
    private static float cameraAngle = 45.5f;
    // my measurement of how the camera is set could be off, so this variable is in place
    // so that adjustments can be made to the angle value easily for accurate distance calculation
    static float angleError = 20.25f; // degrees

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) {
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError)))
    // + (distanceFromTapeToCenterOfHub + 24.0f) -
    // horizontalDistanceFromLimelightToShooter);
    // }

    // trigonometric formula using the angle given by the limelight to determine horizontal distance based
    // on the fixed vertical distance.
    // The Math class has many valuable constants and methods like the trigonometric functions and conversion methods,
    // both of which are used in these two methods
    public static float getXDistanceToGoal(double verticalAngle) {
        return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError)))
                - horizontalDistanceFromLimelightToShooter + distanceFromTapeToCenterOfHub + 18.0f);
    }

    // very similar method as above, but instead of distance from shooter to the hub
    public static float getXDistanceFromFrontOfRobotToFender(double verticalAngle) {
        return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle + cameraAngle - angleError)))
                - distanceFromFenderToTape - horizontalDistanceFromFrontOfRobotToLimelight);
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromFrontOfRobotToLimelight -
    // distanceFromFenderToTape);
    // }

    // these two variables are for the math shot
    static float calculatedVelocity = 0.0f;
    static float calculatedAngle = 0.000f;
    // this variable is the coefficient that relates linear velocity of the shooter wheels,
    // and projectile velocity of the ball
    public static float shootingCoefficient = 2.765f;

    // these are 3 boolean variables that are set to false each time the robot prepares to shoot
    // this is true when the hood is in the right position to shoot
    static boolean setHoodYet = false;
    // this is set to true if the hood has to move up to get to the right angle to shoot
    static boolean goUp = false;
    // this is set to true if the hood has to move down to get to the right angle to shoot
    static boolean goDown = false;

    // this is for the math shot, keeps track of whether the math shot calculation has been initiated
    static boolean calculationBoolean = false;
    // % power values for different shots the robot can make. found through experimentation
    public double dumpShotSpeed = 0.4472;
    public double lowDumpShotSpeed = 0.22;
    public double hangarShotSpeed = 0.50;

    // used for the math shot. tempAngle is used in the code for determining whether the math shot calculation is complete
    static float tempAngle = 0.000f;
    // this variable is not used in the code because its function is performed elsewhere.
    // it is meant to keep track of the calculated distance that the robot is from the hub
    static float calculatedDistance = 0.000f;

    // these three methods perform the math shot calculations.
    // essentially, this code finds the minimum possible projectile velocity where there exists an angle at which to shoot
    // that can make the shot from anywhere on the field. The code does this by efficiently and systematically iterating
    // through all possible combinations of velocities and angles, starting from the low end of numbers, until it finds
    // a combination that works, in which case the code will stop and record the values.

    // this is very complex math for someone who didn't write it, but it's actually very simple to me.
    // essentially, you separate the x and y component funuctions of the projectile motion of the ball. you then
    // find their intersection points with horizontal lines of the distance goals of each component.
    // you then find the slope of line that goes through both of those intersection points, which is calculated
    // by the slopeOfLine method below. then, iterating through the possible velocities and angles, record the combination
    // that is exactly when the value of the slope switches from positive to negative because it will a line nearly
    // straight up and down, meaning that the time at which the ball reaches its x distance goal and y distance goal
    // will be the same, and since it is the minimum possible velocity, it will take the least amount of time
    // for the shooter to rev up. The ball will also be more stable in the air because it is not going as fast
    // and will in theory not land in the hub with as much velocity.
    public static void testVelocity(float x) {

        float velocity = 0.0f;
        for (float i = minimumVelocity; i <= maximumVelocity; i += 1.0f) {
            if ((testAngle(i, x) > minimumAngle) && (testAngle(i, x) < maximumAngle)) {
                velocity = i;
                break;
            }
        }
        calculatedVelocity = velocity;
        calculatedAngle = testAngle(velocity, x);
    }

    public static float testAngle(float velocity, float x) {

        float angle = minimumAngle;

        float[] possibleAngles = new float[(int) ((maximumAngle * 1000.0f) - (minimumAngle * 1000.0f) + 1)];
        for (int i = 0; i < possibleAngles.length; i++) {
            possibleAngles[i] = angle;
            angle = angle + 0.001f;
        }

        for (float possibleAngle : possibleAngles) {
            float slope = slopeOfLine(velocity, possibleAngle, x);
            if (!Double.isNaN(slope)) {
                if (slope > 0.00) {
                    angle = possibleAngle;
                    break;
                }
            }
        }

        while (slopeOfLine(velocity, angle, x) > 0.000f) {
            angle += 0.005f;
        }
        return angle;
    }

    public static float slopeOfLine(float velocity, float possibleAngle, float x) {

        return (float) (((y - x)
                / (((-1.0f) * ((-1.0f) * velocity * Math.sin(possibleAngle) - Math.sqrt(Math.pow(velocity, 2)
                        * Math.pow(Math.sin(possibleAngle), 2) + ((2.0f * gravity) * (y0 - y)))) / gravity)
                        - (x / (velocity * Math.cos(possibleAngle))))));
    }

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    // self-explanatory. converts the shooter wheel linear velocity to the built in motor velocity sensor
    // in the falcon 500
    public static float shooterWheelLinearVelocityToMotorVelocity(double projectileVelocity) {
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21) * 2048.0f * (1.0f / 10.0f))));
    }

    // self-explanatory. same as above except it converts to motor percent output instead of the motor velocity.
    // this method is based on an experimental value that relates motor percent output and motor velocity
    public static float shooterWheelLinearVelocityToMotorPercentOutput(double projectileVelocity) {
        // same method except dividing. could change to return above method / 22068.97f
        return (float) ((shootingCoefficient
                * (projectileVelocity * (1.0f / (4.0f * Math.PI)) * (1.0f / 1.21f) * 2048.0f * (1.0f / 10.0f)))
                / 22068.97f);
    }

    // variable used for centering. not used in code. it was supposed to be used
    // as a value in a quadratic function that centered the robot and gradually slowed down
    // the centering speed based on how far off from center the robot was from the target.
    double centeringA = 0.0002;

    // boolean method that centers the robot. Returns true if robot is centered on the target,
    // returns false otherwise.
    // this method turns the robot in the z direction and allows the drivers to go forward and back
    // or strafe while centering and stay locked on to the target.
    // this method is used in teleop. Refer to teleopPeriodic() in the Robot class for usage.
    public boolean centerRobotOnTarget() {
        // finds the x angle offset value from the limelight. there is an additional -4.0 degree offset
        // put in here because the shooter of the robot was tilted to the right about 4.0 degrees
        double xAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0) - 4.0;

        // records the y and x inputs on the joy stick
        double ySpeed = Constants.stick.getRawAxis(1);
        double xSpeed = Constants.stick.getRawAxis(0);
        // direction maintainer for squared y and x values. explained in DriveTrain class
        double yDirectionMaintainer = 1.0;
        double xDirectionMaintainer = 1.0;
        if (ySpeed < 0.0) {
            yDirectionMaintainer = -1.0;
        }
        if (xSpeed < 0.0) {
            xDirectionMaintainer = -1.0;
        }
        if ((ySpeed > -0.2) && (ySpeed < 0.2)) {
            ySpeed = 0.00;
        }
        if ((xSpeed > -0.2) && (xSpeed < 0.2)) {
            xSpeed = 0.00;
        }

        // turbo mode. explained in DriveTrain class
        if (Constants.stick.getRawButton(4)) {
            Robot.drive.driveSpeedCoefficient = 0.85;
        } else {
            Robot.drive.driveSpeedCoefficient = 0.7;
        }

        // this is a series of if statements that changes how the robot moves depending on the angle
        // read by the limelight with the -4.0 degree offset. The farther away the robot is from center, which
        // is determined by the magnitude of the xAngle variable, the faster the robot turns towards the center.
        // in the first if statement below, it checks if the robot is off by more than 15 degrees. If so,
        // it turns the robot at 0.2, or 20% speed. There are two if statements for each magnitude of offset,
        // with one being positive and the other negative. This is so the robot can center whether it needs
        // to turn right or left in order to center. I also use else if statements and finally an else at the end
        // to ensure that only one of these scenarios is carried out each time the method is called so there are
        // not conflicting directions in code for the robot.
        // in the subsequent statements, like the ones with the magnitude of 4.0 degrees, the speed slows down
        // to 0.12 and finally to 0.06. Within 1.0 degree of the center, the robot will stop turning because
        // it is now essentially centered.
        if (xAngle < -15.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.2);
            return false;
        } else if (xAngle > 15.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.2);
            return false;
        } else if (xAngle < -4.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.12);
            return false;
        } else if (xAngle > 4.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.12);
            return false;
        } else if (xAngle < -1.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, -0.06);
            return false;
        } else if (xAngle > 1.0) {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.06);
            return false;
        } else {
            Robot.drive.mecanumDrive.driveCartesian(
                    Robot.drive.driveSpeedCoefficient * -ySpeed * ySpeed * yDirectionMaintainer,
                    (Robot.drive.driveSpeedCoefficient + 0.1) * xSpeed * xSpeed * xDirectionMaintainer, 0.0);
            return true;
        }
    }

    // this method finds the y angle offset from the limelight, then uses it to calculate the horizontal distance
    // that the robot is away from the hub.
    // the robot has a sweet spot distance for the dump shot, so this method works to get the robot to that distance
    // from wherever they can see the target on the field and ideally when there are centered on the target.
    public boolean distanceRobotFromTargetCorrectly() {
        // get y angle offset from limelight
        double yAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        // calculate horizontal distance the robot is from the hub.
        double distance = getXDistanceFromFrontOfRobotToFender(yAngle);

        // if the robot is less than 33.0 inches away from the target, move back from the hub.
        // the reason there is a greater than 300.0 inches away is because when the robot got very close to the hub,
        // the limelight would sometimes lock on to another target, causing the calculated distance to be very large.
        // In this case, the robot would assume through this code that this error was happening, and would move back.
        if ((distance < 33.0) || (distance > 300.0)) {
            Robot.drive.mecanumDrive.driveCartesian(0.175, 0.0, 0.0);
            return false;
        // if the distance is greater than 55.0 inches, move forward towards the hub.
        } else if (distance > 55.0) {
            Robot.drive.mecanumDrive.driveCartesian(-0.175, 0.0, 0.0);
            return false;
        // if within 33.0 and 55.0 inches, stop. in sweet spot.
        // note the use of if, else if, and finally else again so that only one scenario runs each time the method is called
        } else {
            Robot.drive.mecanumDrive.driveCartesian(0.0, 0.0, 0.0);
            return true;
        }
    }

    // the one double argument of this method determines the idle shooter speed, meaning the speed
    // that the shooter stays at when not shooting.
    public void shooterIdle(double shooterSpeed) { // does this need to be in the shooter methods or just in
                                                   // teleopPeriodic
        // if not shooting and no manual control of the hood
        if (!shooting && !dumpShot && !lowDumpShot && !hangarShot && Constants.xbox.getPOV() != 0 && Constants.xbox.getPOV() != 180) {
            // set master shooter speed, which sets both the master and slave motors to the same speed, to the argument double value
            masterShooterMotor.set(shooterSpeed);
            // if autonomous (not teleop), set indexer speed to 0 because indexByColor cannot be disabled during autonomous
            // so it might as well just be off because it's not necessary and could cause unwanted issues.
            if (!DriverStation.isTeleop()) {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    // keeps track of shooting time. Used so the robot does not get locked in shooting mode or is used
    // if the secondary driver forgets to turn it off by placing a time limit on shooting.
    Timer shootingTimer = new Timer();

    // method for shooting prepration. resets all variables needed.
    public void shootInit() {
        setHoodYet = false;
        tempAngle = 0.000f;
        goUp = false;
        goDown = false;
        calculationBoolean = false;
    }

    // math shot. very similar to other shooting methods. differences are described below where they occur.
    public void shoot() {

        if (Constants.xbox.getRawButtonPressed(5)) {
            if (!dumpShot && !lowDumpShot && !hangarShot) {
                shooting = !shooting;
                if (shooting) {
                    shootInit();
                }
            }
        }

        if (shooting && !dumpShot && !lowDumpShot && !hangarShot) {

            // if haven't initiated calculation yet
            if (!calculationBoolean) {
                // run test velocity with calculated distance based on y angle offset from the limelight.
                // process of calculation explained above.
                testVelocity(getXDistanceToGoal(
                        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
                // calculations have now been initiated. calculationBoolean is now true.
                calculationBoolean = true;
                shootingTimer.reset();
                shootingTimer.start();
            }

            // tests whether the calculations are complete by testing the absolute difference between
            // calculated angle and tempAngle to see that it is greater than 0.004.
            // double and float values are never exactly what you set them as i.e. if I set a new double value
            // to 1.0, it might actually be 0.999999999998 in the code, which is virtually the same, but not the same.
            // therefore, you should never directly test for equality between two doubles or floats, and instead check that
            // there absolute difference is greater than a certain value to make sure that they are actually different.
            // at first, both of these variables are set to 0.0, so if the calculation has completed, calculated angle should be
            // greater than 0.004.
            // this code actually could have been done a better way because after the code runs once, calculatedAngle is never set
            // to 0.0f again, but tempAngle is. The logic then technically didn't make sense, but the calculation happened so fast
            // because it was made so efficient that it didn't actually matter. Plus, the math shot was not used much in matches.
            if (Math.abs(calculatedAngle - tempAngle) > 0.004) {

                if (DriverStation.isTeleop()) {
                    if (shootingTimer.get() > 10.0) {
                        shooting = false;
                    }
                }

                // masterShooterMotor.set(ControlMode.PercentOutput,
                // (shooterWheelLinearVelocityToMotorPercentOutput(calculatedVelocity)));
                masterShooterMotor.set(ControlMode.Velocity, calculatedVelocity * shootingCoefficient);

                if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goDown == false)) {
                    goUp = true;
                }

                if (goUp) {
                    hoodMotor.set(0.2);
                    if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                if ((hoodMotor.getEncoder().getPosition() > (90.0 - Math.toDegrees(calculatedAngle)))
                        && (goUp == false)) {
                    goDown = true;
                }

                if (goDown) {
                    hoodMotor.set(-0.2);
                    if ((hoodMotor.getEncoder().getPosition() < (90.0 - Math.toDegrees(calculatedAngle)))) {
                        hoodMotor.set(0.0);
                        hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                                ControlType.kPosition);
                        setHoodYet = true;
                    }
                }

                double distance = getXDistanceToGoal(
                        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
                if (distance > 200.0) {
                    shootingCoefficient = 2.825f;
                } else {
                    shootingCoefficient = 2.765f;
                }
                double thresholdVelocity = calculatedVelocity;
                System.out.println("calculated: " + thresholdVelocity);
                System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
                System.out.println("distance: " + distance);
                if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                    if (DriverStation.isTeleop()) {
                        if (Constants.xbox.getRawAxis(3) > 0.8) {
                            Robot.intake.indexerMotor.set(-0.8);
                        } else {
                            Robot.intake.indexerMotor.set(0.0);
                        }
                    } else {
                        Robot.intake.indexerMotor.set(-0.8);
                    }
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
            }
        }
    }

    // calculates distance the robot is from the target. sets the robot's LEDs to blinking green
    // if in dump shot sweet spot and red if not. acts as a visual cue for the drivers.
    public void dumpShotLEDs() {
        // calculates distance using method above and y angle offset from the limelight
        double distance = getXDistanceFromFrontOfRobotToFender(
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0));
        System.out.println(distance);

        // if in sweet spot
        if ((distance > 33.0) && (distance < 55.0)) {
            // also have an element in smart dashboard, namely a boolean box, that the drivers can look at for sweet spot
            SmartDashboard.putBoolean("Dumpshot Distance", true);
            // the set "speed" here refers to a mode for the LEDs. The meaning of each set speed is specific
            // to the Rev Blinkin LED controller. The manual, which contains the table explaining
            // the set values and their respective modes, can be found online.
            // blinking green
            LEDsSparkMax.set(0.15);
        // if not in sweet spot
        } else {
            // element in smart dashboard that indicates the value, whether true or false, here
            SmartDashboard.putBoolean("Dumpshot Distance", false);
            // red
            LEDsSparkMax.set(0.57);
        }
    }

    public void dumpShot() { // 92 inches according to distance method
        // hood all the way down. there's code to put the hood down that could take away
        // later if we locked hood in place

        // dumpShotLEDs() method above always running and evaluating distance and changing the LED color accordingly
        dumpShotLEDs();

        // if button 4 on the xbox controller pressed,
        if (Constants.xbox.getRawButtonPressed(4)) {
            // check that not in any other shooting mode
            if (!shooting && !lowDumpShot && !hangarShot) {
                // if not, toggle dumpShot i.e. the same button can be used to turn on and off dumpShot
                dumpShot = !dumpShot;
                // if dumpShot is now true
                if (dumpShot) {
                    // run shootInit() and start the timer
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        // if dumpShot now true and other shooting variables not true
        if (dumpShot && !shooting && !lowDumpShot && !hangarShot) {

            // if 10.0 seconds or more have passed since turning on dumpShot, turn it off
            if (shootingTimer.get() > 10.0) {
                dumpShot = false;
            }

            // masterShooterMotor.set(ControlMode.PercentOutput, dumpShotSpeed);
            // using velocity control, enabled by PID stuff above, set master shooter motor
            // to the predetermined dumpShot velocity, which is the dumpShotSpeed coefficient value
            // times this constant of 1900.0
            masterShooterMotor.set(ControlMode.Velocity, dumpShotSpeed * 1900.0);

            // dump shot is with the hood all the way down. if hood is between 10.5 (the bottom) and 11.0 degrees,
            // it's fine. otherwise, the hood is moved down.
            // if the angle is greater than 11.0 degrees, goDown is true, which will start the process
            // of moving the hood down. If the angle is fine, the hood is set and the code can move on to the next part.
            if ((hoodMotor.getEncoder().getPosition() > 11.0)) { // only goes down to bottom angle of 10.5 degrees if
                                                                 // it's above.
                goDown = true;
            } else {
                setHoodYet = true;
            }

            if (goDown) {
                // set the hood to a negative speed so it goes down
                hoodMotor.set(-0.2);
                // hood does not stop instantly, so stop at 11.0 degrees and it will land at about 10.5 degrees.
                if ((hoodMotor.getEncoder().getPosition() < 11.0)) {
                    // stop the motor by setting speed to 0.0
                    hoodMotor.set(0.0);
                    // set reference to hood's current position using PID, which will not allow the hood to move.
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
                    // hood is now set. the code can move on.
                    setHoodYet = true;
                }
            }

            // minimum velocity that the motor must have in order to begin dump shot shooting.
            // the threshold velocity value makes sure that the shooter motor is revved up enough
            // to shoot.
            double thresholdVelocity = 9200.0;
            // prints threshold velocity
            System.out.println("calculated: " + thresholdVelocity);
            // prints actual motor velocity
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            // if the motor velocity is greater than the threshold velocity and the hood is set
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                // if teleop mode and in driver control
                if (DriverStation.isTeleop()) {
                    // secondary driver controls indexer. if raw axis 3, which is a trigger on the xbox controller,
                    // is pressed past 0.8, or 80%
                    if (Constants.xbox.getRawAxis(3) > 0.8) {
                        // set indexer to index in and put the ball in the shooter
                        Robot.intake.indexerMotor.set(-0.8);
                    } else {
                        // else stop the indexer
                        Robot.intake.indexerMotor.set(0.0);
                    }
                } else {
                    // if in autonomous, don't wait for driver control, spin indexer motor and put ball into shooter
                    Robot.intake.indexerMotor.set(-0.8);
                }
            } else {
                // if not up to speed yet or hood is not set, don't move indexer
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    // similar to dumpShot, but different set speed and threshold value
    public void lowDumpShot() {
        if (Constants.xbox.getRawButtonPressed(1)) {
            if (!shooting && !dumpShot && !hangarShot) {
                lowDumpShot = !lowDumpShot;
                if (lowDumpShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (lowDumpShot && !shooting && !dumpShot && !hangarShot) {

            if (shootingTimer.get() > 10.0) {
                lowDumpShot = false;
            }

            // masterShooterMotor.set(ControlMode.PercentOutput, dumpShotSpeed);
            masterShooterMotor.set(ControlMode.Velocity, lowDumpShotSpeed * 1900.0);

            if ((hoodMotor.getEncoder().getPosition() > 13.0)) { // only goes down to bottom angle of 10.5 degrees if
                                                                 // it's above.
                goDown = true;
            } else {
                setHoodYet = true;
            }

            if (goDown) {
                hoodMotor.set(-0.1);
                if ((hoodMotor.getEncoder().getPosition() < 13.0)) {
                    hoodMotor.set(0.0);
                    setHoodYet = true;
                }
            }

            double thresholdVelocity = 4000.0;
            System.out.println("calculated: " + thresholdVelocity);
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                if (DriverStation.isTeleop()) {
                    if (Constants.xbox.getRawAxis(3) > 0.8) {
                        Robot.intake.indexerMotor.set(-0.8);
                    } else {
                        Robot.intake.indexerMotor.set(0.0);
                    }
                } else {
                    Robot.intake.indexerMotor.set(-0.8);
                }
            } else {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    // similar to dumpShot, but different set speed, threshold value, and hood angle.
    // the code here involves the possibility of having the move the hood up or down
    // to get to the correct angle for shooting
    public void hangarShot() {
        if (Constants.xbox.getRawButtonPressed(3)) {
            if (!shooting && !dumpShot && !lowDumpShot) {
                hangarShot = !hangarShot;
                if (hangarShot) {
                    shootInit();
                    shootingTimer.reset();
                    shootingTimer.start();
                }
            }
        }

        if (hangarShot && !shooting && !dumpShot && !lowDumpShot) {

            if (shootingTimer.get() > 10.0) {
                hangarShot = false;
            }

            masterShooterMotor.set(ControlMode.Velocity, hangarShotSpeed * 1900.0);

            if ((hoodMotor.getEncoder().getPosition() < 34.5)
                    && (goDown == false)) {
                goUp = true;
            }

            if (goUp) {
                hoodMotor.set(0.2);
                if ((hoodMotor.getEncoder().getPosition() > 34.5)) {
                    hoodMotor.set(0.0);
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                            ControlType.kPosition);
                    setHoodYet = true;
                }
            }

            if ((hoodMotor.getEncoder().getPosition() > 34.5)
                    && (goUp == false)) {
                goDown = true;
            }

            if (goDown) {
                hoodMotor.set(-0.2);
                if ((hoodMotor.getEncoder().getPosition() < 34.5)) {
                    hoodMotor.set(0.0);
                    hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                            ControlType.kPosition);
                    setHoodYet = true;
                }
            }

            double thresholdVelocity = 9200.0 * (hangarShotSpeed / dumpShotSpeed);
            System.out.println("calculated: " + thresholdVelocity);
            System.out.println("real:" + masterShooterMotor.getSelectedSensorVelocity(1));
            if ((masterShooterMotor.getSelectedSensorVelocity(1) > thresholdVelocity) && setHoodYet) {
                if (Constants.xbox.getRawAxis(3) > 0.8) {
                    Robot.intake.indexerMotor.set(-0.8);
                } else {
                    Robot.intake.indexerMotor.set(0.0);
                }
            } else {
                Robot.intake.indexerMotor.set(0.0);
            }
        }
    }

    // manual hood control
    // notice also the use of if, else if, and else to make sure only one scenario is carried out
    // each time the method is called. explained further in centerRobotOnTarget()
    public void hoodControl() {
        // if not in process of shooting
        if (!shooting && !dumpShot && !lowDumpShot && !hangarShot) {
            // D-Pad on xbox controller is 0-360 degrees, where 0 is straight up. the values go up clockwise
            // if D-Pad is down
            if (Constants.xbox.getPOV() == 180) {
                // move hood down at 10% speed
                hoodMotor.set(-0.1);
                // if D-Pad up
            } else if (Constants.xbox.getPOV() == 0) {
                // move hood up at 10% speed
                hoodMotor.set(0.1);
            } else {
                // else set the hood speed to 0 and stop it from moving using PID (explained further in dumpShot())
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(), ControlType.kPosition);
            }
        }
    }

    // method that takes one double argument that is the desired hood set degree.
    // method will move hood to the desired degree.
    // useful for autonomous in preparing the hood when anticipating shooting from certain distances.
    // minimizes shooting preparation time.
    // this method uses the same code as shooting methods above for moving the hood up and down.
    // the code is explained in dumpShot.
    public void moveHood(double degree) {

        if ((hoodMotor.getEncoder().getPosition() < degree) && (goDown == false)) {
            goUp = true;
        }

        if (goUp) {
            hoodMotor.set(0.1);
            if ((hoodMotor.getEncoder().getPosition() > degree)) {
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                        ControlType.kPosition);
                setHoodYet = true;
            }
        }

        if ((hoodMotor.getEncoder().getPosition() > degree) && (goUp == false)) {
            goDown = true;
        }

        if (goDown) {
            hoodMotor.set(-0.1);
            if ((hoodMotor.getEncoder().getPosition() < degree)) {
                hoodMotor.set(0.0);
                hoodMotor.getPIDController().setReference(hoodMotor.getEncoder().getPosition(),
                        ControlType.kPosition);
                setHoodYet = true;
            }
        }
    }

}