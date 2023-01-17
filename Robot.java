package frc.robot; // the package statement goes at the top
// of each file that indicates where each java file is in the file system

// import section. This goes after the package statement
// all libraries used in Robot.java. This will be different for every file
// it's best not to have any unused libraries in the import section
// because it's unnecessary clutter in your file and could
// unnecessarily use more processing power for your computer and rio
// this is also true for extra classes, constructors, variables, and methods.
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticHub;
import com.kauailabs.navx.frc.*; // the ".*" at the end here means
// that all classes in the referenced folder will be imported.
// otherwise, just the one specific class
// in the one specific folder referenced in the statement
// will be imported.

// after installing the non-First libraries for hardware like the CTRE motors
// and rev robotics stuff, you can use them in your project by right-clicking
// build.gradle in the file panel on the left
// and selecting "Manage Vendor Libraries". That should bring up a menu
// at the top of your screen that will allow you to select and add the libraries
// you want to your project.


// the Robot class will automatically be created for you when you create
// a new java project in VS Code. Below is the class declaration
// don't change anything here
// class names (and constructors (see below)) are always capitalized.
// variables and methods use camel case i.e. firstSecondThird.
// the first word is not capitalized, and the following words are.
// no spaces are allowed in any names. underscores are fine.
public class Robot extends TimedRobot {

  // here is the object initialization section
  // it generally goes right below the class declaration
  // and usually before the method section.

  // objects are instances of classes.
  // the other classes in this project i.e. the Shooter class
  // have been made into objects.
  // classes are like an idea or template for something
  // Objects are the real thing made from that template.
  // the shooter class is the idea of a shooter.
  // I create a real shooter object here that I can use
  // to control the real shooter on the robot.

  // some classes are made by the programmer. others are made
  // by first in the wpilib libraries or by hardware manufacturers
  // like ctre (cross the road electronics) and rev robotics

  // classes that you make you don't have the import
  // in the import section above as long as your classes are
  // in the same package (in this case frc.robot).

  // an object is also a variable.
  // it's fine in this context to just make everything public and static.
  // public means it can be referenced from anywhere in the code
  // when a variable is static it means that there doesn't have to be
  // an object of the class that it is in for the variable to be referenced.
  // this makes referencing these same objects/variables in other classes.
  // these variables below are here because Robot.java is a central place
  // for all of them
  static Shooter shooter = new Shooter();
  static DriveTrain drive = new DriveTrain();
  static Intake intake = new Intake();
  static Climber climber = new Climber();

  // these three classes below are made by hardware manufacturers
  // the parantheses part at the end refers to a constructor. Each class
  // may require different pieces of information that are called either arguments or parameters
  // in order to create an object from it.
  // PowerDistribution requires the CAN index first
  // and the ModuleType second (either rev or ctre)
  static PowerDistribution PDP = new PowerDistribution(6, ModuleType.kRev);
  // PneumaticHub requires the CAN index
  static PneumaticHub ph = new PneumaticHub(15);
  // AHRS requires nothing
  static AHRS gyro = new AHRS();
  // each class could have multiple different constructors that make the object
  // do different things depending on which constructor is used.
  // in order to look at a class, hold the control key and hover the mouse
  // over the class name. First has good documentation for their classes.
  // They are normally notated well with descriptions of each part of the class
  // Constructors that you can use will start like this:
  // "public [Class name]([arguments, if applicable]){"


  // Methods section is below, with the exeption of a few variables for convenience
  // some methods are created by me, some are necessary parts of Robot.java
  // that are called by the rio or DriverStation automatically

  // I created this method in order to have one easy way to clear all faults
  // across the entire robot. It references every single applicable system
  // and executes its respective clear faults method.
  public void clearStickyFaults() {
    PDP.clearStickyFaults();
    ph.clearStickyFaults();

    shooter.masterShooterMotor.clearStickyFaults();
    shooter.slaveShooterMotor.clearStickyFaults();
    shooter.hoodMotor.clearFaults();

    drive.flMotor.clearFaults();
    drive.frMotor.clearFaults();
    drive.blMotor.clearFaults();
    drive.brMotor.clearFaults();

    intake.intakeMotor.clearFaults();
    intake.indexerMotor.clearFaults();
  }

  // a true or false variable that is initialized when the robot turns on
  static boolean isRed;

  // variable that controls which stage of autonomous the robot is in.
  // changes automatically with code throughout autonomous
  int autoStage = 0;
  // keeps time during autonomous for execution of the different stages
  // at the right time
  Timer autonomousTimer = new Timer();

  ///////////////////////////////////////////////////////
  // Robot

  // this method runs once when the robot is turned on
  @Override // override comes from the fact that this class Robot
  // you're working in is a model created by First. The original version
  // of robotInit() is the class TimedRobot or even below that in the
  // derivation tree. This class is a derivation of TimedRobot, as explained
  // by "extends TimedRobot" in the class declaration above.
  // the @Override statement is actually not necessary, and will automatically
  // be put above methods from First
  public void robotInit() {
    clearStickyFaults(); // described above
    // method from the DriveTrain class, executed using the drive object,
    // which is an object from the DriveTrain class.
    // the first part of the statement below is "drive", so we know we
    // are referencing the "drive" object. The "." separates the
    // subject from the verb, in a sense. The subject "drive"
    // executes the verb "driveTrainInit()".
    // methods, just like the constructor, have arguments/parameters contained
    // within parantheses.
    drive.driveTrainInit();
    // method from the shooter class, executed using the shooter object,
    // which is an object from the Shooter class
    shooter.shooterRobotInit();
    // method from the Climber class, executed using the climber object,
    // which is an object from the Climber class
    climber.climberInit();
    // method from the Intake class, executed using the intake object,
    // which is an object from the Intake class
    intake.intakeInit();

    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      isRed = true;
    } else {
      isRed = false;
    }

  }

  // called every 15 ms, regardless of mode
  @Override
  public void robotPeriodic() { 
    SmartDashboard.putString("Pressure", (ph.getPressure(0)) + " psi");
  }

  ////////////////////////////////////////////////////////
  // Autonomous

  // runs once when autonomous starts
  @Override
  public void autonomousInit() {
    clearStickyFaults(); // explained above
    autonomousTimer.reset(); // sets the timer to 0.0 s
    autonomousTimer.start(); // starts timer
    autoStage = 0; // sets autonomous stage to 0
    shooter.shooterInit(); // see Shooter class
    shooter.resetHoodEncoders(); // see Shooter class
    // true or false variable that describes whether the intake
    // is literally up or down on the robot
    intake.isIntakeDown = false;
    // true or false variable that describes whether the level 2 climber
    // is literally up or down on the robot
    climber.isLevel2ClimberDown = true;
    // turns off the compressor
    ph.disableCompressor();
    // two commands that make the gyro ready for use during autonomous.
    // use both in the init code for any mode in which you want to use the gyro
    gyro.calibrate();
    gyro.zeroYaw();
  }

  // @Override
  // public void autonomousPeriodic(){
  // System.out.println(gyro.getYaw());
  // // drive.driveTrainByControls(0.0, 0.0, , false);
  // drive.mecanumDrive.driveCartesian(0.0, 0.0,
  // Constants.quadraticPositionAndSpeed(0.1, 0.5, 90.0, gyro.getYaw()));

  // }

  // this was not used in the final code. something I experimented with
  Timer turningTimer = new Timer();

  // called every 15 ms during the autonomous period
  @Override
  public void autonomousPeriodic(){
    // method that is always running. Shooting starts when the
    // dumpShot variable is set to true.
    shooter.dumpShot();
    shooter.shooterIdle(0.20); // shooter speed idles at 20%.
    // prints the time in seconds to the console and riolog.
    // console is in DriverStation (make sure you have +prints enabled).
    // riolog is in VS Code. Search for it in the top right with the W button
    System.out.println("timer: " + autonomousTimer.get());
    // prints yaw degrees from the gyro to the console and riolog
    System.out.println("yaw: " + gyro.getYaw());

    // a switch statement statement has one argument, usually a number. it allows
    // you to have multiple different situations or cases. In this case,
    // I've used the variable autonomousStage. 
    // Only one case is carried out every time the switch statement is called
    // it depends on the value of the argument
    // the value changes during autonomous, and so the value of the argument
    // is checked every time the switch statement runs
    // I used a switch statement because it makes it easy to see
    // the order of execution of the autonomous mode because it's presented linearly.
    switch(autoStage){

      // robot is reset and ready to shoot and go. Intake is put down
      case 0 : { // standard switch statement syntax is "case [argument value] : {}"
        autoStage = 1; // sets the autonomous stage to 1
        drive.stopMotors(); // stops drive train motors. see DriveTrain class
        // resets drive train encoder values see DriveTrain class
        drive.resetDriveTrainEncoders();
        intake.intakeDown(); // puts intake down. see Intake class
        shooter.shootInit(); // makes the shooter ready to shoot. See Shooter class
        break; // break is necessary at the end of every case 
      }

      // robot shoots one ball
      case 1 : {
        // activates the dumpShot() method. Robot shoots one ball automatically
        shooter.dumpShot = true;
        // if 1.0 second has passed. Time for the intake to physically drop down
        if(autonomousTimer.get() > 1.0){
          autoStage = 2; // sets the autonomous stage to 2
          // deactivates the dumpShot() method. robot stops shooting
          shooter.dumpShot = false;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
          // sets the intake motor to 65%.
          // Speeds are from -1.0 to 1.0. This is 65% in the negative direction.
          // in this case, negative meant intake in a ball. It all depends on how
          // the robot is built and wired which way is which. Find out
          // with trial and error.
          intake.intakeMotor.set(-0.65);
        }
        break;
      }

      // robot drives forward 40.0 inches (intake is forward).
      // Intake is running so the ball in front of the intake is picked up.
      case 2 : {
        // robot drives forward 40.0 inches (intake is forward).
        // first argument = number of inches. second argument = direction.
        // see DriveTrain class
        drive.driveTrainByInches(40.0, 0);
        // after 5.0 seconds into autonomous, robot is done driving.
        // move on to next stage.
        if(autonomousTimer.get() > 5.0){
          autoStage = 4;
          // every time you want to drive with the driveTrainByInches method,
          // which is based directly off encoders,
          // call stopMotors() and resetDriveTrainEncoders(), ideally in that order.
          // this ensures you're starting at encoder counts of 0 every time you
          // want to drive a certain distance
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      // drive back towards hub
      case 4 : {
        drive.driveTrainByInches(35.0, 1);
        if(autonomousTimer.get() > 9.0){
          autoStage = 5;
          drive.stopMotors();
          drive.resetDriveTrainEncoders();
        }
        break;
      }

      // center robot. when centered, shoot until autonomous is over
      case 5 : {
        if(shooter.centerRobotOnTarget()){
          shooter.dumpShot = true;
        }
        break;
      }
    }

  }

  ///////////////////////////////////////////////////////////
  // Tele-operated

  // for drivers. Allows time to be printed to SmartDashboard
  Timer teleopTimer = new Timer();

  @Override
  public void teleopInit() {
    clearStickyFaults();
    shooter.shooterInit();
    intake.isIntakeDown = true;
    climber.isLevel2ClimberDown = true;
    // true or false variable describing whether the traverse climber hooks
    // are literally activated. activated means clamped
    climber.isTraverseClimberActivated = false;
    // enables the compressor to run until the pressure is in the 115-120 psi range
    // first argument = min pressure. second argument = max pressure.
    // max pressure cannot be greater than 120, according to code and rules
    ph.enableCompressorAnalog(115, 120);
    teleopTimer.reset(); // sets teleop timer to 0.0 s
    teleopTimer.start(); // starts teleop timer
    // true or false variable describing whether the color sensor
    // is being used for intaking
    intake.intakingByColor = true;
  }

  // runs every 15 ms during the teleop period
  @Override
  public void teleopPeriodic() {
    // accesses a numerical field in SmartDashboard called timer. The value in the field
    // is DriverStation.getMatchTime() which is equal to how much time is left in the match.
    // all further manipulation will be in SmartDashboard
    SmartDashboard.putNumber("Timer", DriverStation.getMatchTime());

    // if pressing and holding button 2 on joystick or pushing axis 2 on xbox controller
    // beyond 50%, activate centering
    if (Constants.stick.getRawButton(2) || (Constants.xbox.getRawAxis(2) > 0.5)) {
      shooter.centerRobotOnTarget();
    } 

    // same format for the distancing method
    // else if (Constants.stick.getRawButton(3) || Constants.xbox.getRawButton(6)) {
    //   shooter.distanceRobotFromTargetCorrectly();
    // } 

    // if not centering or distancing, drive normally with the driveTrainByControls() method
    // see DriveTrain class
    else {
      drive.driveTrainByControls(Constants.stick.getRawAxis(1), Constants.stick.getRawAxis(0),
          Constants.stick.getRawAxis(2));
    }

    // if button 5 on the xbox controller is pressed, set hoodEncoder to 10.5.
    // hood motor encoders are in degrees.
    // the hood all the way down is approximately 10.5 degrees.
    // this code is useful if the hood encoder value is messed up and requires
    // manual resetting during the match using hood manual control.
    // a driver would manually put the hood down while not shooting then press this button
    if(Constants.xbox.getRawButtonPressed(5)){
      shooter.hoodMotor.getEncoder().setPosition(10.5);
    }

    // getRawButton() returns true if the button is being pressed and held
    // getRawButtonPressed() returns true one time each time the button is pressed and released

    // all of these methods are called each time teleopPeriodic() is called
    // they contain all of the possible actions of the robot
    // each of these methods checks for driver input and executes accordingly.
    intake.intakeTeleop();
    //shooter.shoot();
    shooter.dumpShot();
    shooter.lowDumpShot();
    // shooter.hangarShot();
    shooter.shooterIdle(0.0);
    shooter.hoodControl();
    climber.climberTeleop();
  }

  /////////////////////////////////////////////////////////////
  // Test

  // runs once when test mode is activated. not a part of regular match play
  @Override
  public void testInit() {

    shooter.shooterInit();
    autonomousTimer.reset();
    autonomousTimer.start();
    ph.enableCompressorAnalog(115, 120);
  }

  // called every 15 ms during test mode. not a part of regular match play.
  // lots of test or diagnostic code
  @Override
  public void testPeriodic() {
    shooter.masterShooterMotor.set(0.0);
    intake.indexerMotor.set(0.0);
    // if(Constants.stick.getRawButton(1)){
    // canSparkMax.set(0.2);
    // } else if(Constants.stick.getRawButton(2)){
    // canSparkMax.set(-0.2);
    // } else {
    // canSparkMax.set(0.0);
    // canSparkMax.getPIDController().setReference(canSparkMax.getEncoder().getPosition(),
    // ControlType.kPosition);
    // }

    // System.out.println("intakeUpSolenoid: " + Robot.intake.intakeUp.get());
    // System.out.println("intakeDownSolenoid: " + Robot.intake.intakeDown.get());

    // if(Constants.stick.getRawButtonPressed(1)){
    // Robot.intake.intakeUp.set(false);
    // Robot.intake.intakeDown.set(true);
    // }

    // if(Constants.stick.getRawButtonPressed(2)){
    // Robot.intake.intakeDown.set(false);
    // Robot.intake.intakeUp.set(true);
    // }

    // intake.intakeAngleMotor.set(0.2);
    // System.out.println(intake.intakeAngleMotor.get());
    // shooter.masterShooterMotor.set(ControlMode.Velocity, 0.2,
    // DemandType.ArbitraryFeedForward, 0.5);
    // shooter.masterShooterMotor.pid
    // shooter.masterShooterMotor.set(ControlMode.Velocity, 1500.0);
    // System.out.println(shooter.masterShooterMotor.getSelectedSensorVelocity(1));

    // intake.indexByColor();

    Color detectedColor = intake.indexerColorSensor.getColor();
    System.out.println(
    "red: " + detectedColor.red + ", blue: " + detectedColor.blue + ", green: " +
    detectedColor.green);

    // // if (!Robot.shooter.shooting && !Robot.shooter.dumpShot) {
    // // if ((Math.abs(detectedColor.red - 0.32) < 0.02) ||
    // (Math.abs(detectedColor.blue - 0.305) < 0.02)) {
    // // intake.indexerMotor.set(0.0);
    // // } else {
    // // intake.indexerMotor.set(-0.2);
    // // }
    // // }

    // intake.indexByColor();

    // System.out.println(doubleSolenoid.get());

    // if(Constants.stick.getRawButton(1)){
    // intake.intakeAngleMotor.set(-0.5);
    // } else if (Constants.stick.getRawButton(2)) {
    // intake.intakeAngleMotor.set(0.5);
    // } else {
    // intake.intakeAngleMotor.set(0.0);
    // }

    // System.out.println(solenoid0.get());
    // System.out.println(solenoid1.get());
    // System.out.println("*******************************");

    // if(Constants.xbox.getRawButtonPressed(7)){
    // solenoid1.set(true);
    // // if(solenoid0.get()){
    // // solenoid0.set(false);
    // // solenoid1.set(true);
    // // } else if(solenoid1.get()){
    // // solenoid1.set(false);
    // // solenoid0.set(true);
    // // }
    // //solenoid1.set(true);
    // //doubleSolenoid.toggle();
    // }

    // if(Constants.xbox.getRawButtonPressed(8)){
    // solenoid0.set(true);
    // }

    // if(Constants.xbox.getRawButtonPressed(2)){
    // ph.fireOneShot(1);
    // }
    // ph.getCompressorConfigType()
    // ph.
    // System.out.println(ph.get);

    // if(autonomousTimer.get() > 10.0){
    // doubleSolenoid.set(Value.kReverse);
    // autonomousTimer.reset();
    // autonomousTimer.start();
    // }

    // System.out.println("intake angle motor encoders: " +
    // intake.intakeAngleMotor.getEncoder().getPosition());

    // still have the intake up and down stuff. Move at 0.5 speed until get within a
    // certain number of encoder counts of the goal
    // then PID
    // have buttons that go up and down at 0.35 or something and if you press the
    // buttons, you still switch between intake up and down
    // that finish the way

    // could double check distance calculation

    //////////////////////////////////////////////////////////////////////////
    // Color detectedColor = intake.indexerColorSensor.getColor();
    // System.out.println("blue: " + new Color(0.0, 0.290, 0.0).blue);
    // //double difference = Math.abs(detectedColor.blue - 0.2975); // or 0.295. <
    ////////////////////////////////////////////////////////////////////////// 0.015
    // double difference = Math.abs(detectedColor.red - 0.295); // 0.2932 // 0.2976
    // System.out.println("difference: " + difference);

    // if (difference < 0.025){
    // intake.indexerMotor.set(0.0);
    // } else {
    // intake.indexerMotor.set(-0.2);
    // }

    // ColorMatchResult match = intake.colorMatch.matchClosestColor(detectedColor);
    // System.out.println(match.color.red + " " + match.color.blue + " " +
    // match.color.green);

    // if(Constants.stick.getRawButtonPressed(1)){
    // shooter.shooting = !shooter.shooting;
    // }

    // if ((Math.abs(detectedColor.red - new Color(0.195, 0.374, 0.432).red) < 0.05)
    // && detectingBall) {
    // ballPresent = true;
    // intakeTimer.reset();
    // intakeTimer.start();
    // } else {
    // if(!ballPresent){
    // intake.indexerMotor.set(-0.2);
    // }
    // }

    // if(ballPresent){
    // detectingBall = false;
    // intake.indexerMotor.set(-0.2);
    // if(intakeTimer.get() > 0.6){
    // intake.indexerMotor.set(0.0);
    // if(shooter.shooting || shooter.dumpShot || (Constants.xbox.getPOV() == 180)){
    // detectingBall = true;
    // ballPresent = false;
    // }
    // }
    // }

    // // if(ballPresent){

    // // } else {

    // // }

    // // else {
    // // ballPresent = false;
    // // intake.indexerMotor.set(-0.2);
    // // }

    // if(ballPresent){

    // }
    /////////////////////////////////////////////////////////////////////

    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, 0.87);
    // System.out.println("calculated " +
    // (shooter.shooterWheelLinearVelocityToMotorVelocity(shooter.calculatedVelocity)));
    // System.out.println("actual speed " +
    // (shooter.masterShooterMotor.getSelectedSensorVelocity()));
    // System.out.println("calculated angle: " + (90.0 - (shooter.calculatedAngle *
    // 180.0 / Math.PI)));
    // System.out.println("actual angle: " +
    // shooter.hoodMotor.getEncoder().getPosition());

    // if(Constants.stick.getRawButtonPressed(5)){
    // shooter.angleError -= 0.05;
    // } else if(Constants.stick.getRawButton(6)){
    // shooter.angleError += 0.05;
    // } else if(Constants.stick.getRawButton(3)){
    // shooter.angleError -= 0.01;
    // } else if(Constants.stick.getRawButtonPressed(4)){
    // shooter.angleError += 0.01;
    // }

    // //System.out.println(shooter.getXDistanceFromCenterOfHub(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.getXDistanceFromFrontOfRobotToFender(NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0)));
    // System.out.println(shooter.angleError);

    // public static float getXDistanceFromCenterOfHub(double verticalAngle) { // x
    // // distance from front of robot to fender. better for testing
    // return (float) (((y - cameraHeight) / Math.tan(Math.toRadians(verticalAngle +
    // cameraAngle - angleError))) - horizontalDistanceFromLimelightToFrontOfRobot -
    // distanceFromFenderToTape);
    // }

    // if(Constants.stick.getRawButtonPressed(7)){
    // shooterActivated = !shooterActivated;
    // }
    // if(Constants.stick.getRawButtonPressed(11)){ // adjust shooter speed
    // shooterActivated = true;
    // shooterSpeed -= 0.05;
    // }
    // if(Constants.stick.getRawButtonPressed(12)){
    // shooterActivated = true;
    // shooterSpeed += 0.05;
    // }
    // if(shooterActivated){
    // shooter.masterShooterMotor.set(ControlMode.PercentOutput, shooterSpeed);
    // } else {
    // shooter.masterShooterMotor.set(0.0);
    // }
  }

  //////////////////////////////////////////////////////////////
  // Disabled

  // runs once when the robot is disabled
  @Override
  public void disabledInit() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setDouble(1.0);
  }

  // called every 15 ms while the robot is disabled
  @Override
  public void disabledPeriodic() {
  }

}