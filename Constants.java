package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class Constants {

  // Index
  // all CAN index numbers except for last two
  public static int flMotorIndex = 6;
  public static int blMotorIndex = 21;
  public static int frMotorIndex = 20;
  public static int brMotorIndex = 13;

  public static int masterShooterMotorIndex = 10;
  public static int slaveShooterMotorIndex = 11;
  public static int hoodMotorIndex = 3;

  public static int intakeMotorIndex = 24;
  public static int indexerMotorIndex = 1;
  public static int intakeAngleMotorIndex = 8;

  public static int centerClimberHeightMotorIndex = 23;

  // joy stick index numbers are found in driver station
  public static int stickIndex = 0;
  public static int xboxIndex = 1;

  //////////////////////////////////////////////////////
  // Controllers

  // Joystick objects. one for joy stick, the other for the xbox controller.
  // the one argument is the driver station joy stick index number
  public static Joystick stick = new Joystick(stickIndex);
  public static Joystick xbox = new Joystick(xboxIndex);

  //////////////////////////////////////////////////////
  // Functions

  // I honestly forget how I wrote this method and came up with this formula, but it does work.
  // explanation in DriveTrain class
  public static double quadraticPositionAndSpeed(double minimumMotorSpeed, double maximumMotorSpeed,
      double positionGoal, double currentPosition) {

    double a = ((positionGoal * maximumMotorSpeed - minimumMotorSpeed * positionGoal
        - minimumMotorSpeed * (positionGoal / 2) + minimumMotorSpeed * (positionGoal / 2))
        / (positionGoal * (positionGoal / 2) * ((positionGoal / 2) - positionGoal)));

    double b = ((maximumMotorSpeed - a * (positionGoal / 2) * (positionGoal / 2) - minimumMotorSpeed)
        / (positionGoal / 2));

    double speed = a * currentPosition * currentPosition + b * currentPosition + minimumMotorSpeed;

    return speed;
  }

  
}