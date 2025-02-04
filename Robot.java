// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;


//import com.pathplanner.lib.PathPlanner;
/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  //Drivetrain
 SparkMax leftdrive1 = new SparkMax(1, MotorType.kBrushless);
  SparkMax leftdrive2 = new SparkMax(2, MotorType.kBrushless);
 SparkMax rightdrive1 = new SparkMax(3, MotorType.kBrushless);
 SparkMax rightdrive2 = new SparkMax(4, MotorType.kBrushless);
  DifferentialDrive drive = new DifferentialDrive(leftdrive1, rightdrive1);

  //inputs
  Joystick driverstick = new Joystick(0);
  Joystick opstick = new Joystick(1);

  //shooter
  static SparkMax shooterMotor1 = new SparkMax(5, MotorType.kBrushless);
   static SparkMax shooterMotor2 = new SparkMax(6, MotorType.kBrushless);
  Robot shoot = new Robot();
    public static void shoot(Joystick driverstick) {
      if (driverstick.getTriggerPressed()) {
      shooterMotor1.set(.25);
      shooterMotor2.set(-.25);
    }
  }


/**
 * Initializes a Victor motor controller on PWM port 0.
 * This motor controller is named redlineTalon1.
 */


/*public void setSparkMax(Spark leftShootSpark1) {
   leftShootMax1 = rightShootMax1;*/






//arm
//Commented out for coding later on
/*yourActuator = new Servo(RobotMap.YOUR_ACTUATOR_CHANNEL);
yourActuator.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
yourActuator.setSpeed(1.0); // to open
yourActuator.setSpeed(-1.0); // to close


wrist
 

 


/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(driverstick.getX(), driverstick.getY());
   
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
