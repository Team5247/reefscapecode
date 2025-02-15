// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

 
  //arm
Talon redline1 = new Talon(0);
Talon redline2 = new Talon(1);
DigitalInput toplimitSwitch = new DigitalInput(0);
DigitalInput bottomlimitSwitch = new DigitalInput(1);
//wrist
linearservo servo = new linearservo();
@Override
public void robotPeriodic() {
    //SmartDashboard.putNumber("Power", shootPower);
    SmartDashboard.putBoolean("High Limit", toplimitSwitch.get());
    SmartDashboard.putBoolean("Low Limit", bottomlimitSwitch.get());
}


/**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {} 

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic(){
    drive.arcadeDrive(driverstick.getX(), driverstick.getY());
    double articulatePower = 0.0;
        articulate(articulatePower);
                    
                         if (opstick.getRawButtonPressed(2)) {
                          shooterMotor1.set(.25);
                         shooterMotor2.set(.25);
                        }
                      if (opstick.getRawButtonReleased(2)){
                      shooterMotor1.set(0);
                      shooterMotor2.set(0);
                    }
                      if (opstick.getRawButtonPressed(1)){
                     shooterMotor1.set(.25);
                     shooterMotor2.set(.25);
                    }
                        if (opstick.getRawButtonReleased(1)){
                          shooterMotor1.set(0);
                          shooterMotor2.set(0);
                        }
                      }
                   public void articulate(double speed) {
                    if (opstick.getRawButton(7)) {
                      toplimitSwitch.get();
                      redline1.set(.25);
                      redline2.set(.25);
                    } else {
                      toplimitSwitch.close();
                      redline1.set(0);
                      redline2.set(0);
                    }
                    if (opstick.getRawButton(9)) {
                      bottomlimitSwitch.get();
                      redline1.set(-.25);
                      redline2.set(-.25);
                    } else {
                    bottomlimitSwitch.close();
                      redline1.set(0);
                      redline2.set(0);
                    }
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
