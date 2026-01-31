// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  private CANcoder encoder = new CANcoder(2);
  private TalonFX motor = new TalonFX(3);

  private double kp = 2;
  private PIDController pidController = new PIDController(kp, 0, 0);
  private XboxController xboxController = new XboxController(0);
  

  //runs once when robot turns on
  public Robot() {
    pidController.enableContinuousInput(-0.5, 0.5 );
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
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    double x = -xboxController.getRawAxis(1);
    double y = -xboxController.getRawAxis(0);
    double degree = Math.atan2(x,y);
    degree /= (2*Math.PI);
    degree -= 0.25;
    if(degree < -0.5) 
      degree += 1;
    System.out.print(degree);

    //double error = encoder.getAbsolutePosition().getValueAsDouble() - degree;
    motor.set(-pidController.calculate(encoder.getAbsolutePosition().getValueAsDouble(), degree));

  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
