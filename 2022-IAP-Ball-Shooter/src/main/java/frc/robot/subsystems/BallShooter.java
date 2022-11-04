// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class BallShooter extends SubsystemBase {
  /** Creates a new BallShooter. */
  private WPI_TalonSRX flyWheel = new WPI_TalonSRX(Constants.flyWheelID);
  private WPI_TalonSRX feedWheel = new WPI_TalonSRX(Constants.feedWheelID);


  public BallShooter() {
    flyWheel.configFactoryDefault();
    feedWheel.configFactoryDefault();

    flyWheel.setInverted(false);
    feedWheel.setInverted(false);

    flyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }


  public void startFly(double power){
    flyWheel.set(ControlMode.PercentOutput, power);
  }
  public void startFeed(double power){
    feedWheel.set(ControlMode.PercentOutput, power);
  }

  public void resetEncoder(){
    flyWheel.setSelectedSensorPosition(0);
  }



  @Override
  public void periodic() {
    if(RobotContainer.getJoystick1().getRawButtonPressed(1)){
      startFly(0.5);
    }
    else if(RobotContainer.getJoystick1().getRawButtonPressed(10)){
      startFly(0.0);
    }
    if(RobotContainer.getJoystick2().getTriggerPressed()){
      startFeed(0.5);
    }
    else{
      startFeed(0.0);
    }
  }
}
