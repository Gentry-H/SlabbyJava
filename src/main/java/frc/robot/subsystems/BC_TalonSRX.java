// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class BC_TalonSRX implements MotorController {

  public TalonSRX talonSRX = null;

  private double speed = 0;
  private boolean selectedGear = false;
  private double maxSpeed = 1500;

  /** Creates a new BC_TalonSRX. */
  public BC_TalonSRX(int device) {
    talonSRX = new TalonSRX(device);
  }

  // Speed is a percentage as a decimal between 0 and 1 (1 = 100%)
  public void set(double speed) {

    // Run in low gear
    if(!selectedGear) {
      speed *= maxSpeed;
      talonSRX.set(ControlMode.Velocity, speed);
      this.speed = speed;
      talonSRX.selectProfileSlot(0, 0);
    } else { // Run in high gear
      speed *= 3200;
      talonSRX.set(ControlMode.Velocity, speed);
      this.speed = speed;
      talonSRX.selectProfileSlot(1, 0);
    }
  }

  public void setMotionMagic(double position) {
    talonSRX.set(ControlMode.MotionMagic, position);
  }

  public void setInverted(boolean isInverted) {
    talonSRX.setInverted(isInverted);
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    talonSRX.setNeutralMode(neutralMode);
  }

  // Takes a boolean, true is high gear
  public void setGear(boolean gear) {selectedGear = gear;}

  public void setMaxSpeed(double maxSpeed) {this.maxSpeed = maxSpeed;}

  public void resetSensorPosition() {
    talonSRX.setSelectedSensorPosition(0, 0, 0);
  }

  public void setConfig_kF(int profileSlot, double value) {
    talonSRX.config_kF(profileSlot, value, 0);
  }

  public void setConfig_kP(int profileSlot, double value) {
    talonSRX.config_kP(profileSlot, value, 0);
  }

  public void setConfig_kI(int profileSlot, double value) {
    talonSRX.config_kI(profileSlot, value, 0);
  }

  public void setConfig_kD(int profileSlot, double value) {
    talonSRX.config_kD(profileSlot, value, 0);
  }

  public void clearStickyFaults() {
    talonSRX.clearStickyFaults();
  }

  public void setFollower(TalonSRX motor, boolean isInverted) {
    talonSRX.follow(motor);
    talonSRX.setInverted(isInverted);
  }

  public void setSelectedFeedbackSensor(FeedbackDevice device) {
    talonSRX.configSelectedFeedbackSensor(device,0,0);
  
  }
  
  public void setSupplyCurrentLimit(boolean enable, double limit, double trigger, double triggerTime) {
    talonSRX.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(enable, limit, trigger, triggerTime));
  }

  public void setNominalPeakOutput(int profileSlot) {
    talonSRX.selectProfileSlot(profileSlot,0);
    talonSRX.configNominalOutputForward(0,0);
    talonSRX.configNominalOutputReverse(0,0);
    talonSRX.configPeakOutputForward(1.0,0);
    talonSRX.configPeakOutputReverse(-1.0,0);
  }

  public void configureMotionMagic(int acceleration, int cruiseVelocity) {
    talonSRX.configMotionAcceleration(acceleration, 0);
    talonSRX.configMotionCruiseVelocity(cruiseVelocity, 0);
  }

  public void setRamp(double ramp) {
    talonSRX.configClosedloopRamp(ramp, 0);
  }

  public TalonSRX getMotorController() {
    return talonSRX;
  }

  public double getSpeed() {return this.speed;}

  public boolean getInverted() {return talonSRX.getInverted();}

  public double getClosedLoopError() {
    return talonSRX.getClosedLoopError(0);
  }

  public double getClosedLoopTarget() {
    return talonSRX.getClosedLoopTarget(0);
  }

  public boolean getGear() {return selectedGear;}

  public double getMaxSpeed() {return this.maxSpeed;}

  public double getEncoderValue() {return talonSRX.getSelectedSensorPosition(0);}

  @Override
  public double get() {
    // TODO Auto-generated method stub
    return 0;
  }

  @Override
  public void disable() {
    // TODO Auto-generated method stub
    
  }

  @Override
  public void stopMotor() {
    // TODO Auto-generated method stub
    
  }
}
