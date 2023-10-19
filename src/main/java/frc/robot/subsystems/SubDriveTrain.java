// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.*;
import static frc.robot.Constants.MotorConstants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import static frc.robot.Constants.MiscConstants.*;

public class SubDriveTrain extends SubsystemBase {

  // Motor Controllers
  private BC_TalonSRX leftDriveMotor = new BC_TalonSRX(MOTOR_LEFT_MASTER);
  private BC_TalonSRX rightDriveMotor = new BC_TalonSRX(MOTOR_RIGHT_MASTER);

  private BC_TalonSRX leftFollowMotor = new BC_TalonSRX(MOTOR_LEFT_FOLLOWER);
  private BC_TalonSRX rightFollowMotor = new BC_TalonSRX(MOTOR_RIGHT_FOLLOWER);

  private DifferentialDrive driveTrain = new DifferentialDrive(leftDriveMotor, rightDriveMotor);

  private Solenoid gearSolenoid = new Solenoid(PCM_0, PneumaticsModuleType.CTREPCM, PCM_0_GEARSHIFT);

  private double maxSpeed = VELOCITY_SP_MAX_LG;

  // Navx setup and increasing the update rate default is 60 range is 4-200. Loading of the RobioRio should be monitored
  private AHRS ahrs = new AHRS(Port.kMXP, (byte)120);

  // Used to drive straight
  private float ahrsYawStraight = 0;
  private float ahrsPitchLevel = 0;

  private double directedRotation = 0;

  /** Creates a new SubDriveTrain. */
  public SubDriveTrain() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configure() {

    // Clear sticky faults
    leftDriveMotor.clearStickyFaults();
    leftFollowMotor.clearStickyFaults();
    rightDriveMotor.clearStickyFaults();
    rightFollowMotor.clearStickyFaults();

    // Set drive motor inverted
    leftDriveMotor.setInverted(false);
    rightDriveMotor.setInverted(true);

    // Set the left and right followers
    leftFollowMotor.setFollower(leftDriveMotor.getMotorController(), false);
    rightFollowMotor.setFollower(rightDriveMotor.getMotorController(), true);

    // Set the feedback sensor
    leftDriveMotor.setSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    rightDriveMotor.setSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

    // Set the current limits on the motors
    leftDriveMotor.setSupplyCurrentLimit(true,CONTINUOUS_CURRENT_LIMIT,PEAK_CURRENT_LIMIT,DURATION_CURRENT_LIMIT);
    rightDriveMotor.setSupplyCurrentLimit(true,CONTINUOUS_CURRENT_LIMIT,PEAK_CURRENT_LIMIT,DURATION_CURRENT_LIMIT);

    // Set nominal and peak outputs for the motor for the different profile slots
    leftDriveMotor.setNominalPeakOutput(0);
    rightDriveMotor.setNominalPeakOutput(0);
    leftDriveMotor.setNominalPeakOutput(1);
    rightDriveMotor.setNominalPeakOutput(1);

    // Set the ramp
    leftDriveMotor.setRamp(0.15);
    rightDriveMotor.setRamp(0.15);

    // Setup the Motion acceleration and velocity
    leftDriveMotor.configureMotionMagic(8192, 0);
    rightDriveMotor.configureMotionMagic(8192, 0);

    // Set max speed
    leftDriveMotor.setMaxSpeed(maxSpeed);
    rightDriveMotor.setMaxSpeed(maxSpeed);

    // PID constants for Profile 0 low gear Profile 1 high gear of Talon left
    SmartDashboard.putNumber("LEFT_KF_0",LEFT_KF_0);
    SmartDashboard.putNumber("LEFT_KP_0",LEFT_KP_0);
    SmartDashboard.putNumber("LEFT_KI_0",LEFT_KI_0);
    SmartDashboard.putNumber("LEFT_KD_0",LEFT_KD_0);

    leftDriveMotor.setConfig_kF(0, LEFT_KF_0);
    leftDriveMotor.setConfig_kP(0, LEFT_KP_0);
    leftDriveMotor.setConfig_kI(0, LEFT_KI_0);
    leftDriveMotor.setConfig_kD(0, LEFT_KD_0);

    SmartDashboard.putNumber("LEFT_KF_1",LEFT_KF_1);
    SmartDashboard.putNumber("LEFT_KP_1",LEFT_KP_1);
    SmartDashboard.putNumber("LEFT_KI_1",LEFT_KI_1);
    SmartDashboard.putNumber("LEFT_KD_1",LEFT_KD_1);

    leftDriveMotor.setConfig_kF(1, LEFT_KF_1);
    leftDriveMotor.setConfig_kP(1, LEFT_KP_1);
    leftDriveMotor.setConfig_kI(1, LEFT_KI_1);
    leftDriveMotor.setConfig_kD(1, LEFT_KD_1);

    // PID constants for Profile 0 low gear Profile 1 high gear of Talon Right

    SmartDashboard.putNumber("RIGHT_KF_0",RIGHT_KF_0);
    SmartDashboard.putNumber("RIGHT_KP_0",RIGHT_KP_0);
    SmartDashboard.putNumber("RIGHT_KI_0",RIGHT_KI_0);
    SmartDashboard.putNumber("RIGHT_KD_0",RIGHT_KD_0);

    rightDriveMotor.setConfig_kF(0, RIGHT_KF_0);
    rightDriveMotor.setConfig_kP(0, RIGHT_KP_0);
    rightDriveMotor.setConfig_kI(0, RIGHT_KI_0);
    rightDriveMotor.setConfig_kD(0, RIGHT_KD_0);

    SmartDashboard.putNumber("RIGHT_KF_1",RIGHT_KF_1);
    SmartDashboard.putNumber("RIGHT_KP_1",RIGHT_KP_1);
    SmartDashboard.putNumber("RIGHT_KI_1",RIGHT_KI_1);
    SmartDashboard.putNumber("RIGHT_KD_1",RIGHT_KD_1);

    rightDriveMotor.setConfig_kF(1, RIGHT_KF_1);
    rightDriveMotor.setConfig_kP(1, RIGHT_KP_1);
    rightDriveMotor.setConfig_kI(1, RIGHT_KI_1);
    rightDriveMotor.setConfig_kD(1, RIGHT_KD_1);
  }

  public void getPidFromDashboard() {
    // PID constants for Profile 0 low gear Profile 1 high gear of Talon left
    leftDriveMotor.setConfig_kF(0, SmartDashboard.getNumber("LEFT_KF_0",LEFT_KF_0));
    leftDriveMotor.setConfig_kP(0, SmartDashboard.getNumber("LEFT_KP_0",LEFT_KP_0));
    leftDriveMotor.setConfig_kI(0, SmartDashboard.getNumber("LEFT_KI_0",LEFT_KI_0));
    leftDriveMotor.setConfig_kD(0, SmartDashboard.getNumber("LEFT_KD_0",LEFT_KD_0));
  
    leftDriveMotor.setConfig_kF(1, SmartDashboard.getNumber("LEFT_KF_1",LEFT_KF_1));
    leftDriveMotor.setConfig_kP(1, SmartDashboard.getNumber("LEFT_KP_1",LEFT_KP_1));
    leftDriveMotor.setConfig_kI(1, SmartDashboard.getNumber("LEFT_KI_1",LEFT_KI_1));
    leftDriveMotor.setConfig_kD(1, SmartDashboard.getNumber("LEFT_KD_1",LEFT_KD_1));
  
    // PID constants for Profile 0 low gear Profile 1 high gear of Talon Right
    rightDriveMotor.setConfig_kF(0, SmartDashboard.getNumber("RIGHT_KF_0",RIGHT_KF_0));
    rightDriveMotor.setConfig_kP(0, SmartDashboard.getNumber("RIGHT_KP_0",RIGHT_KP_0));
    rightDriveMotor.setConfig_kI(0, SmartDashboard.getNumber("RIGHT_KI_0",RIGHT_KI_0));
    rightDriveMotor.setConfig_kD(0, SmartDashboard.getNumber("RIGHT_KD_0",RIGHT_KD_0));
  
    rightDriveMotor.setConfig_kF(1, SmartDashboard.getNumber("RIGHT_KF_1",RIGHT_KF_1));
    rightDriveMotor.setConfig_kP(1, SmartDashboard.getNumber("RIGHT_KP_1",RIGHT_KP_1));
    rightDriveMotor.setConfig_kI(1, SmartDashboard.getNumber("RIGHT_KI_1",RIGHT_KI_1));
    rightDriveMotor.setConfig_kD(1, SmartDashboard.getNumber("RIGHT_KD_1",RIGHT_KD_1));
  }

  // Drives the drivetrain with arcade drive given speed and rotation
  public void drive(double speed, double rotation) {
    driveTrain.setDeadband(0.02);
    driveTrain.arcadeDrive(speed, rotation, true);
  }

  /*  Drives the Drive train straight given speed                       */
  /*  use m_driveTrain->SetYawStraightValue(m_driveTrain->GetYaw());    */
  /*  to set the direction you want to go in                            */

  public void driveStraight(double speed) {
    double rotation = 0.0;
    double headingError = ahrsYawStraight - ahrs.getYaw();
    if(headingError > 0.0) {
      // Normalize for quadrant I
      rotation = (1.0 - ((180.0-(headingError))/180.0));
  //    if(headingError > 5)
  //    std::cout << "CmdDriveWithController>> headingError is: " << headingError  << std::endl;
    }
    if(headingError  < 0.0) {
      // Normailize for quadrant II
      rotation = (-1.0 + (180.0+(headingError))/180.0);
  //    if(headingError < -5)
  //      std::cout << "CmdDriveWithController>> headingError is: " << headingError  << std::endl;
    }
      
      // Correct for quadrents III and IV
    if(rotation > 1.0) {
      rotation = (rotation - 1.0) * -1.0;
      if(rotation > 1.0) {
  //      std::cout << "CmdDriveWithController>> ****** Wooooooo"  << std::endl;
      }
    }
    else if(rotation < -1.0) {
      rotation = (rotation + 1.0) * -1.0;
      if(rotation < -1.0) {
  //      std::cout << "CmdDriveWithController>> ****** Wooooooo"  << std::endl;
      }
    }

    // catch and avoid erratic jumps
    if(headingError > 300)
      rotation = -0.2;
    else if(headingError < -300)
      rotation = 0.2;

    // Add offset if needed
    if(rotation > 0.0 && rotation < 0.2) {
      rotation = rotation + 0.07;
    }
    else if(rotation < 0.0 && rotation > -0.2) {
      rotation = rotation - 0.07;
    }

    driveTrain.setDeadband(0.02);
    driveTrain.arcadeDrive(speed, rotation, true);
  }

  public void toggleDriveTrainGear() {
    boolean gear = gearSolenoid.get();
    gearSolenoid.set(!gear);
  }

  public void setMaxSpeed(double maxSpeed) {
    this.maxSpeed = maxSpeed;
    leftDriveMotor.setMaxSpeed(this.maxSpeed);
    rightDriveMotor.setMaxSpeed(this.maxSpeed);
  }

  public void setRamp(double ramp) {
    leftDriveMotor.setRamp(ramp);
    rightDriveMotor.setRamp(ramp);
  }

  public void driveByRotations(double leftRotations, double rightRotations) {
    // TODO: multiply input rotations by number of encoder ticks per revolution. 2048 is for falcons, we're using QuadEncoders
    leftRotations *= 2048;
    rightRotations *= 2048;

    leftDriveMotor.setMotionMagic(leftRotations);
    rightDriveMotor.setMotionMagic(rightRotations);
  }

  public void resetEncoderPositions() {
    leftDriveMotor.resetSensorPosition();
    rightDriveMotor.resetSensorPosition();
  }

  public void zeroYaw() {
    ahrs.zeroYaw();
  }

  public void setYawStraightValue(float yawValue) {
    ahrsYawStraight = yawValue;
  }

  public void setPitchLevelValue(float pitchValue) {
    ahrsPitchLevel = pitchValue;
  }

  public void setDirectedRotation(double rotation) {
    directedRotation = rotation;
  }

  public double getLeftEncoderValue() {
    return leftDriveMotor.getEncoderValue();
  }

  public double getRightEncoderValue() {
    return rightDriveMotor.getEncoderValue();
  }

  public double getLeftErrorValue() {
    return leftDriveMotor.getClosedLoopError();
  }

  public double getRightErrorValue() {
    return rightDriveMotor.getClosedLoopError();
  }

  public boolean getGear() {
    return gearSolenoid.get();
  }

  public float getYaw() {
    return ahrs.getYaw();
  }

  public float getYawStraightValue() {
    return ahrsYawStraight;
  }

  public float getPitchLevelValue() {
    return ahrsPitchLevel;
  }

  public double getDirectedRotation() {
    return directedRotation;
  }
}
