// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubDriveTrain;
import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.MotorConstants.*;

public class CmdDriveWithController extends CommandBase {

  private SubDriveTrain subDriveTrain;
  private Joystick driverController;

  /** Creates a new CmdDriveWithController. */
  public CmdDriveWithController(SubDriveTrain subDriveTrain, Joystick driverController) {
    this.subDriveTrain = subDriveTrain;
    this.driverController = driverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subDriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Speed as a percentage
    double speed = 0;

    // This sets up the virtual low gear
    if(driverController.getRawButton(BUTTON_B)) {
      subDriveTrain.setMaxSpeed(VELOCITY_SP_MAX_LL);
    } else {
      subDriveTrain.setMaxSpeed(VELOCITY_SP_MAX_LG);
    }

    // Sets the speed based on the driver controller triggers
    if (driverController.getRawAxis(AXIS_R_TRIG) > 0)  {
      speed = -1*driverController.getRawAxis(AXIS_R_TRIG);
    } else {
      speed = driverController.getRawAxis(AXIS_L_TRIG);
    }

    // Set the rotation
    double rotation = 0;

    // Booleans for detecting if we want to move the robot
    boolean isMoveDesired = (speed > 0.02 || speed < -0.02);
    boolean isTurnDesired = (driverController.getRawAxis(AXIS_LX) > 0.02 || driverController.getRawAxis(AXIS_LX)  < -0.02);

    // is the controller telling us to turn?
    if(isTurnDesired) {
      subDriveTrain.setYawStraightValue(subDriveTrain.getYaw());

      if(subDriveTrain.getGear()) {
        rotation = driverController.getRawAxis(AXIS_LX)*0.7;
      } else {
        rotation = driverController.getRawAxis(AXIS_LX)*0.8;
      }
    } else if(isMoveDesired) {

//      double headingError = m_driveTrain->GetYawStraightValue() - m_driveTrain->GetYaw();
//
//      if(headingError > 0.0) {
//        // Normalize for quadrant I
//        rotation = (1.0 - ((180.0-(headingError))/180.0));
//        if(headingError > 5)
//          std::cout << "CmdDriveWithController>> headingError is: " << headingError  << std::endl;
//      }
//      if(headingError  < 0.0) {
//        // Normailize for quadrant II
//        rotation = (-1.0 + (180.0+(headingError))/180.0);
//        if(headingError < -5)
//          std::cout << "CmdDriveWithController>> headingError is: " << headingError  << std::endl;
//      }
//
//      // Correct for quadrents III and IV
//      if(rotation > 1.0) {
//        rotation = (rotation - 1.0) * -1.0;
//        if(rotation > 1.0) {
//          std::cout << "CmdDriveWithController>> ****** Wooooooo"  << std::endl;
//        }
//      }
//      else if(rotation < -1.0) {
//        rotation = (rotation + 1.0) * -1.0;
//        if(rotation < -1.0) {
//          std::cout << "CmdDriveWithController>> ****** Wooooooo"  << std::endl;
//        }
//      }
//
//      // catch and avoid erratic jumps
//      if(headingError > 300)
//        rotation = -0.2;
//      else if(headingError < -300)
//        rotation = 0.2;
//
//      // Add offset if needed
//      if(rotation > 0.0 && rotation < 0.2) {
//        rotation = rotation + 0.07;
//      }
//      else if(rotation < 0.0 && rotation > -0.2) {
//        rotation = rotation - 0.07;
//      }
//      std::cout << "CmdDriveWithController>> Rotation set to: " << rotation << " Yaw is: " << m_driveTrain->GetYaw() << " Goal-Yaw is: " << m_driveTrain->GetYawStraightValue() << std::endl;
//    }
    }

    // Pass the speed and rotation to the drivetrain
    subDriveTrain.drive(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
