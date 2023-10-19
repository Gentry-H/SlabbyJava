// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Autos;
import frc.robot.commands.CmdDriveWithController;
import frc.robot.commands.CmdShiftGear;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SubDriveTrain;

import static frc.robot.Constants.OperatorConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SubDriveTrain subDriveTrain = new SubDriveTrain();

  private final Joystick driverController = new Joystick(DRIVER_CONTROLLER);

  private final JoystickButton driverController_button_a = new JoystickButton(driverController, BUTTON_A); // Acquire target to shoot
  private final JoystickButton driverController_button_b = new JoystickButton(driverController, BUTTON_B); // Virtul low gear
  private final JoystickButton driverController_button_x = new JoystickButton(driverController, BUTTON_X); // Index to shooter
  private final JoystickButton driverController_button_y = new JoystickButton(driverController, BUTTON_Y); // Intake ball
  private final JoystickButton driverController_button_lbump = new JoystickButton(driverController, BUTTON_L_BUMP); // Shift gears
  private final JoystickButton driverController_button_rbump = new JoystickButton(driverController, BUTTON_R_BUMP); // Aim drive train towards ball for intake
  private final JoystickButton driverController_button_select = new JoystickButton(driverController, BUTTON_SELECT); // Not Used
  private final JoystickButton driverController_button_start = new JoystickButton(driverController, BUTTON_START); // Not Used
  private final JoystickButton driverController_button_l3 = new JoystickButton(driverController, BUTTON_L3); // Not Used
  private final JoystickButton driverController_button_r3 = new JoystickButton(driverController, BUTTON_R3); // Not Used

  private final Joystick auxController = new Joystick(AUX_CONTROLLER);

  private final JoystickButton auxController_button_a = new JoystickButton(auxController, BUTTON_A); // Acquire target to shoot
  private final JoystickButton auxController_button_b = new JoystickButton(auxController, BUTTON_B); // Virtul low gear
  private final JoystickButton auxController_button_x = new JoystickButton(auxController, BUTTON_X); // Index to shooter
  private final JoystickButton auxController_button_y = new JoystickButton(auxController, BUTTON_Y); // Intake ball
  private final JoystickButton auxController_button_lbump = new JoystickButton(auxController, BUTTON_L_BUMP); // Shift gears
  private final JoystickButton auxController_button_rbump = new JoystickButton(auxController, BUTTON_R_BUMP); // Aim drive train towards ball for intake
  private final JoystickButton auxController_button_select = new JoystickButton(auxController, BUTTON_SELECT); // Not Used
  private final JoystickButton auxController_button_start = new JoystickButton(auxController, BUTTON_START); // Not Used
  private final JoystickButton auxController_button_l3 = new JoystickButton(auxController, BUTTON_L3); // Not Used
  private final JoystickButton auxController_button_r3 = new JoystickButton(auxController, BUTTON_R3); // Not Used

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public void configureDrive() {
    subDriveTrain.configure();
    subDriveTrain.getPidFromDashboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Setup the command to shift gears when right bumper is pressed
    driverController_button_lbump.whenPressed(new CmdShiftGear(subDriveTrain));

    // Set the default command for the Drive Train
    subDriveTrain.setDefaultCommand(new CmdDriveWithController(subDriveTrain, driverController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
