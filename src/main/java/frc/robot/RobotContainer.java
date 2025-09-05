// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.BreakerLib.driverstation.BreakerInputStream;
import frc.robot.BreakerLib.driverstation.BreakerInputStream2d;
import frc.robot.BreakerLib.driverstation.gamepad.controllers.BreakerXboxController;
import frc.robot.BreakerLib.util.math.functions.BreakerLinearizedConstrainedExponential;


import frc.robot.subsystems.*;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems , commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final BreakerXboxController controller = new BreakerXboxController(Constants.OperatorConstants.kDriverControllerPort);
  private final Roller roller = new Roller();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
      
  private BreakerInputStream driverX, driverY, driverOmega;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the trigger bindings
    configureBindings();
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

      // LEFT BUMPER --> RESET LOCALIZER'S POSE
      controller.getLeftBumper().onTrue(Commands.runOnce(() -> drivetrain.getLocalizer().resetPose(new Pose2d(0,0, Rotation2d.fromRotations(0.5)))));

      // ---------------- SWERVE DRIVE ----------------

      // LEFT THUMBSTICK --> DRIVE
      BreakerInputStream2d driverTranslation = controller.getLeftThumbstick();
      driverTranslation = driverTranslation
              .clamp(1.0)
              .deadband(Constants.OperatorConstants.TRANSLATIONAL_DEADBAND, 1.0)
              .mapToMagnitude(new BreakerLinearizedConstrainedExponential(0.075, 3.0, true))
              .scale(Constants.DriveConstants.MAXIMUM_TRANSLATIONAL_VELOCITY.in(Units.MetersPerSecond));
      driverX = driverTranslation.getY();
      driverY = driverTranslation.getX();

      // RIGHT THUMBSTICK --> ROTATE
      driverOmega = controller.getRightThumbstick().getX()
              .clamp(1.0)
              .deadband(Constants.OperatorConstants.ROTATIONAL_DEADBAND, 1.0)
              .map(new BreakerLinearizedConstrainedExponential(0.364, 6.6, true))
              .scale(Constants.DriveConstants.MAXIMUM_ROTATIONAL_VELOCITY.in(Units.RadiansPerSecond));
  
      drivetrain.setDefaultCommand(drivetrain.getTeleopControlCommand(driverX, driverY, driverOmega, Constants.DriveConstants.TELEOP_CONTROL_CONFIG));

      // Binds buttons that set certain positions for the arm
      
      controller.getButtonY().onTrue(Commands.runOnce(() ->
        arm.setArmPosition(0.0)));
      controller.getButtonX().onTrue(Commands.runOnce(() ->
        arm.setArmPosition(-3.0)));
      controller.getButtonB().onTrue(Commands.runOnce(() ->
        arm.setArmPosition(3.0)));

      //Binds 2 buttons to turn the arm clockwise and counterclockwise when pressed
      
      controller.getDPad().getUp().whileTrue(Commands.run(() ->
        arm.setVoltageOutput(0.1)));
      controller.getDPad().getUp().onFalse(Commands.runOnce(() ->
        arm.setVoltageOutput(0.0)));
      controller.getDPad().getDown().whileTrue(Commands.run(() ->
        arm.setVoltageOutput(-0.1)));
      controller.getDPad().getDown().onFalse(Commands.runOnce(() ->
        arm.setVoltageOutput(0.0)));  
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}