// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */

  public static Command moveForward(Drivetrain drivetrain) {
    final var request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity);

    return Commands.sequence(
      Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(1)), drivetrain),
      Commands.waitSeconds(1),
      Commands.runOnce(() -> drivetrain.setControl(request.withVelocityX(0.0)), drivetrain)
    );
  }

}
