/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;
import static frc.team3602.robot.Constants.DrivetrainConstants.*;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrainSubsys = DRIVETRAIN_SUBSYS;
  private final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsys = new IntakeSubsystem();

  // Operator interfaces
  private final CommandXboxController xboxController = new CommandXboxController(XBOX_CONTROLLER_PORT);
  private double driveSpeed = 1.0;

  // Autonomous
  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    drivetrainSubsys
        .setDefaultCommand(drivetrainSubsys.applyRequest(
            () -> drivetrainSubsys.fieldCentricDrive
                .withVelocityX((-xboxController.getLeftY() * MAX_SPEED) * driveSpeed)
                .withVelocityY((-xboxController.getLeftX() * MAX_SPEED) * driveSpeed)
                .withRotationalRate((-xboxController.getRightX() * MAX_ANGULAR_RATE) * driveSpeed)));
  }

  private void configButtonBindings() {
    // While holding right bumper, slow drivetrain down half speed
    xboxController.rightBumper()
        .whileTrue(new InstantCommand(() -> driveSpeed = 0.5))
        .whileFalse(new InstantCommand(() -> driveSpeed = 1.0));

    // While holding a button, align with an apriltag
    xboxController.a().whileTrue(drivetrainSubsys.alignWithTarget());

    // While holding b button, run the intake at 500 RPM
    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 500));

    // While holding x button, reverse the intake at 500 RPM
    xboxController.x().whileTrue(intakeSubsys.runIntake(() -> -500));
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);

    sendableChooser.addOption("Autonomous", drivetrainSubsys.swerveControllerCommand());
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
