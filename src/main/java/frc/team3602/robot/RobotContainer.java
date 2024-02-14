/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.ClimberSubsystem;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import monologue.Logged;

public class RobotContainer implements Logged {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  // Subsystems
  private final DrivetrainSubsystem driveSubsys = kDrivetrainSubsys;
  private final ShooterSubsystem shooterSubsys = new ShooterSubsystem(powerDistribution);
  private final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem();
  // private final ClimberSubsystem climberSubsys = new ClimberSubsystem();

  private final Vision vision = new Vision();
  private final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, shooterSubsys, vision);

  // Operator interfaces
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);

  // Autonomous
  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    // driveSubsys
    // .setDefaultCommand(driveSubsys.applyRequest(
    // () -> driveSubsys.fieldCentricDrive
    // .withVelocityX(-xboxController.getLeftY() * kMaxSpeed.in(MetersPerSecond))
    // .withVelocityY(-xboxController.getLeftX() * kMaxSpeed.in(MetersPerSecond))
    // .withRotationalRate(-xboxController.getRightX() *
    // kMaxAngularRate.in(MetersPerSecond))));

    // pivotSubsys.setDefaultCommand(pivotSubsys.holdAngle());

    // climberSubsys.setDefaultCommand(climberSubsys.holdHeights());
  }

  private void configButtonBindings() {
    xboxController.a().whileTrue(superstructure.pickupCmd());

    // xboxController.b().whileTrue(shooterSubsys.runShooter(() -> 0.50));

    // xboxController.x().whileTrue(shooterSubsys.runShooter(() -> 0.0));

    // When a is pressed go to pickup note
    // xboxController.a().onTrue(pivotSubsys.setAngle(() -> 5));

    // When y is pressed go to inside frame
    // xboxController.b().onTrue(pivotSubsys.setAngle(() -> 10));
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
