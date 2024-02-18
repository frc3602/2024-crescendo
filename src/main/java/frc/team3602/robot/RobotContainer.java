/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.ClimberSubsystem;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import monologue.Logged;

public class RobotContainer implements Logged {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  // Subsystems
  private final DrivetrainSubsystem driveSubsys = kDrivetrainSubsys;
  public final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem();
  private final ClimberSubsystem climberSubsys = new ClimberSubsystem();

  private final Vision vision = new Vision();
  private final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, shooterSubsys, vision);

  // Operator interfaces
  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);

  // Autonomous
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    driveSubsys
        .setDefaultCommand(driveSubsys.applyRequest(
            () -> driveSubsys.fieldCentricDrive
                .withVelocityX(-xboxController.getLeftY() * kMaxSpeed.in(MetersPerSecond))
                .withVelocityY(-xboxController.getLeftX() * kMaxSpeed.in(MetersPerSecond))
                .withRotationalRate(-xboxController.getRightX() *
                    kMaxAngularRate.in(MetersPerSecond))));

    pivotSubsys.setDefaultCommand(pivotSubsys.holdAngle());

    climberSubsys.setDefaultCommand(climberSubsys.holdHeights());
  }

  private void configButtonBindings() {
    xboxController.a().whileTrue(superstructure.pickupCmd()).onFalse(intakeSubsys.stopIntake());

    xboxController.leftBumper().whileTrue(shooterSubsys.runShooter(() -> 0.75)).onFalse(shooterSubsys.stopShooter());

    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.75)).onFalse(intakeSubsys.stopIntake());

    xboxController.x().onTrue(pivotSubsys.setAngle(() -> 40));

    xboxController.y().onTrue(climberSubsys.setHeight(() -> 28.0));

    xboxController.rightBumper().onTrue(climberSubsys.setHeight(() -> 47.75));
  }

  private void configAutonomous() {
    NamedCommands.registerCommand("onePieceCommand", Commands.none());

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
