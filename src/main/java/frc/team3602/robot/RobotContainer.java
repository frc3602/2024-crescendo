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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems._PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.auton.AutonFactory;
import frc.team3602.robot.subsystems.ClimberSubsystem;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import monologue.Logged;
import monologue.Annotations.Log;

public class RobotContainer implements Logged {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

  // Subsystems
  private final DrivetrainSubsystem driveSubsys = kDrivetrainSubsys;
  public final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final _PivotSubsystem pivotSubsys = new _PivotSubsystem();
  private final ClimberSubsystem climberSubsys = new ClimberSubsystem();

  @Log
  public double targetDistance;

  public final Vision vision = new Vision();
  public final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, shooterSubsys, vision);

  // Operator interfaces
  private double _kMaxSpeed = kMaxSpeed, _kMaxAngularRate = kMaxAngularRate;

  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);

  // private final Trigger atVelocity = new Trigger(shooterSubsys::atVelocity);

  // Autonomous
  private final Telemetry logger = new Telemetry(_kMaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("testPickup", superstructure.testPickup());
    NamedCommands.registerCommand("oneNoteMiddle", superstructure.oneNoteMiddle());
    NamedCommands.registerCommand("twoNoteMiddle", superstructure.twoNoteMiddle());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // autoChooser.addOption("One Note Middle", superstructure.oneNoteMiddle());

    driveSubsys.registerTelemetry(logger::telemeterize);

    configDefaultCommands();
    configButtonBindings();
  }

  private void configDefaultCommands() {
    driveSubsys
        .setDefaultCommand(driveSubsys.applyRequest(
            () -> driveSubsys.fieldCentricDrive
                .withVelocityX(xboxController.getLeftY() * _kMaxSpeed)
                .withVelocityY(xboxController.getLeftX() * _kMaxSpeed)
                .withRotationalRate(-xboxController.getRightX() *
                    _kMaxAngularRate)));

    pivotSubsys.setDefaultCommand(pivotSubsys.holdAngle());

    // shooterSubsys.setDefaultCommand(shooterSubsys.runShooterSpeed());

    climberSubsys.setDefaultCommand(climberSubsys.holdHeights());
  }

  private void configButtonBindings() {
    xboxController.leftTrigger(0.5).toggleOnTrue(new InstantCommand(() -> {
      _kMaxSpeed = kMaxSpeed * 0.5;
    })).toggleOnFalse(new InstantCommand(() -> {
      _kMaxSpeed = kMaxSpeed;
    }));

    xboxController.a().whileTrue(superstructure.pickupCmd()).onFalse(intakeSubsys.stopIntake());

    xboxController.leftBumper().onTrue(shooterSubsys.runShooterSpeed(0.75, 0.75))
        .onFalse(shooterSubsys.stopMotorsCmd());

    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.75))
        .onFalse(intakeSubsys.stopIntake());

    xboxController.x().onTrue(superstructure.inFrameCmd());

    xboxController.y().whileTrue(intakeSubsys.runIntake(() -> -0.25)).onFalse(intakeSubsys.stopIntake());

    xboxController.pov(180).onTrue(climberSubsys.setHeight(() -> 28.0));

    xboxController.pov(0).onTrue(climberSubsys.setHeight(() -> 47.75));

    xboxController.pov(90).whileTrue(pivotSubsys.setAngle(() -> pivotSubsys.lerpTable.get(vision.getTargetDistance())));

    xboxController.rightBumper().onTrue(superstructure.oneNoteMiddle());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
