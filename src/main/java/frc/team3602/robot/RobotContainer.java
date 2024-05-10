/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
import monologue.Annotations.Log;

public class RobotContainer implements Logged {
  // private final PowerDistribution powerDistribution = new PowerDistribution(1,
  // ModuleType.kRev);

  public final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  public final CommandXboxController guitarController = new CommandXboxController(kGuitarController);
  // Subsystems
  private final DrivetrainSubsystem driveSubsys = new DrivetrainSubsystem(
      kDrivetrainConstants,
      xboxController,
      kFrontLeftModuleConstants,
      kFrontRightModuleConstants,
      kBackLeftModuleConstants,
      kBackRightModuleConstants);
  public final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem();
  private final ClimberSubsystem climberSubsys = new ClimberSubsystem();

  @Log
  public double targetDistance;

  public final Vision vision = new Vision();
  public final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, driveSubsys, shooterSubsys,
      vision);

  // Operator interfaces
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  private double _kMaxSpeed = kMaxSpeed, _kMaxAngularRate = kMaxAngularRate;

  // Autonomous
  // private final Telemetry logger = new Telemetry(_kMaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
 
   autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Drive Polarity", polarityChooser);

    polarityChooser.setDefaultOption("Default", 1.0);
    polarityChooser.addOption("Positive", 1.0);
    polarityChooser.addOption("Negative", -1.0);

    // driveSubsys.registerTelemetry(logger::telemeterize);

    configDefaultCommands();
    configButtonBindings();
  }

  private void configDefaultCommands() {
    driveSubsys
        .setDefaultCommand(driveSubsys.applyRequest(
            () -> driveSubsys.fieldCentricDrive
                .withVelocityX(polarityChooser.getSelected() * xboxController.getLeftY() *
                    _kMaxSpeed)
                .withVelocityY(polarityChooser.getSelected() * xboxController.getLeftX() *
                    _kMaxSpeed)
                .withRotationalRate(-xboxController.getRightX() *
                    _kMaxAngularRate)));

    pivotSubsys.setDefaultCommand(pivotSubsys.holdAngle());
    climberSubsys.setDefaultCommand(climberSubsys.holdHeights());


    //  shooterSubsys.setDefaultCommand(shooterSubsys.newRunShooterVoltage(() -> shooterSubsys.topVoltages, () -> shooterSubsys.bottomVoltages));
    //   shooterSubsys.setDefaultCommand(shooterSubsys.newRunShooterVoltage(() -> 1, () -> 1));

  }

  private void configButtonBindings() {
    // xboxController.leftTrigger(0.5).toggleOnTrue(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed * 0.5;
    // })).toggleOnFalse(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed;
    // }));

    // Xbox controls

    xboxController.a().whileTrue(superstructure.getNote()).onFalse(intakeSubsys.stopIntake());

    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.6))
        .onFalse(superstructure.stopIntakeAndShooter());
    // .75>.6
    xboxController.x().onTrue(superstructure.testNewShooterCmd());
    //    xboxController.x().onTrue(superstructure.aimTrap());
    xboxController.y().whileTrue(intakeSubsys.runIntake(() -> -0.6)).onFalse(intakeSubsys.stopIntake());

    xboxController.rightTrigger().onTrue(superstructure.aimSpeakerCmd())
        .onFalse(shooterSubsys.stopMotorsCmd().alongWith(driveSubsys.stopRumble()));

    xboxController.y().whileTrue(intakeSubsys.runIntake(() -> -0.25)).onFalse(intakeSubsys.stopIntake());

    // xboxController.leftBumper().whileTrue(pivotSubsys.runPivotWithLerpTable());


    xboxController.rightBumper().onTrue(superstructure.ampScoreCommand());

    xboxController.leftTrigger().whileTrue(superstructure.leftTriggerCmd());

    // Guitar controls
    guitarController.pov(180).onTrue(climberSubsys.setHeight(() -> 26.5));
    // height 28>27 >redone>30>28>26.5

    guitarController.pov(0).onTrue(climberSubsys.setHeight(() -> 47));

    // Triggers
    // new
    // Trigger(intakeSubsys::getColorSensor).onTrue(ledSubsys.setGreen()).onFalse(ledSubsys.setAlliance());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
