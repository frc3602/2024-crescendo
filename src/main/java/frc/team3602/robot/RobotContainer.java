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
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
//import frc.team3602.robot.subsystems.LEDSubsystem;
import frc.team3602.robot.subsystems._PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.ClimberSubsystem;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import monologue.Logged;
import monologue.Annotations.Log;

public class RobotContainer implements Logged {
  private final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

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
  private final _PivotSubsystem pivotSubsys = new _PivotSubsystem();
  private final ClimberSubsystem climberSubsys = new ClimberSubsystem();
  // private final LEDSubsystem ledSubsys = new LEDSubsystem();

  @Log
  public double targetDistance;

  public final Vision vision = new Vision();
  public final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, driveSubsys, shooterSubsys, vision);

  // Operator interfaces
  private SendableChooser<Double> polarityChooser = new SendableChooser<>();

  private double _kMaxSpeed = kMaxSpeed, _kMaxAngularRate = kMaxAngularRate;


  // Autonomous
  private final Telemetry logger = new Telemetry(_kMaxSpeed);

  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("aimSpeakerCmd", superstructure.aimSpeakerCmd());
    NamedCommands.registerCommand("ampScoreCommand", superstructure.ampScoreCommand());
    NamedCommands.registerCommand("autonPickupCmd", superstructure.autonPickupCmd());
        NamedCommands.registerCommand("getNote", superstructure.getNote());



    NamedCommands.registerCommand("oneNoteMiddle", superstructure.oneNoteMiddle());
    NamedCommands.registerCommand("oneStartNoteMiddleAmpSide", superstructure.oneStartNoteMiddleAmpSide());
    NamedCommands.registerCommand("oneLeftMoveShort", superstructure.oneLeftMoveShort());
    NamedCommands.registerCommand("twoNoteMiddleAmpSide", superstructure.twoNoteMiddleAmpSide());
    NamedCommands.registerCommand("twoNoteMiddle", superstructure.twoNoteMiddle());
    NamedCommands.registerCommand("oneNoteLeftAmpSideStart", superstructure.oneNoteLeftAmpSideStart());
    NamedCommands.registerCommand("twoNoteMiddleEnd", superstructure.twoNoteMiddleEnd());
    NamedCommands.registerCommand("twoNoteLeftStart", superstructure.twoNoteLeftStart());
    NamedCommands.registerCommand("twoNoteMiddleAmpSideEnd", superstructure.twoNoteMiddleAmpSideEnd());
    NamedCommands.registerCommand("oneNoteLeftFirst", superstructure.oneNoteLeftFirst());
    NamedCommands.registerCommand("twoNoteMoveAmpSideShoot", superstructure.twoNoteMoveAmpSideShoot());
    NamedCommands.registerCommand("twoNoteRightStart", superstructure.twoNoteRightStart());
    NamedCommands.registerCommand("twoNoteRightEnd", superstructure.twoNoteRightStart());
    NamedCommands.registerCommand("oneNoteMoveRightFirst", superstructure.oneNoteMoveRightFirst());
    NamedCommands.registerCommand("threeNoteMiddleAmpSide", superstructure.threeNoteMiddleAmpSide());
    NamedCommands.registerCommand("threeFirstNoteMiddleAmpSide", superstructure.threeFirstNoteMiddleAmpSide());
    // NamedCommands.registerCommand("oneNoteRight", superstructure.oneNoteRight());

/*//LEGIBLE AUTONS--if functional, comment out ALL other auton commands
  
    NamedCommands.registerCommand("sideStart", superstructure.sideStart());
    NamedCommands.registerCommand("centerStart", superstructure.sideStart());
   
    NamedCommands.registerCommand("intakeNote", superstructure.intakeNote());
   
    NamedCommands.registerCommand("shootCloseAmp", superstructure.shootCloseAmp());
    NamedCommands.registerCommand("shootCloseCenter", superstructure.shootCloseCenter());
    NamedCommands.registerCommand("shootCloseSource", superstructure.shootCloseSource());
   
    NamedCommands.registerCommand("shootFarAmp", superstructure.shootFarAmp());
    NamedCommands.registerCommand("shootFarInnerAmp", superstructure.shootFarInnerAmp());
    NamedCommands.registerCommand("shootFarCenter", superstructure.shootFarCenter());
    NamedCommands.registerCommand("shootFarInnerSource", superstructure.shootFarInnerSource());
    NamedCommands.registerCommand("shootFarSource", superstructure.shootFarSource());
 */



    NamedCommands.registerCommand("oneNoteTwistFirst", superstructure.oneNoteLeftFirst());
    NamedCommands.registerCommand("twoNoteTwistStart", superstructure.oneNoteLeftFirst());
    NamedCommands.registerCommand("twoNoteTwistEnd", superstructure.oneNoteLeftFirst());

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
    // shooterSubsys.setDefaultCommand(shooterSubsys.runShooterSpeed());

    // climberSubsys.setDefaultCommand(climberSubsys.holdHeights());
  }

  private void configButtonBindings() {
    // xboxController.leftTrigger(0.5).toggleOnTrue(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed * 0.5;
    // })).toggleOnFalse(new InstantCommand(() -> {
    // _kMaxSpeed = kMaxSpeed;
    // }));

    // Xbox controls
    xboxController.a().whileTrue(superstructure.pickupCmd()).onFalse(intakeSubsys.stopIntake());

    xboxController.rightTrigger().onTrue(superstructure.aimSpeakerCmd())
        .onFalse(shooterSubsys.stopMotorsCmd());

    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.6))
        .onFalse(intakeSubsys.stopIntake());
    // .75>.6
    xboxController.x().onTrue(superstructure.inFrameCmd());

    xboxController.y().whileTrue(intakeSubsys.runIntake(() -> -0.25)).onFalse(intakeSubsys.stopIntake());

   xboxController.leftBumper().whileTrue(pivotSubsys.runSetAngle(() -> pivotSubsys.lerpAngle)); // 23

    xboxController.rightBumper().onTrue(superstructure.ampScoreCommand());

    xboxController.leftTrigger().whileTrue(driveSubsys.alignWithTarget());

    // Guitar controls
    guitarController.pov(180).onTrue(climberSubsys.setHeight(() -> 26.75));
    // height 28>27

    guitarController.pov(0).onTrue(climberSubsys.setHeight(() -> 47.75));

    // Triggers
    // new
    // Trigger(intakeSubsys::getColorSensor).onTrue(ledSubsys.setGreen()).onFalse(ledSubsys.setAlliance());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
