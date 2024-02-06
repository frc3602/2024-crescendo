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

import frc.team3602.robot.Constants.DrivetrainConstants;
// import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import monologue.Logged;

public class RobotContainer implements Logged {
  // Subsystems
  private final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsys = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsys = new PivotSubsystem();

  private final Vision vision = new Vision();
  private final Superstructure superstructure = new Superstructure(intakeSubsys, pivotSubsys, shooterSubsys, vision);
  public final PowerDistribution powerDistribution = new PowerDistribution(1, ModuleType.kRev);

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
    pivotSubsys.setDefaultCommand(pivotSubsys.holdAngle());
  }

  private void configButtonBindings() {
    // When a is pressed go to pickup note
    xboxController.a().whileTrue(superstructure.pickupCmd());

    // When y is pressed go to inside frame
    xboxController.y().whileTrue(superstructure.inFrameCmd());
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
