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
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import static frc.team3602.robot.Constants.OperatorInterfaceConstants.*;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSubsystem drivetrainSubsys = kDrivetrainSubsys;
  private final ShooterSubsystem shooterSubsys = new ShooterSubsystem();
  public final IntakeSubsystem intakeSubsys = new IntakeSubsystem();

  // Operator interfaces
  private final CommandXboxController xboxController = new CommandXboxController(kXboxControllerPort);
  private double driveSpeed = 1.0;

  // Autonomous
  private final SendableChooser<Command> sendableChooser = new SendableChooser<>();

  private final ChoreoTrajectory trajectory = Choreo.getTrajectory("traj");

  public RobotContainer() {
    configDefaultCommands();
    configButtonBindings();
    configAutonomous();
  }

  private void configDefaultCommands() {
    drivetrainSubsys
        .setDefaultCommand(drivetrainSubsys.applyRequest(
            () -> drivetrainSubsys.fieldCentricDrive
                .withVelocityX((-xboxController.getLeftY() * kMaxSpeed.in(MetersPerSecond)) * driveSpeed)
                .withVelocityY((-xboxController.getLeftX() * kMaxSpeed.in(MetersPerSecond)) * driveSpeed)
                .withRotationalRate((-xboxController.getRightX() * kMaxAngularRate.in(MetersPerSecond)) * driveSpeed)));
  }

  private void configButtonBindings() {
    // While holding right bumper, slow drivetrain down half speed
    xboxController.rightBumper()
        .whileTrue(new InstantCommand(() -> driveSpeed = 0.5))
        .whileFalse(new InstantCommand(() -> driveSpeed = 1.0));

    // While holding a button, align with an apriltag
    xboxController.a().whileTrue(drivetrainSubsys.alignWithTarget());

    // While holding b button, run the intake at 500 RPM
    xboxController.b().whileTrue(intakeSubsys.runIntake(() -> 0.75))
        .whileFalse(intakeSubsys.run(() -> intakeSubsys.stopIntake()));

    // While holding x button, reverse the intake at 500 RPM
    xboxController.x().whileTrue(intakeSubsys.runIntake(() -> -0.75))
        .whileFalse(intakeSubsys.run(() -> intakeSubsys.stopIntake()));
  }

  private void configAutonomous() {
    SmartDashboard.putData(sendableChooser);

    sendableChooser.addOption("Autonomous", drivetrainSubsys.choreoSwerveCommand(trajectory));
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
