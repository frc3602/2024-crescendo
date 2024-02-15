/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.ClimberSubsystem;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;
  // private final ClimberSubsystem climberSubsys;
  private final Vision vision;
  private final XboxController xboxController;

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys,
      Vision vision, XboxController xboxController) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    // this.climberSubsys = climberSubsys;
    this.vision = vision;
    this.xboxController = xboxController;
  }

  // public Command inFrameCmd() {
  // return pivotSubsys.setAngle(() -> kInFramePos);
  // }

  public Command pickupCmd() {
    return Commands.sequence(
        // pivotSubsys.setAngle(() -> kPickupPos),
        intakeSubsys.runIntakeTwo(() -> 0.15).until(() -> xboxController.getBButton()).until(intakeSubsys::getColorSensor));
  }

  // public Command speakerCmd() {
  // return Commands.sequence(
  // pivotSubsys.setAngle(() ->
  // Degrees.of(pivotSubsys.lerpTable.get(vision.getTargetDistance()))));
  // }

  public Command ampCmd() {
    return Commands.sequence();
  }

  public Command trapCmd() {
    return Commands.sequence();
  }
}
