/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;
  private final Vision vision;

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    this.vision = vision;
  }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> kInFramePos);
  }

  public Command pickupCmd() {
    return Commands.sequence(
        pivotSubsys.setAngle(() -> kPickupPos),
        intakeSubsys.runIntake(() -> 0.50).until(intakeSubsys::getColorSensor));
  }

  public Command speakerCmd() {
    return Commands.sequence(
        intakeSubsys.runIntake(() -> 0.75).until(intakeSubsys::getColorSensor),
        pivotSubsys.setAngle(() -> Degrees.of(90)),
        shooterSubsys.runShooter(() -> 0.75).alongWith(intakeSubsys.runIntake(() -> 0.75)));
  }

  public Command ampCmd() {
    return Commands.sequence();
  }

  public Command trapCmd() {
    return Commands.sequence();
  }
}
