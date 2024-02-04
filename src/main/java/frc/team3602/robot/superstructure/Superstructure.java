/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.superstructure;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
  }

  public Command speakerCmd() {
    return Commands.sequence(
        intakeSubsys.runIntake(() -> 0.75).until(intakeSubsys::getColorSensor),
        pivotSubsys.setTarget(() -> Degrees.of(90)),
        shooterSubsys.runShooter(() -> 0.75).alongWith(intakeSubsys.runIntake(() -> 0.75)));
  }
}
