/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  private final Trigger atVelocity;

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    // this.climberSubsys = climberSubsys;
    this.vision = vision;

    atVelocity = new Trigger(shooterSubsys::atVelocity);
  }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> 45);
  }

  public Command pickupCmd() {
    return Commands.sequence(
        pivotSubsys.setAngle(() -> 1.75),
        intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()));
  }

  // public Command shootCmd() {
  //   return shooterSubsys.runShooter()
  //       .alongWith(intakeSubsys.runIntake(() -> 0.75).unless(atVelocity.negate()));
  // }

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
