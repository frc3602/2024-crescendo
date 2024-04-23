/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.DrivetrainSubsystem;
import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;
  private final DrivetrainSubsystem driveSubsys;
  // private final ClimberSubsystem climberSubsys;
  private final Vision vision;

  private BooleanSupplier atVelocitySup = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return shooterSubsys.isAtVelocity;
    }
  };

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, DrivetrainSubsystem driveSubsys,
      ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.driveSubsys = driveSubsys;
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    this.vision = vision;
  }

  public Command inFrameCmd() {
    return Commands.sequence(
      pivotSubsys.setAngle(45.0),
      pivotSubsys.runPivot().until(pivotSubsys.isAtPosition)
    );
  }

  // USED IN TELEOP

  public Command ampScoreCommand() {
    return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.runShooterSpeed(0.25, 0.25).withTimeout(0.2),//.2>.17

      Commands.print("Setting Angle"),
      pivotSubsys.setAngle(100.0, () -> pivotSubsys.getEffort()),  // 90.0 degrees worked for states
      pivotSubsys.runPivot().until(() -> pivotSubsys.encoderValue.getAsDouble() >= 90.0),

      Commands.print("Slowing Pivot"),
      pivotSubsys.setAngle(() -> 1.5),
      pivotSubsys.runPivot().until(() -> pivotSubsys.encoderValue.getAsDouble() >= 100.0)
    );
  }

  public Command leftTriggerCmd() {
    return Commands.parallel(
      Commands.sequence(
        pivotSubsys.setAngle(25.0),
        pivotSubsys.runPivot().until(pivotSubsys.isAtPosition)
      ),
      shooterSubsys.runShooterSpeed(0.8, 0.8)       
    );
  }


  public Command aimSpeakerCmd() {
    return Commands.parallel(
        shooterSubsys.runShooterSpeed(0.8, 0.8),
        pivotSubsys.runPivotWithLerpTable(),
        driveSubsys.teleopTurnTowardSpeaker(),
        Commands.print("aimSpeakerCmd"));
  }

  public Command aimTrap() {
    return Commands.sequence(
        driveSubsys.turnTowardTrap().until(() -> driveSubsys.pointToTrap()),
        driveSubsys.driveTowardTrap().until(() -> driveSubsys.closeToTrap()));
  }

  public Command getNote() {
    return Commands.sequence(
      Commands.sequence(
        pivotSubsys.setAngle(9.0),
        pivotSubsys.runPivot().until(pivotSubsys.isAtPosition)
      ),
      // angle 11>9
      Commands.parallel(
        intakeSubsys.runIntake(() -> 0.75).until(() -> intakeSubsys.getSensor1()),
        driveSubsys.driveTowardNote().until(() -> intakeSubsys.getSensor1())),
        Commands.parallel(
          Commands.sequence(
            pivotSubsys.setAngle(25.0),
            pivotSubsys.runPivot().until(pivotSubsys.isAtPosition)
          ),
          intakeSubsys.runIntake(() -> 0.15).until(() -> intakeSubsys.getSensor2())
        )
      );
  }
}
