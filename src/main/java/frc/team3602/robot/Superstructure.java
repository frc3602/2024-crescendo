/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems._PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.commands.TestPickup;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import static frc.team3602.robot.Constants.PivotConstants.*;

import java.util.function.BooleanSupplier;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final _PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;
  // private final ClimberSubsystem climberSubsys;
  private final Vision vision;

  private BooleanSupplier atVelocitySup = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return shooterSubsys.isAtVelocity;
    }
  };

  public Superstructure(IntakeSubsystem intakeSubsys, _PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    // this.climberSubsys = climberSubsys;
    this.vision = vision;
  }

  // public Command waitForVelocity() {
  //   return Commands.waitSeconds(0.2).andThen(Commands.waitUntil((() -> atVelocitySup.getAsBoolean())));
  // }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> 45);
  }

  // public Command testPickup() {
  //   return new TestPickup();

  //   return new SequentialCommandGroup(
  //     new PrintCommand("Test #1"),
  //     new WaitCommand(2.0),
  //     new PrintCommand("Test #2")
  //   );
  // }

  // public Command testPickup() {
  //   return Commands.sequence(
  //     Commands.print("Spinning Up Shooter"),
  //     shooterSubsys.setRPM(5500, 5500),

  //     Commands.print("Setting Angle"),
  //     pivotSubsys.setAngle(() -> 5.0),

  //     Commands.print("Intaking Note"),
  //     intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),

  //     Commands.print("Shooting Note"),
  //     intakeSubsys.runIntake(() -> 0.75),

  //     Commands.print("Stopping Shooter"),
  //     shooterSubsys.stopShooter()
  //   );
  // }

  public Command testPickup() {
    return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.setRPM(5000, 5000),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 8.0),

      Commands.print("Intaking Note"),
      intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),

      Commands.print("Waiting for Spinup"),
      Commands.waitSeconds(0.20),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75),

      shooterSubsys.stopShooter()
    );
  }

  public Command oneNoteMiddle() {
    return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
      Commands.waitSeconds(0.2),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 23.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),

      Commands.print("Waiting for Spinup"),
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      // Commands.print("Stopping Shooter"),
      // shooterSubsys.stopShooter(),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2)
    );
  }

  public Command twoNoteMiddle() {
    return Commands.sequence(
      Commands.print("Intaking Note"),
      intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),

      // Commands.print("Spinning Up Shooter"),
      // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtSpeed),
      // Commands.waitSeconds(0.2),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 45.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),

      Commands.print("Waiting for Spinup"),
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      Commands.print("Stopping Shooter"),
      shooterSubsys.stopShooter()
    );
  }

  // public Command testPickup() {
  //   return Commands.sequence(
  //       intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),
  //       shooterSubsys.setRPM(5500, 5500),
  //       new WaitCommand(1.0).alongWith(intakeSubsys.runOnce(() -> System.out.println("Testing"))),
  //       (intakeSubsys.runIntake(() -> 0.75) /* .unless(() -> !shooterSubsys.atVelocity) */ ),
  //       shooterSubsys.stopShooter());
  // }

  // public Command firstShootMiddle() {
  // return Commands.sequence(
  // pivotSubsys.setAngle(() -> 0.50),
  // shooterSubsys.setRPM(() -> 5500, () -> 5500),
  // intakeSubsys.runIntake(() -> -0.75).unless(atVelocity));
  // }

  public Command pickupCmd() {
    return Commands.sequence(
        pivotSubsys.setAngle(() -> 1.75),
        intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()));
  }

  // public Command shootCmd() {
  // return shooterSubsys.runShooter()
  // .alongWith(intakeSubsys.runIntake(() -> 0.75).unless(atVelocity.negate()));
  // }

  // public Command speakerCmd() {
  //   return Commands.sequence(
  //     pivotSubsys.setAngle(() -> pivotSubsys.lerpTable.get(vision.getTargetDistance()))
  //   );
  // }

  public Command ampCmd() {
    return Commands.sequence();
  }

  public Command trapCmd() {
    return Commands.sequence();
  }
}
