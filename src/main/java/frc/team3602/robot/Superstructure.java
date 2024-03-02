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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems.PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;
import frc.team3602.robot.subsystems.ClimberSubsystem;
import static frc.team3602.robot.Constants.PivotConstants.*;

import java.util.function.BooleanSupplier;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final PivotSubsystem pivotSubsys;
  private final ShooterSubsystem shooterSubsys;
  // private final ClimberSubsystem climberSubsys;
  private final Vision vision;

  private BooleanSupplier atVelocitySup = new BooleanSupplier() {
    @Override
    public boolean getAsBoolean() {
      return shooterSubsys.isAtVelocity;
    }
  };

  public Superstructure(IntakeSubsystem intakeSubsys, PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    // this.climberSubsys = climberSubsys;
    this.vision = vision;
  }

  public Command atVelocity(BooleanSupplier isFinishedSup) {
    return new FunctionalCommand(
        () -> {

        },
        () -> {
          var topTarget = shooterSubsys.topVelocityRPM;
          var bottomTarget = shooterSubsys.bottomVelocityRPM;

          var tolerance = 600;

          boolean topOk = MathUtil.isNear(topTarget, shooterSubsys.getTopEncoder(), tolerance);
          boolean bottomOk = MathUtil.isNear(bottomTarget, shooterSubsys.getBottomEncoder(), tolerance);

          shooterSubsys.isAtVelocity = topOk && bottomOk;
        }, (onEnd) -> {

        }, isFinishedSup, pivotSubsys);
  }

  public Command waitForVelocity() {
    return Commands.waitSeconds(2) /* .andThen(Commands.waitUntil((() -> atVelocitySup.getAsBoolean()))) */ ;
  }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> 45);
  }

  public Command testPickup() {
    return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.setRPM(5500, 5500),

      Commands.print("Setting Angle"),
      pivotSubsys.setAngle(() -> 5.0),

      Commands.print("Intaking Note"),
      intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),

      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75),

      Commands.print("Stopping Shooter"),
      shooterSubsys.stopShooter()
    );
  }

  // public Command testPickup() {
  //   return Commands.sequence(
  //     Commands.print("Setting Angle"),
  //     pivotSubsys.setAngle(() -> 5.0),
  //     Commands.print("Intaking Note"),
  //     intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),
  //     Commands.parallel(
  //       Commands.print("Spinning Up Shooter"),
  //       shooterSubsys.setRPM(5500, 5500),
  //       Commands.sequence(
  //         Commands.print("Waiting for Spinup"),
  //         Commands.waitSeconds(2),
  //         Commands.print("Shooting Note"),
  //         intakeSubsys.runIntake(() -> 0.75)
  //       )
  //     ),
  //     shooterSubsys.stopShooter()
  //   );
  // }

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
