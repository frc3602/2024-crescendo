/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.team3602.robot.subsystems.IntakeSubsystem;
import frc.team3602.robot.subsystems._PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

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
    this.vision = vision;
  }

  // public Command waitForVelocity() {
  //   return Commands.waitSeconds(0.2).andThen(Commands.waitUntil((() -> atVelocitySup.getAsBoolean())));
  // }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> 45);
  }

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
 // initial30>25>23
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      // Commands.print("Stopping Shooter"),
      // shooterSubsys.stopShooter(),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),
      intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
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
      pivotSubsys.runSetAngle(() -> 38.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),

      Commands.print("Waiting for Spinup"),
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      Commands.print("Stopping Shooter"),
      shooterSubsys.stopShooter()
    );
  }


   public Command oneNoteLeftFirst() {
return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
      Commands.waitSeconds(0.2),
    //until(() -> shooterSubsys.isAtVelocity)> timeout

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 31.0).until(() -> pivotSubsys.isAtPosition),
 // initial30>25>23>put in left first>45>38>36>32>31
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      // Commands.print("Stopping Shooter"),
      // shooterSubsys.stopShooter(),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),
      intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
     );
  }

  public Command twoNoteLeftStart() {
    return Commands.sequence(
      Commands.print("Intaking Note"),
      intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),
    //intake speed .25>.5>.7 made parrallel to copy of move>.25

      // Commands.print("Spinning Up Shooter"),
      // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtSpeed),
      // Commands.waitSeconds(0.2),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 40).until(() -> pivotSubsys.isAtPosition),
      //init 40>45>48>52>35>38>40
      Commands.waitSeconds(0.2)
    );
  }

  public Command twoNoteLeftEnd() {
    return Commands.sequence(
      Commands.print("Waiting for Spinup"), 
      shooterSubsys.runShooterSpeed(0.75, 0.75).withTimeout(0.2),
      Commands.waitSeconds(0.2),
      Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
    //intake speed .7>.75 copied and pasted >0.65
    );
  }

   public Command oneNoteRight() {
     return Commands.sequence(
      Commands.print("Spinning Up Shooter"),
      shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
      Commands.waitSeconds(0.2),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 23.0).until(() -> pivotSubsys.isAtPosition),
 // initial30>25>23
      Commands.waitSeconds(0.2),
      
      Commands.print("Shooting Note"),
      intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

      // Commands.print("Stopping Shooter"),
      // shooterSubsys.stopShooter(),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
      Commands.waitSeconds(0.2),
      intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
     );
  }

 public Command twoNoteRightStart() {
    return Commands.sequence(
      Commands.print("Intaking Note"),
      intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()),
    //intake speed .25>.5>.7 made parrallel to copy of move>.25

      // Commands.print("Spinning Up Shooter"),
      // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtSpeed),
      // Commands.waitSeconds(0.2),

      Commands.print("Setting Angle"),
      pivotSubsys.runSetAngle(() -> 40).until(() -> pivotSubsys.isAtPosition),
      //init 40>45>48>52>35>38>40
      Commands.waitSeconds(0.2)
    );
  }

    public Command twoNoteRightEnd() {
    return Commands.sequence(
      Commands.print("Waiting for Spinup"), 
      shooterSubsys.runShooterSpeed(0.75, 0.75).withTimeout(0.2),
      Commands.waitSeconds(0.2),
      Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
    //intake speed .7>.75 copied and pasted >0.65
    );
  }

  public Command pickupCmd() {
    return Commands.sequence(
        pivotSubsys.setAngle(() -> 1.75),
        intakeSubsys.runIntake(() -> 0.25).until(() -> intakeSubsys.getColorSensor()));
      //  pivotSubsys.setAngle(() -> 25));
  }

  public Command trapCmd() {
    return Commands.sequence();
  }
}
