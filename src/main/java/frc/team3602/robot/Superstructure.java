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
import frc.team3602.robot.subsystems._PivotSubsystem;
import frc.team3602.robot.subsystems.ShooterSubsystem;

import java.lang.annotation.Target;
import java.util.function.BooleanSupplier;

public class Superstructure {
  private final IntakeSubsystem intakeSubsys;
  private final _PivotSubsystem pivotSubsys;
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

  public Superstructure(IntakeSubsystem intakeSubsys, _PivotSubsystem pivotSubsys, DrivetrainSubsystem driveSubsys,
      ShooterSubsystem shooterSubsys,
      Vision vision) {
    this.driveSubsys = driveSubsys;
    this.intakeSubsys = intakeSubsys;
    this.pivotSubsys = pivotSubsys;
    this.shooterSubsys = shooterSubsys;
    this.vision = vision;
  }

  // public Command waitForVelocity() {
  // return Commands.waitSeconds(0.2).andThen(Commands.waitUntil((() ->
  // atVelocitySup.getAsBoolean())));
  // }

  public Command inFrameCmd() {
    return pivotSubsys.setAngle(() -> 45);
  }

  // DO NOT TOUCH used in twoNoteMiddle, and twoNoteRight Auton
  public Command oneNoteMiddle() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 23.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>25>23
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        Commands.print("Stopping Shooter"),
        shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 16.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)

    // Commands.print("Intaking Note"),
    // intakeSubsys.runIntake(() -> 0.75).until(() ->
    // intakeSubsys.getColorSensor()),
    // Commands.waitSeconds(0.2)
    );
  }

  // DO NOT TOUCH used in twoNoteMiddle Auton
  public Command twoNoteMiddle() {
    return Commands.sequence(
        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.7).until(() -> intakeSubsys.getSensor1()),
        Commands.waitSeconds(0.2),
        // intake speed .25>.75>.5>.4
        // Commands.print("Spinning Up Shooter"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtSpeed),
        // Commands.waitSeconds(0.2),

        // Commands.print("Setting Angle"),
        // pivotSubsys.runSetAngle(() -> 41).until(() -> pivotSubsys.isAtPosition),
        // // 40 adjusted path to frc field>37>39>36>37>39>41
        // Commands.waitSeconds(0.2),

        // aimSpeakerCmd(),

        Commands.print("Waiting for Spinup"),
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        Commands.print("Stopping Shooter"),
        shooterSubsys.stopShooter());

  }

  // DO NOT TOUCH used in twoNoteMiddleAmpSide Auton, and in
  // threeNoteMiddleAmpSide Auton
  public Command oneStartNoteMiddleAmpSide() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 21.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>25>23
        Commands.waitSeconds(0.2),
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),
        // Commands.print("Stopping Shooter"),
        // shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 9.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2)
    // intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
    // pivot angle initial 16 -> 11>9
    );
  }

  // DO NOT TOUCH used in twoNoteMiddleAmpSide Auton, threeNoteMiddleAmpSide,
  // threeNoteLeftAmpSide, and oneNoteMoveRight
  // Auton
  public Command twoNoteMiddleAmpSide() {
    return Commands.sequence(

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 9.0).until(() -> pivotSubsys.isAtPosition),

        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.50)
    // Commands.waitSeconds(0.2)
    // intake speed .25>.75 same as twoNoteMiddle>.4>.35>.3>.27>.25>.2
    );
  }

  // DO NOT TOUCH used in twoNoteMiddleAmpSide Auton, threeNoteMiddleAmpSide, and
  // threeNoteLeftAmpSide
  // Auton, and twoNoteLeftAmpSide Auton
  public Command twoNoteMiddleAmpSideEnd() {
    return Commands.sequence(
        // Commands.waitSeconds(0.2),
        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 39.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        // pivotSubsys.runSetAngle(() -> 23.0).until(() -> pivotSubsys.isAtPosition),
        // pivotSubsys.runSetAngle(() -> 20.0).until(() -> pivotSubsys.isAtPosition),
        // pivotSubsys.runSetAngle(() -> 6.0).until(() -> pivotSubsys.isAtPosition),
        // coppied from other working one
        // 36 c&p from twoNoteMiddle>34>36>38>39 Commands.print("Waiting for Spinup"),
        // Commands.print("Setting Angle"),
        // pivotSubsys.runSetAngle(() -> 39).until(() -> pivotSubsys.isAtPosition),
        // //36 c&p from twoNoteMiddle>34>36>38>39

        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),

        // Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2)
    // intake speed .7>.75 copied and pasted >0.65>.75
    );
  }

  // used in threeNoteMiddleAmpSide, and threeNoteLeftAmpSide Auton
  public Command threeFirstNoteMiddleAmpSide() {
    return Commands.sequence(
        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 29.0).until(() -> pivotSubsys.isAtPosition),
        pivotSubsys.runSetAngle(() -> 15.0).until(() -> pivotSubsys.isAtPosition),
        pivotSubsys.runSetAngle(() -> 6.0).until(() -> pivotSubsys.isAtPosition),

        Commands.waitSeconds(0.2),
        // angle 20>16>11>8>5>6>16>18>20

        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.27)
    // intake speed .4>.3>.27
    // Commands.waitSeconds(0.2)
    );
  }

  // used in threeNoteMiddleAmpSide, and threeNoteLeftAmp Auton
  public Command threeNoteMiddleAmpSide() {
    return Commands.sequence(
        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 43.0).until(() -> pivotSubsys.isAtPosition),
        // initial 45>43
        Commands.waitSeconds(0.2),

        // Commands.print("Spinning Up Shooter"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),
        // until(() -> shooterSubsys.isAtVelocity)> timeout

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2)

    // Commands.print("Stopping Shooter"),
    // shooterSubsys.stopShooter(),
    );
  }

  // Used in threeNoteLeftAmpSide Auton
  public Command oneNoteLeftAmpSideStart() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 19.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>25>23
        Commands.waitSeconds(0.2),
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),
        // Commands.print("Stopping Shooter"),
        // shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 7.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2)
    // intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
    // pivot angle initial 16 -> 11>9>11>9>7
    );
  }

  // Commands.print("Intaking Note"),
  // intakeSubsys.runIntake(() -> 0.75).until(() ->
  // intakeSubsys.getColorSensor()),
  // Commands.waitSeconds(0.2)
  // // Commands.print("Spinning Up Shooter"),
  // // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
  // shooterSubsys.isAtSpeed),
  // // Commands.waitSeconds(0.2),
  // Commands.print("Setting Angle"),
  // pivotSubsys.runSetAngle(() -> 39).until(() -> pivotSubsys.isAtPosition),
  // //40 adjusted path to frc field>37>39
  // Commands.waitSeconds(0.2),
  // Commands.print("Waiting for Spinup"),
  // Commands.waitSeconds(0.2),
  // Commands.print("Shooting Note"),
  // intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),
  // Commands.print("Stopping Shooter"),
  // shooterSubsys.stopShooter()
  // }

  // public Command testPickup() {
  // return Commands.sequence(
  // Commands.print("Spinning Up Shooter"),
  // shooterSubsys.setRPM(5000, 5000),
  // Commands.print("Setting Angle"),
  // pivotSubsys.runSetAngle(() -> 8.0),
  // Commands.print("Intaking Note"),
  // intakeSubsys.runIntake(() -> 0.25).until(() ->
  // intakeSubsys.getColorSensor()),
  // Commands.print("Waiting for Spinup"),
  // Commands.waitSeconds(0.20),
  // Commands.print("Shooting Note"),
  // intakeSubsys.runIntake(() -> 0.75),
  // shooterSubsys.stopShooter());
  // }

  public Command twoNoteMiddleEnd() {
    return Commands.sequence(
        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 23).until(() -> pivotSubsys.isAtPosition),
        // 40 adjusted path to frc field>37>39
        Commands.waitSeconds(0.2),

        Commands.print("Waiting for Spinup"),
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        Commands.print("Stopping Shooter"),
        shooterSubsys.stopShooter(),
        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 16.0).until(() -> pivotSubsys.isAtPosition));
  }

  // used in twoNoteLeft Auton
  public Command oneNoteLeftFirst() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        Commands.waitSeconds(0.2),
        // until(() -> shooterSubsys.isAtVelocity)> timeout

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 31.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>25>23>put in left first>45>38>36>32>31
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2)

    // Commands.print("Stopping Shooter"),
    // shooterSubsys.stopShooter(),
    );
  }

  // used in twoNoteLeft Auton
  public Command twoNoteLeftStart() {
    return Commands.sequence(

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2),

        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.75),
        // intake speed .25>.5>.7 made parrallel to copy of move>.25>.75

        // Commands.print("Spinning Up Shooter"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtSpeed),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 39).until(() -> pivotSubsys.isAtPosition),
        // init 40>45>48>52>35>38>40 adjusted path to frc field>37>39>42
        Commands.waitSeconds(0.2));
  }

  public Command oneNoteRight() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 32.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>32
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        // Commands.print("Stopping Shooter"),
        // shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2));
  }

  // used in twoNoteRight Auton
  public Command twoNoteRightStart() {
    return Commands.sequence(
        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.25),
        // intake speed .25>.5>.7 made parrallel to copy of move>.25

        // Commands.print("Spinning Up Shooter"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtSpeed),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 37).until(() -> pivotSubsys.isAtPosition),
        // init 40>45>48>52>35>38>40 adjusted path to frc field>37
        Commands.waitSeconds(0.2));
  }

  // Used in twoNoteRightAuton
  public Command twoNoteRightEnd() {
    return Commands.sequence(
        // Commands.print("Waiting for Spinup"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).withTimeout(0.2),
        // Commands.waitSeconds(0.2),
        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2)
    // intake speed .65>.75
    );
  }

  public Command firstFourNoteAutonRight() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 32.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>32
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        // Commands.print("Stopping Shooter"),
        // shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2));
  }

  public Command startFourNoteAutonRight() {
    return Commands.sequence(

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 8.0).until(() -> pivotSubsys.isAtPosition),
        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2),

        Commands.print("Intaking Note"),
        intakeSubsys.runIntake(() -> 0.75),
        // intake speed .25>.5>.7 made parrallel to copy of move>.25>.75

        // Commands.print("Spinning Up Shooter"),
        // shooterSubsys.runShooterSpeed(0.75, 0.75).until(() ->
        // shooterSubsys.isAtSpeed),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 39).until(() -> pivotSubsys.isAtPosition),
        // init 40>45>48>52>35>38>40 adjusted path to frc field>37>39>42
        Commands.waitSeconds(0.2));
  }

  // used in oneLeftMoveShort and oneNoteMoveMiddle
  public Command oneLeftMoveShort() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        // Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 21.0).until(() -> pivotSubsys.isAtPosition),
        // initial30>25>23
        Commands.waitSeconds(0.2),
        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2)

    // Commands.print("Stopping Shooter"),
    // shooterSubsys.stopShooter(),
    );
  }

  public Command twoNoteMoveAmpSideShoot() {
    return Commands.sequence(

        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 21.0).until(() -> pivotSubsys.isAtPosition),
        // angle is same as oneNoteLeftFirst> realized path shoots at bumper-same as two
        // note middle>25>23

        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2));
  }

  public Command oneNoteMoveRightFirst() {
    return Commands.sequence(
        Commands.print("Spinning Up Shooter"),
        shooterSubsys.runShooterSpeed(0.75, 0.75).until(() -> shooterSubsys.isAtVelocity),
        Commands.waitSeconds(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 21.0).until(() -> pivotSubsys.isAtPosition),
        // angle is same as oneNoteLeftFirst> realized path shoots at bumper-same as two
        // note middle>25>23

        Commands.waitSeconds(0.2),

        Commands.print("Shooting Note"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        // Commands.print("Stopping Shooter"),
        // shooterSubsys.stopShooter(),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 15.0).until(() -> pivotSubsys.isAtPosition));
    // Commands.waitSeconds(0.2),
    // intakeSubsys.runIntake(() -> 0.65).withTimeout(0.2)
    // );
  }
  // public Command oneNoteMoveRight() {
  // return Commands.sequence(
  // Commands.print("Spinning Up Shooter"),
  // shooterSubsys.runShooterSpeed(0.7, 0.7).until(() ->
  // shooterSubsys.isAtVelocity),
  // Commands.waitSeconds(0.2),

  // Commands.print("Setting Angle"),
  // pivotSubsys.runSetAngle(() -> 21.0).until(() -> pivotSubsys.isAtPosition));
  // }
  // // initial30>25>23 copy and pasted>31>28>23>21

  // public Command autonPickupCmd() {
  //   return Commands.sequence(
  //       pivotSubsys.setAngle(() -> 9), // 1.75>7>15>11>9
  //       intakeSubsys.runIntake(() -> 0.6).until(() -> intakeSubsys.getSensor1()),
  //       intakeSubsys.runIntake(() -> 0.3).until(() -> intakeSubsys.getSensor2()),
  //       pivotSubsys.setAngle(() -> 20))
  //       .alongWith(driveSubsys.turnTowardNote());
  // }

  // public Command pickupCmd() {
  // return Commands.sequence(
  // Commands.parallel(
  // pivotSubsys.runSetAngle(() -> 9), // 1.75>7>15>11>9
  // intakeSubsys.runIntake(() -> 0.6).until(() -> intakeSubsys.getSensor1())),
  // // intakeSubsys.runIntake(() -> 0.3).until(() -> intakeSubsys.getSensor2()),
  // pivotSubsys.setAngle(() -> 20))
  // .alongWith(driveSubsys.turnTowardNote());
  // }


  




   public Command autonCloseNoteShootCmd() {
    return Commands.sequence(
        Commands.print("firstAutonAimSpeakerCmd"),
        // intakeSubsys.stopIntake(),
       //pivotSubsys.runSetAngle(() -> 40).until(() -> pivotSubsys.isAtPosition),

        pivotSubsys.runSetAngle(() -> 39).until(() -> pivotSubsys.encoderValue >=35),
      
     //!pivotSubsys.isAtPosition ? pivotSubsys.runSetAngle(() -> 39).until (() -> pivotSubsys.isAtPosition): pivotSubsys.runOnce(() -> {}),
               pivotSubsys.stopMotors(),
  shooterSubsys.runShooterSpeed(0.65, 0.65).until(() -> shooterSubsys.topOut >= 0.55), //speed .8>.65  topout .65>.55

        //.until(() -> (pivotSubsys.encoderValue >= 45)),
        Commands.print("autonAimSpeakerCmd")
        
        );
  }







  //pivot angle 49>47>45

  // public boolean position23() {
  // var target = 23;
  // var tolerance = 2;

  // return ((MathUtil.isNear(target, pivotSubsys.getDegrees(), tolerance)));
  // }

  public Command autonCenterShootCmd() {
    return Commands.sequence(
        Commands.parallel(
            shooterSubsys.runShooterSpeed(0.8, 0.8).until(() -> shooterSubsys.isAtVelocity),
            pivotSubsys.runSetAngle(() -> 23).until(() -> pivotSubsys.encoderValue <= 24),
            intakeSubsys.stopIntake()),

        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 9.0).until(() -> pivotSubsys.isAtPosition),

        shooterSubsys.stopMotorsCmd());

  }

  public Command autonSideShootCmd() {
    return Commands.sequence(
        Commands.parallel(
            shooterSubsys.runShooterSpeed(0.55, 0.55).until(() -> shooterSubsys.isAtVelocity),//velocity .8>.7>.6>.55
            pivotSubsys.runSetAngle(() -> 21).until(() -> pivotSubsys.encoderValue <= 22),
            intakeSubsys.stopIntake()),

        Commands.waitSeconds(0.2),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),

        Commands.print("Setting Angle"),
        pivotSubsys.runSetAngle(() -> 9.0).until(() -> pivotSubsys.isAtPosition),
        // Commands.waitSeconds(0.2)

        shooterSubsys.stopMotorsCmd());
  }

   public Command keepPivot() {
    return Commands.sequence(
        pivotSubsys.runSetAngle(() -> 14).until(() -> pivotSubsys.isAtPosition)

    //    !pivotSubsys.isAtPosition ? pivotSubsys.runSetAngle(() -> 14).until (() -> pivotSubsys.isAtPosition): pivotSubsys.runOnce(() -> {})) ;
        
        //.until(() -> pivotSubsys.isAtPosition)
    // pivotSubsys.holdAngle()
    );
  }
  
  public Command autonAimSpeakerCmd() {
    return Commands.sequence(
        Commands.print("firstAutonAimSpeakerCmd"),
        // intakeSubsys.stopIntake(),
        shooterSubsys.runShooterSpeed(0.8, 0.8).until(() -> shooterSubsys.isAtVelocity),
       //pivotSubsys.runSetAngle(() -> 40).until(() -> pivotSubsys.isAtPosition),

        pivotSubsys.runSetAngle(() -> 45).until(() -> pivotSubsys.isAtPosition),
      
     !pivotSubsys.isAtPosition ? pivotSubsys.runSetAngle(() -> 49).until (() -> pivotSubsys.isAtPosition): pivotSubsys.runOnce(() -> {}),

        //.until(() -> (pivotSubsys.encoderValue >= 45)),
        Commands.print("autonAimSpeakerCmd"),
         pivotSubsys.stopMotors()
        
        );
  }

  public Command autonShootCmd() {
    return Commands.sequence(
        Commands.print("AutonShootCmd"),
        intakeSubsys.runIntake(() -> 0.75).withTimeout(0.2),
        Commands.waitSeconds(0.2),
        shooterSubsys.stopMotorsCmd(),
        Commands.print("secondAutonshootcmd")
        );
  }


      public Command autonGetNote() {
    return Commands.sequence(
        pivotSubsys.runSetAngle(() -> 9).until(() -> pivotSubsys.isAtPosition),
        // angle 11>9
        Commands.parallel(
            intakeSubsys.runIntake(() -> 0.75).until(() -> intakeSubsys.getSensor1()),
            driveSubsys.driveTowardNote().until(() -> intakeSubsys.getSensor1())),
        // Commands.parallel(
            // pivotSubsys.runSetAngle(() -> 25).until(() -> pivotSubsys.isAtPosition),
            driveSubsys.stopDriveCmd(),
            intakeSubsys.runIntake(() -> 0.15).until(() -> intakeSubsys.getSensor2()));

}
  // public Command trapCmd() {
  // return Commands.sequence();
  // }

  // USED IN TELEOP

  public Command ampScoreCommand() {
    return Commands.sequence(
        Commands.parallel(
            Commands.print("Spinning Up Shooter"),
            shooterSubsys.runShooterSpeed(0.2, 0.2).until(() -> shooterSubsys.isAtVelocity),

            Commands.print("Setting Angle"),

            // pivotSubsys.runSetAngle(() -> 30.0).until(() -> pivotSubsys.isAtPosition)),
            // pivotSubsys.runSetAngle(() -> 55.0).until(() -> pivotSubsys.isAtPosition),
            // pivotSubsys.runSetAngle(() -> 80.0).until(() -> pivotSubsys.isAtPosition),
            pivotSubsys.runSetAngle(() -> 100.0).until(() -> pivotSubsys.isAtPosition)));
  }

  public Command aimSpeakerCmd() {
    return Commands.parallel(
        shooterSubsys.runShooterSpeed(0.8, 0.8),
        pivotSubsys.runSetAngle(() -> pivotSubsys.lerpAngle),
        driveSubsys.turnTowardSpeaker(),
        Commands.print("aimSpeakerCmd"));
  }

  // TODO if time- add a tolerance to runsetangle and try to use in auton

  public Command aimTrap() {
    return Commands.sequence(
        driveSubsys.turnTowardTrap().until(() -> driveSubsys.pointToTrap()),
        driveSubsys.driveTowardTrap().until(() -> driveSubsys.closeToTrap()));
  }

    public Command getNote() {
    return Commands.sequence(
        pivotSubsys.runSetAngle(() -> 9).until(() -> pivotSubsys.isAtPosition),
        // angle 11>9
        Commands.parallel(
            intakeSubsys.runIntake(() -> 0.75).until(() -> intakeSubsys.getSensor1()),
            driveSubsys.driveTowardNote().until(() -> intakeSubsys.getSensor1())),
        Commands.parallel(
            pivotSubsys.runSetAngle(() -> 25).until(() -> pivotSubsys.isAtPosition),
            intakeSubsys.runIntake(() -> 0.15).until(() -> intakeSubsys.getSensor2())));

}
}
