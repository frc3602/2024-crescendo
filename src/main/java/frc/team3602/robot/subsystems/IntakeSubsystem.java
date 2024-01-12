/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.team3602.robot.Constants.ShooterConstants.*;

public class IntakeSubsystem implements Subsystem {
  // Motor controllers

  private final CANSparkMax intakeMotor = new CANSparkMax(TOP_SHOOTER_MOTOR_ID, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();

  public IntakeSubsystem() {

  }
}
