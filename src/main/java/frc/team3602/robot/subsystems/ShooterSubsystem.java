/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.team3602.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

public class ShooterSubsystem implements Subsystem {
  // Motor controllers
  private final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder topShooterMotorEncoder = topShooterMotor.getEncoder();
  private final RelativeEncoder bottomShooterMotorEncoder = bottomShooterMotor.getEncoder();

  // PID controllers
  // private final SparkPIDController topShooterMotorController =
  // topShooterMotor.getPIDController();
  // private final SparkPIDController bottomShooterMotorController =
  // bottomShooterMotor.getPIDController();

  public ShooterSubsystem() {
    configShooterSubsys();
  }

  public Command runShooter(DoubleSupplier percentage) {
    return run(() -> topShooterMotor.set(percentage.getAsDouble()));
  }

  public Command stopShooter() {
    return runOnce(() -> topShooterMotor.stopMotor());
  }

  private void configShooterSubsys() {
    // Top shooter motor config
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.setSmartCurrentLimit(kTopShooterMotorCurrentLimit);
    topShooterMotor.enableVoltageCompensation(topShooterMotor.getBusVoltage());
    topShooterMotor.setOpenLoopRampRate(0.01);
    topShooterMotor.burnFlash();

    // Bottom shooter motor config
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.follow(topShooterMotor, false);
    bottomShooterMotor.setSmartCurrentLimit(kBottomShooterMotorCurrentLimit);
    bottomShooterMotor.enableVoltageCompensation(bottomShooterMotor.getBusVoltage());
    // bottomShooterMotor.setInverted(true);
    bottomShooterMotor.setOpenLoopRampRate(0.01);
    bottomShooterMotor.burnFlash();
  }
}
