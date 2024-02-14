/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3602.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import monologue.Logged;
import monologue.Annotations.Log;

public class ShooterSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  private final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

  private final PowerDistribution powerDistribution;

  @Log
  private double topCurr;

  @Log
  private double topOut;

  @Log
  private double bottomCurr;

  @Log
  private double bottomOut;

  public ShooterSubsystem(PowerDistribution powerDistribution) {
    this.powerDistribution = powerDistribution;

    configShooterSubsys();
  }


  public Command runShooter(DoubleSupplier percentage) {
    return run(() -> {
      topShooterMotor.set(percentage.getAsDouble());
      bottomShooterMotor.set(percentage.getAsDouble());
    });
  }

  public Command stopMotors() {
    return runOnce(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    topCurr = powerDistribution.getCurrent(6);
    topOut = topShooterMotor.getAppliedOutput();

    bottomCurr = powerDistribution.getCurrent(5);
    bottomOut = bottomShooterMotor.getAppliedOutput();
  }

  private void configShooterSubsys() {
    // Top shooter motor config
    topShooterMotor.restoreFactoryDefaults(true);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.setSmartCurrentLimit(kTopShooterMotorCurrentLimit);
    topShooterMotor.enableVoltageCompensation(topShooterMotor.getBusVoltage());
    topShooterMotor.setOpenLoopRampRate(0.01);

    // Bottom shooter motor config
    bottomShooterMotor.restoreFactoryDefaults(true);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setSmartCurrentLimit(kBottomShooterMotorCurrentLimit);
    bottomShooterMotor.enableVoltageCompensation(bottomShooterMotor.getBusVoltage());
    bottomShooterMotor.setOpenLoopRampRate(0.01);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();
  }
}
