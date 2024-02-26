/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3602.robot.Constants.ShooterConstants.*;

import java.util.function.DoubleSupplier;

import monologue.Logged;
import monologue.Annotations.Log;

public class ShooterSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  private double topOut, bottomOut;

  @Log
  private double topVolts, bottomVolts;

  public final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
  public final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();

  // Controllers
  @Log
  private double topVelocityRPM = 5500, bottomVelocityRPM = 5500; // 2300 trap_top: 1800, trap_bottom: 3000

  // private double kP, kI, kD;

  private final SparkPIDController topController = topShooterMotor.getPIDController();
  private final SparkPIDController bottomController = bottomShooterMotor.getPIDController();

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Shooter Top RPM", topVelocityRPM);
    SmartDashboard.putNumber("Shooter Bottom RPM", bottomVelocityRPM);

    // SmartDashboard.putNumber("Shooter kP", kP);
    // SmartDashboard.putNumber("Shooter kI", kI);
    // SmartDashboard.putNumber("Shooter kD", kD);

    configShooterSubsys();
  }

  @Log
  public double getTopEncoder() {
    return topShooterEncoder.getVelocity();
  }

  @Log
  public double getBottomEncoder() {
    return bottomShooterEncoder.getVelocity();
  }

  @Log
  public boolean atVelocity() {
    if ((getTopEncoder() >= 5000 && getBottomEncoder() >= 5000)) { // 3700 trap_top: 1300, trap_bottom: 2400
      return true;
    } else {
      return false;
    }
  }

  public Command setTopRPM(DoubleSupplier velocityRPM) {
    return runOnce(() -> {
      topVelocityRPM = velocityRPM.getAsDouble();
    });
  }

  public Command setBottomRPM(DoubleSupplier velocityRPM) {
    return runOnce(() -> {
      bottomVelocityRPM = velocityRPM.getAsDouble();
    });
  }

  public Command setRPM(DoubleSupplier topVelocityRPM, DoubleSupplier bottomVelocityRPM) {
    return runOnce(() -> {
      this.topVelocityRPM = topVelocityRPM.getAsDouble();
      this.bottomVelocityRPM = bottomVelocityRPM.getAsDouble();
    });
  }

  public Command runShooterSpeed(double topSpeed, double bottomSpeed) {
    return run(() -> {
      topShooterMotor.set(topSpeed);
      bottomShooterMotor.set(bottomSpeed);
    });
  }

  public Command runShooter() {
    return run(() -> {
      topController.setReference(topVelocityRPM, ControlType.kVelocity);
      bottomController.setReference(bottomVelocityRPM, ControlType.kVelocity);
    });
  }

  public Command stopShooter() {
    return runOnce(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    topOut = topShooterMotor.getAppliedOutput();
    bottomOut = bottomShooterMotor.getAppliedOutput();

    topVolts = topShooterMotor.getBusVoltage();
    bottomVolts = bottomShooterMotor.getBusVoltage();

    // // Get new tuning numbers from shuffleboard
    // var _kP = SmartDashboard.getNumber("Shooter kP", kP);
    // var _kI = SmartDashboard.getNumber("Shooter kI", kI);
    // var _kD = SmartDashboard.getNumber("Shooter kD", kD);

    // // Check if tuning numbers changed and update controller values
    // if (_kP != kP) {
    // kP = _kP;
    // topController.setP(kP);
    // bottomController.setP(kP);
    // }
    // if (_kI != kI) {
    // kI = _kI;
    // topController.setI(kI);
    // bottomController.setI(kI);
    // }
    // if (_kD != kD) {
    // kD = _kD;
    // topController.setD(kD);
    // bottomController.setD(kD);
    // }

    // Get new velocity numbers from shuffleboard
    // var _topVelocityRPM = SmartDashboard.getNumber("Shooter Top RPM",
    // topVelocityRPM);
    // var _bottomVelocityRPM = SmartDashboard.getNumber("Shooter Bottom RPM",
    // bottomVelocityRPM);

    // Check if velocity numbers changed and update values
    // if ((_topVelocityRPM != topVelocityRPM)) {
    // topVelocityRPM = _topVelocityRPM;
    // }
    // if ((_bottomVelocityRPM != bottomVelocityRPM)) {
    // bottomVelocityRPM = _bottomVelocityRPM;
    // }
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

    // Encoder config
    topShooterEncoder.setMeasurementPeriod(10);
    topShooterEncoder.setAverageDepth(2);

    bottomShooterEncoder.setMeasurementPeriod(10);
    bottomShooterEncoder.setAverageDepth(2);

    // Controls config
    topController.setFeedbackDevice(topShooterEncoder);
    bottomController.setFeedbackDevice(bottomShooterEncoder);

    topController.setP(kP);
    topController.setI(kI);
    topController.setD(kD);

    bottomController.setP(kP);
    bottomController.setI(kI);
    bottomController.setD(kD);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();
  }
}
