/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class PivotSubsystem implements Subsystem, Logged {
  // Motor controllers
  private final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);

  // Encoders
  private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  // PID controllers
  private final SparkPIDController pivotMotorPIDController = pivotMotor.getPIDController();

  @Log.NT
  public double encoderPosition;

  @Log.NT
  public double angleDegrees;

  @Log.NT
  public double motorOutput;

  public PivotSubsystem() {
    configPivotSubsys();
  }

  public double getPivotPosition() {
    return pivotEncoder.getPosition();
  }

  public Command setAngle(DoubleSupplier angleDegrees) {
    return runOnce(() -> this.angleDegrees = angleDegrees.getAsDouble());
  }

  public Command holdAngle() {
    return run(() -> {
      pivotMotorPIDController.setReference(angleDegrees, ControlType.kPosition);
    });
  }

  @Override
  public void periodic() {
    encoderPosition = getPivotPosition();

    motorOutput = pivotMotor.getAppliedOutput();
  }

  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());

    pivotEncoder.setPositionConversionFactor(kPivotConversionFactor);

    pivotMotorPIDController.setFeedbackDevice(pivotEncoder);
    pivotMotorPIDController.setP(kPivotGains.kP);
    pivotMotorPIDController.setD(kPivotGains.kD);
    pivotMotorPIDController.setFF(kPivotGains.kF);

    pivotMotor.burnFlash();
  }
}
