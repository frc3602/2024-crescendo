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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.ClimberConstants.*;

import java.util.function.DoubleSupplier;

public class ClimberSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  private double rightOut, leftOut;

  private final CANSparkMax rightMotor = new CANSparkMax(kRightClimberId, MotorType.kBrushless);
  private final CANSparkMax leftMotor = new CANSparkMax(kLeftClimberId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // Controls
  @Log
  private double rightTarget, leftTarget;

  private final SparkPIDController rightController = rightMotor.getPIDController();
  private final SparkPIDController leftController = leftMotor.getPIDController();

  private final ArmFeedforward rightFeedforward = new ArmFeedforward(kS, kG, kV, kA);
  private final ArmFeedforward leftFeedforward = new ArmFeedforward(kS, kG, kV, kA);

  public ClimberSubsystem() {
    SmartDashboard.putNumber("Proportional", kP);
    SmartDashboard.putNumber("Integral", kI);
    SmartDashboard.putNumber(" Derivitave", kD);

    rightTarget = 28.0;
    leftTarget = 28.0;

    resetEncoders();
    configClimberSubsys();
  }

  public void resetEncoders() {
    rightEncoder.setPosition(kRetractedHeight);
    leftEncoder.setPosition(kRetractedHeight);
  }

  @Log
  public double getRightEncoder() {
    return rightEncoder.getPosition();
  }

  @Log
  public double getLeftEncoder() {
    return leftEncoder.getPosition();
  }

  public Command setRightHeight(double setpoint) {
    return runOnce(() -> rightTarget = setpoint);
  }

  public Command setLeftHeight(double setpoint) {
    return runOnce(() -> leftTarget = setpoint);
  }

  public Command setHeight(DoubleSupplier heightInches) {
    return runOnce(() -> {
      rightTarget = heightInches.getAsDouble();
      leftTarget = heightInches.getAsDouble();
    });
  }

  @Log
  private double getRightEffort() {
    var ffEffort = rightFeedforward.calculate(Units.degreesToRadians(rightTarget), 0.0);

    return ffEffort;
  }

  @Log
  private double getLeftEffort() {
    var ffEffort = leftFeedforward.calculate(Units.degreesToRadians(leftTarget), 0.0);

    return ffEffort;
  }

  public Command holdHeights() {
    return run(() -> {
      // rightController.setReference(rightTarget, ControlType.kPosition, 0, getRightEffort());
      // leftController.setReference(leftTarget, ControlType.kPosition, 0, getLeftEffort());

      rightController.setReference(rightTarget, ControlType.kPosition);
      leftController.setReference(leftTarget, ControlType.kPosition);

    });
  }

  public Command stopMotors() {
    return runOnce(() -> {
      rightMotor.stopMotor();
      leftMotor.stopMotor();
    });
  }

  @Override
  public void periodic() {
    rightOut = rightMotor.getAppliedOutput();
    leftOut = leftMotor.getAppliedOutput();

    // Get new tuning numbers from shuffleboard
    double _kP = SmartDashboard.getNumber("Proportional", kP);
    double _kI = SmartDashboard.getNumber("Integral", kI);
    double _kD = SmartDashboard.getNumber("Derivitave", kD);

    // Check if tuning numbers changed and update controller values
    if ((_kP != kP)) {
      kP = _kP;
    }
    if ((_kI != kI)) {
      kI = _kI;
    }
    if ((_kD != kD)) {
      kD = _kD;
    }
  }

  private void configClimberSubsys() {
    // Right motor config
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setInverted(true);
    rightMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    rightMotor.enableVoltageCompensation(rightMotor.getBusVoltage());

    // Left motor config
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setInverted(true);
    leftMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    leftMotor.enableVoltageCompensation(leftMotor.getBusVoltage());

    // Encoder config
    rightEncoder.setPositionConversionFactor(kHeightConvFact);
    leftEncoder.setPositionConversionFactor(kHeightConvFact);

    // Controls config
    rightController.setP(kP);
    rightController.setI(kI);
    rightController.setD(kD);
    rightController.setFF(0.0);

    leftController.setP(kP);
    leftController.setI(kI);
    leftController.setD(kD);
    leftController.setFF(0.0);


    rightController.setFeedbackDevice(rightEncoder);
    leftController.setFeedbackDevice(leftEncoder);

    rightMotor.burnFlash();
    leftMotor.burnFlash();

  }
}
