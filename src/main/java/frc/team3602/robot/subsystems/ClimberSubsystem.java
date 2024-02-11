/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem implements Subsystem, Logged {
  // Motor controllers
  private final CANSparkMax rightMotor = new CANSparkMax(kRightClimberId, MotorType.kBrushless);
  private final CANSparkMax leftMotor = new CANSparkMax(kLeftClimberId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // Controls
  private double rightTarget, leftTarget;

  private final PIDController rightController = new PIDController(kRightP, kRightI, kRightD);
  private final ElevatorFeedforward rightFeedforward = new ElevatorFeedforward(kRS, kRG, kRV, kRA);

  private final PIDController leftController = new PIDController(kLeftP, kLeftI, kLeftD);
  private final ElevatorFeedforward leftFeedforward = new ElevatorFeedforward(kLS, kLG, kLV, kLA);

  public ClimberSubsystem() {
    SmartDashboard.putNumber("Right Proportional", kRightP);
    SmartDashboard.putNumber("Right Integral", kRightI);
    SmartDashboard.putNumber("Right Derivitave", kRightD);
    SmartDashboard.putNumber("Left Proportional", kLeftP);
    SmartDashboard.putNumber("Left Integral", kLeftI);
    SmartDashboard.putNumber("Left Derivitive", kLeftD);

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

  @Log
  private double getRightEffort() {
    var ffEffort = rightFeedforward.calculate(2.0, 0);
    var pidEffort = rightController.calculate(getRightEncoder(), rightTarget);

    return ffEffort + pidEffort;
  }

  @Log
  private double getLeftEffort() {
    var ffEffort = leftFeedforward.calculate(2.0, 0);
    var pidEffort = leftController.calculate(getLeftEncoder(), leftTarget);

    return ffEffort + pidEffort;
  }

  public Command holdHeights() {
    return runOnce(() -> {
      rightMotor.setVoltage(getRightEffort());
      leftMotor.setVoltage(getLeftEffort());
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
    // Get new tuning numbers from shuffleboard
    double rp = SmartDashboard.getNumber("Right Proportional", kRightP);
    double ri = SmartDashboard.getNumber("Right Integral", kRightI);
    double rd = SmartDashboard.getNumber("Right Derivitave", kRightD);
    double lp = SmartDashboard.getNumber("Left Proportional", kLeftP);
    double li = SmartDashboard.getNumber("Left Integral", kLeftI);
    double ld = SmartDashboard.getNumber("Left Derivitive", kLeftD);

    // Check if tuning numbers changed and updat controller values
    if ((rp != kRightP)) {
      kRightP = rp;
    }
    if ((ri != kRightI)) {
      kRightI = ri;
    }
    if ((rd != kRightD)) {
      kRightD = rd;
    }
    if ((lp != kLeftP)) {
      kLeftP = lp;
    }
    if ((li != kLeftI)) {
      kLeftI = li;
    }
    if ((ld != kLeftD)) {
      kLeftD = ld;
    }
  }

  private void configClimberSubsys() {
    // Right motor config
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    rightMotor.enableVoltageCompensation(rightMotor.getBusVoltage());

    // Left motor follower config
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(kMotorCurrentLimit);
    leftMotor.enableVoltageCompensation(leftMotor.getBusVoltage());

    // Encoder config
    rightEncoder.setPositionConversionFactor(kHeightConvFact);
    leftEncoder.setPositionConversionFactor(kHeightConvFact);

    rightMotor.burnFlash();
    leftMotor.burnFlash();

  }
}
