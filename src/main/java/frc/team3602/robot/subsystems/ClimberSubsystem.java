/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.*;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.ClimberConstants.*;

public class ClimberSubsystem implements Subsystem, Logged {
  // Motor controllers
 //@Log
 //public double motorOutput;

  private final CANSparkMax rightMotor = new CANSparkMax(kRightClimberId, MotorType.kBrushless);
  private final CANSparkMax leftMotor = new CANSparkMax(kLeftClimberId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();

  // Controls
  private double kRP, kRI, kRD, kLP, kLI, kLD;

  //@Log
  //public MutableMeasure<Angle> targetAngle;

  //public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

  private final PIDController rightController = new PIDController(kRP, kRI, kRD);
  private final ElevatorFeedforward rightFeedforward = new ElevatorFeedforward(kRS, kRG, kRV, kRA);

  private final PIDController leftController = new PIDController(kLP, kLI, kLD);
  private final ElevatorFeedforward leftFeedforward = new ElevatorFeedforward(kLS, kLG, kLV, kLA);

  public ClimberSubsystem() {
    SmartDashboard.putNumber("Right kP", kRP);
    SmartDashboard.putNumber("Right kI", kRI);
    SmartDashboard.putNumber("Right kD", kRD);
    SmartDashboard.putNumber("Left kP", kLP);
    SmartDashboard.putNumber("Left kI", kLI);
    SmartDashboard.putNumber("Left kD", kLD);

    resetEncoders();
    configClimberSubsys();
  }
  
  public void resetEncoders(){
    rightEncoder.setPosition(0.0);
    leftEncoder.setPosition(0.0);
  }
  
  public void getRightEncoder(){
    rightEncoder.getPosition();
  }
  
  public void getLeftEncoder(){
    leftEncoder.getPosition();
  }

  public Command setRightHeight(DoubleSupplier rightTarget) {
    return runOnce(() -> targetAngle = angleDegrees.get().mutableCopy());
  }

  private double getRightEffort() {
    var ffEffort = rightFeedforward.calculate(2.0, 0);
    var pidEffort = rightController.calculate(getRightEncoder(), targetAngle.in(Degrees));

    return ffEffort + pidEffort;
  }

  public Command stopMotors() {
    return runOnce(() -> {
      pivotMotor.stopMotor();
      pivotFollower.stopMotor();
    });
  }

  @Override
  public void periodic() {
    motorOutput = pivotMotor.getAppliedOutput();

    kP = SmartDashboard.getNumber("Pivot kP", 0);
    kI = SmartDashboard.getNumber("Pivot kI", 0);
    kD = SmartDashboard.getNumber("Pivot kD", 0);
  }

  private void configClimberSubsys() {
    // Right motor config
    rightMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setSmartCurrentLimit(kRightMotorCurrentLimit);
    rightMotor.enableVoltageCompensation(rightMotor.getBusVoltage());

    // Left motor follower config
    leftMotor.setIdleMode(IdleMode.kBrake);
    leftMotor.setSmartCurrentLimit(kLeftMotorCurrentLimit);
    leftMotor.enableVoltageCompensation(leftMotor.getBusVoltage());

    // Encoder config
    rightEncoder.setPositionConversionFactor(kPivotConversionFactor);

    rightMotor.burnFlash();
    leftMotor.burnFlash();

    // Interpolation table config
    lerpTable.put(5.0, 25.0); // 5 feet, 25 degrees
    lerpTable.put(10.0, 35.0); // 10 feet, 35 degrees
  }
}
