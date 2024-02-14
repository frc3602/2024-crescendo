/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.*;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class PivotSubsystemTwo extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  public double motorOutput;

  @Log
  public double motorOutputTwo;

  private final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);

  // Encoders
  private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  // Controls
  private double kP, kI, kD;

  @Log
  public double encoderValue;

  @Log
  public double angle;

  @Log
  public double effort;

  @Log
  public double ffEffort;

  @Log
  public double pidEffort;

  public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

  private final PIDController controller = new PIDController(0.001, 0, 0.0);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(20, 40));

  public PivotSubsystemTwo() {
    SmartDashboard.putNumber("Pivot kP", kP);
    SmartDashboard.putNumber("Pivot kI", kI);
    SmartDashboard.putNumber("Pivot kD", kD);

    controller.setSetpoint(45);

    configPivotSubsys();
  }

  private double getDegrees() {
    return pivotEncoder.getPosition();
  }

  public Command setAngle(DoubleSupplier angleDegrees) {
    return runOnce(() -> controller.setSetpoint(angleDegrees.getAsDouble()));
  }

  private double getEffort() {
    var ffEffort = feedforward.calculate(Units.degreesToRadians(controller.getSetpoint()), 0);
    var pidEffort = controller.calculate(pivotEncoder.getPosition(), controller.getSetpoint());

    this.ffEffort = ffEffort;
    this.pidEffort = pidEffort;

    return ffEffort + pidEffort;
  }

  public Command holdAngle() {
    return run(() -> {
      var effort = getEffort();
      this.effort = effort;

      pivotMotor.setVoltage(effort);
    });
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

    motorOutputTwo = pivotFollower.getAppliedOutput();

    encoderValue = pivotEncoder.getPosition();

    angle = controller.getSetpoint();

    kP = SmartDashboard.getNumber("Pivot kP", 0);
    kI = SmartDashboard.getNumber("Pivot kI", 0);
    kD = SmartDashboard.getNumber("Pivot kD", 0);
  }

  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(true);
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());

    // Pivot motor follower config
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);
    pivotFollower.setSmartCurrentLimit(kPivotFollowerCurrentLimit);
    pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());

    // Pivot encoder config
    // pivotEncoder.setPositionConversionFactor(kPivotConversionFactor);

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();

    controller.setTolerance(1);
    controller.enableContinuousInput(0, 45);

    // Interpolation table config
    lerpTable.put(5.0, 25.0); // 5 feet, 25 degrees
    lerpTable.put(10.0, 35.0); // 10 feet, 35 degrees
  }
}