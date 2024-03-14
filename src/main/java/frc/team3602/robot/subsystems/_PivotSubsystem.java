/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team3602.robot.Vision;
import static frc.team3602.robot.Constants.PivotConstants.*;

import monologue.Logged;
import monologue.Annotations.Log;

public class _PivotSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  public double motorOutput, motorOutputTwo;

  public final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);

  // Encoders
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);
  // private final SparkAbsoluteEncoder pivotEncoder =
  // pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
// motors encoder
// private final RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder();
//   private final RelativeEncoder pivotMotorFollower = pivotFollower.getEncoder();
  
  // Controls
  @Log
  public boolean isAtPosition;

  @Log
  public double encoderValue;

  @Log
  public double angle = 84.0;
  @Log
  public double lerpAngle;
  public double absoluteOffset = 77;

  @Log
  public double effort;

  @Log
  public double ffEffort;

  @Log
  public double pidEffort;

  private final Vision vision = new Vision();

  public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

  private final PIDController controller = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  public _PivotSubsystem() {
    SmartDashboard.putNumber("Angle", angle);

    configPivotSubsys();
  }

  private double getDegrees() {
    return (pivotEncoder.getAbsolutePosition() * 360) - absoluteOffset;
  }

  public boolean atPosition() {
    var target = angle;
    var tolerance = 3;

    return ((MathUtil.isNear(target, getDegrees(), tolerance))&&(angle <= 80));
  }

  public Command runPivot(DoubleSupplier percentage) {
    return run(() -> {
      pivotMotor.set(percentage.getAsDouble());
    });
  }

  public Command setAngle(DoubleSupplier angleDegrees) {
    return runOnce(() -> {
      angle = angleDegrees.getAsDouble();
    });
  }

  public Command runSetAngle(DoubleSupplier angleDegrees) {
    return run(() -> {
      angle = angleDegrees.getAsDouble();

      var effort = getEffort();
      this.effort = effort;

      pivotMotor.setVoltage(effort);
    });
  }

  private double getEffort() {
    // var ffEfort = feedforward.calculate(Units.degreesToRadians(angle), 0);
    var ffEfort = feedforward.calculate(Units.degreesToRadians(getDegrees()), 0);

    var pidEffort = controller.calculate(getDegrees(), angle);

    this.ffEffort = ffEfort;
    this.pidEffort = pidEffort;

    return ffEfort + pidEffort;
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

    encoderValue = getDegrees();

    isAtPosition = atPosition();

    lerpAngle = lerpTable.get(vision.getTargetDistance());

    // var angle = SmartDashboard.getNumber("Angle", this.angle);

    // if (angle != this.angle) {
    // MathUtil.clamp(angle, 0, 130);
    // this.angle = angle;
    // }
  }

  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(true);
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());
    // pivotMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);

    // Pivot motor follower config
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);
    pivotFollower.setSmartCurrentLimit(kPivotFollowerCurrentLimit);
    pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());

    // pivotEncoder.setPositionConversionFactor(kPivotConversionFactor);
    // double offset = pivotEncoder.getZeroOffset();
    // if (getDegrees() > 300.0) {
    // pivotEncoder.setZeroOffset(offset + (360 - getDegrees()));
    // }

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();

    // Interpolation table config
    lerpTable.put(4.6, 32.0); // 4.6 feet, 32 degrees
    lerpTable.put(7.65, 42.0); // 7.65 feet, 42 degrees
    lerpTable.put(10.7, 49.75); // 10.7 feet, 49.75 degrees
    lerpTable.put(13.79, 53.0); // 13.79 feet, 53 degrees
    lerpTable.put(16.9, 52.5); // 16.9 feet, 52.5 degrees
    lerpTable.put(20.6, 45.0); // 20.6 feet, 45 degrees

  }
}
