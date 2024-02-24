/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class PivotSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  public double motorOutput, motorOutputTwo;

  public final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);

  // Encoders
  private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  // Controls
  @Log
  public double encoderValue;

  public double angle = 0;

  @Log
  public double effort;

  public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

  // private final TrapezoidProfile.Constraints constraints = new
  // TrapezoidProfile.Constraints(kMaxVelocity,
  // kMaxAcceleration);
  // private final TrapezoidProfile.State previousProfiledReference = new
  // TrapezoidProfile.State(0, 0);
  // private final TrapezoidProfile profile = new TrapezoidProfile(constraints);

  private final SparkPIDController controller = pivotMotor.getPIDController();
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  public PivotSubsystem() {
    SmartDashboard.putNumber("Angle", angle);

    configPivotSubsys();
  }

  private double getDegrees() {
    return pivotEncoder.getPosition();
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

  private double getEffort() {
    return feedforward.calculate(Units.degreesToRadians(angle), 0);
  }

  public Command holdAngle() {
    return run(() -> {
      var effort = getEffort();
      this.effort = effort;

      // var goal = new TrapezoidProfile.State(angle, 0);
      // var setpoint = profile.calculate(0, previousProfiledReference, goal);

      controller.setReference(angle, ControlType.kPosition, 0, effort, ArbFFUnits.kVoltage);
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

    var angle = SmartDashboard.getNumber("Angle", this.angle);

    if (angle != this.angle) {
      this.angle = angle;
    }
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
    pivotEncoder.setZeroOffset(255.0);

    // pivotEncoder.setPositionConversionFactor(kPivotConversionFactor);
    // double offset = pivotEncoder.getZeroOffset();
    // if (getDegrees() > 300.0) {
    // pivotEncoder.setZeroOffset(offset + (360 - getDegrees()));
    // }

    controller.setFeedbackDevice(pivotEncoder);

    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);

    // controller.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    // controller.setSmartMotionMaxAccel(kMaxAcceleration, 0);
    // controller.setSmartMotionMaxVelocity(kMaxVelocity, 0);

    // controller.setPositionPIDWrappingEnabled(true);
    // controller.setPositionPIDWrappingMinInput(0);
    // controller.setPositionPIDWrappingMaxInput(90);

    // controller.setTolerance(1);
    // controller.enableContinuousInput(0, 45);

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();

    // Interpolation table config
    lerpTable.put(5.0, 32.0); // 5 feet, 25 degrees
    lerpTable.put(10.0, 48.0); // 10 feet, 39.5 degrees
    lerpTable.put(15.0, 43.0); // 15 feet, 43 degrees
    lerpTable.put(20.0, 53.5); // 20 feet, 44.5 degrees
    lerpTable.put(25.0, 41.0); // 25 feet, 41 degrees
  }
}
