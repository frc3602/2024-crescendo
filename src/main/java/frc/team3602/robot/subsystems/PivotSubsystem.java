/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.*;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.PivotConstants.*;

public class PivotSubsystem implements Subsystem, Logged {
  // Motor controllers
  @Log
  public double motorOutput;

  private final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);

  // Encoders
  private final SparkAbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);

  // Controls
  private double kP, kI, kD;

  @Log
  public MutableMeasure<Angle> targetAngle;

  private final PIDController controller = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  public PivotSubsystem() {
    SmartDashboard.putNumber("Pivot kP", kP);
    SmartDashboard.putNumber("Pivot kI", kI);
    SmartDashboard.putNumber("Pivot kD", kD);

    configPivotSubsys();
  }

  @Log
  private Measure<Angle> getDegrees() {
    return Degrees.of(pivotEncoder.getPosition());
  }

  public Command setAngle(Supplier<Measure<Angle>> angleDegrees) {
    return runOnce(() -> targetAngle = angleDegrees.get().mutableCopy());
  }

  private double getEffort() {
    var ffEffort = feedforward.calculate(getDegrees().in(Radians), 0);
    var pidEffort = controller.calculate(getDegrees().in(Degrees), targetAngle.in(Degrees));

    return ffEffort + pidEffort;
  }

  public Command holdAngle() {
    return run(() -> {
      pivotMotor.setVoltage(getEffort());
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

    kP = SmartDashboard.getNumber("Pivot kP", 0);
    kI = SmartDashboard.getNumber("Pivot kI", 0);
    kD = SmartDashboard.getNumber("Pivot kD", 0);
  }

  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());

    // Pivot motor follower config
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, false);
    pivotFollower.setSmartCurrentLimit(kPivotFollowerCurrentLimit);
    pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());

    // Pivot encoder config
    pivotEncoder.setVelocityConversionFactor(kPivotConversionFactor);

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();
  }
}
