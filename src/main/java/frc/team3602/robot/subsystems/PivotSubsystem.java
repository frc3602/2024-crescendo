/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

public class PivotSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  @Log
  public final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);
  private final SparkLimitSwitch lowLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
  private final SparkLimitSwitch highLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  @Log
  public DoubleSupplier motorOutput = () -> pivotMotor.getAppliedOutput();
  public DoubleSupplier motorOutputTwo = () -> pivotFollower.getAppliedOutput();

  // Encoders
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);
  // private final SparkAbsoluteEncoder pivotEncoder =
  // pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
// motors encoder
// private final RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder();
//   private final RelativeEncoder pivotMotorFollower = pivotFollower.getEncoder();
  
  // Controls
  @Log
  public BooleanSupplier isAtPosition = () -> atPosition();
  public BooleanSupplier lowLimit = () -> lowLimitSwitch.isPressed();
  public BooleanSupplier highLimit = () -> highLimitSwitch.isPressed();

  @Log
  public DoubleSupplier encoderValue = () -> getDegrees();

  @Log
  public double angle = 84.0;
  public DoubleSupplier voltage;

  @Log
  public double effort;
  public double ffEffort;
  public double pidEffort;

  private final Vision vision = new Vision();

  public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();

  @Log
  public DoubleSupplier lerpAngle = () -> lerpTable.get(Units.metersToFeet(vision.getTargetDistance()));
  public double absoluteOffset = 77;

  private final PIDController controller = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  public PivotSubsystem() {
    SmartDashboard.putNumber("Angle", angle);

    configPivotSubsys();
  }

  //getDegrees was private
  public double getDegrees() {
    return (pivotEncoder.getAbsolutePosition() * 360) - absoluteOffset;
  }

  public boolean atPosition() {
    var target = angle;
    var tolerance = 2;

    return ((MathUtil.isNear(target, getDegrees(), tolerance)));
  }

  public double getEffort() {
    double ffEffort = feedforward.calculate(Units.degreesToRadians(getDegrees()), 0);

    double pidEffort = controller.calculate(getDegrees(), angle);

    // For Logging
    this.ffEffort = ffEffort;
    this.pidEffort = pidEffort;
    this.effort = (ffEffort + pidEffort);
    
    return ffEffort + pidEffort;
  }

  /*
   * We overload setAngle for convenience.
   * Passing an angle only results in the use of that angle and the PID control determining voltage.
   * Passing a voltage only results in no change of set angle and the use of that voltage.
   * Passing both an angle and a voltage results in the use of that angle and that voltage.
   * 
   * If one want to employ the lerptable, one ought not use setAngle and runPivot but runPivotWithLerpTable.
   */
  public Command setAngle(double angle) {
    return runOnce(() -> {
      this.angle = angle;
      voltage = () -> getEffort();
    });
  }

  public Command setAngle(DoubleSupplier voltage) {
    return runOnce(() -> {
      this.voltage = voltage;
    });
  }

  public Command setAngle(double angle, DoubleSupplier voltage) {
    return runOnce(() -> {
      this.angle = angle;
      this.voltage = voltage;
    });
  }

  /*
   * runPivot works the same as holdAngle; it exists only to differentiate between default and abnormal action.
   * Also, it allows the use of a given voltage rather than the PID control.
   * 
   * If one want to employ the lerptable, one ought not use setAngle and runPivot but runPivotWithLerpTable.
   */
  public Command runPivot() {
    return run(() -> {
      pivotMotor.setVoltage(voltage.getAsDouble());
    });
  }

  public Command runPivotWithLerpTable() {
    return run(() -> {
     angle = ((DoubleSupplier) lerpTable).getAsDouble();
     pivotMotor.setVoltage(getEffort());
    });
  }

  public Command holdAngle() {
    return run(() -> {
      pivotMotor.setVoltage(getEffort());
    });
  }


  //same as holdAngle, but uses only feedforward
  public Command holdPosition() {
    return run(() -> {
      pivotMotor.setVoltage(  feedforward.calculate(Units.degreesToRadians(getDegrees()), 0));
    });
  }

  
  @Override
  public void periodic() {
  }

  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());

    // Pivot motor follower config
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);
    pivotFollower.setSmartCurrentLimit(kPivotFollowerCurrentLimit);
    pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());
    controller.setTolerance(1);

    lowLimitSwitch.enableLimitSwitch(true);
    highLimitSwitch.enableLimitSwitch(true);

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();

    // Interpolation table config
    lerpTable.put(4.6, 32.0); // 4.6 feet, 32 degrees
    lerpTable.put(6.87, 42.0); // 7.65 feet, 42 degrees
    lerpTable.put(8.33,44.0); 
    lerpTable.put(9.91,48.0); 
    lerpTable.put(10.95, 50.0); // 10.7 feet, 49.75 degrees
    lerpTable.put(11.4,51.0); 
    lerpTable.put(12.53,50.0 ); 
    lerpTable.put(13.47, 51.0); // 13.79 feet, 53 degrees
    lerpTable.put(15.24,51.0 ); 
    lerpTable.put(16.0, 50.0); // 16.9 feet, 52.5 degrees
  }
}
