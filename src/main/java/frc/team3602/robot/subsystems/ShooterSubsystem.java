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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.*;

import static frc.team3602.robot.Constants.ShooterConstants.*;

import java.util.function.Supplier;

import monologue.Logged;
import monologue.Annotations.Log;

public class ShooterSubsystem implements Subsystem, Logged {
  // Motor controllers
  private final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
  private final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder topShooterMotorEncoder = topShooterMotor.getEncoder();

  // Controls
  private final InterpolatingTreeMap lerpTable = new InterpolatingTreeMap<Measure<Distance>, Measure<Angle>>();

  private final PIDController controller = new PIDController(kP, kI, kD);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

  public ShooterSubsystem() {
    configShooterSubsys();
  }

  @Log
  private Measure<Velocity<Angle>> getVelocity() {
    return RPM.of(topShooterMotorEncoder.getVelocity());
  }

  private double getEffort(Measure<Velocity<Angle>> velocityRPM) {
    var ffEffort = feedforward.calculate(getVelocity().in(RPM), 0);
    var pidEffort = controller.calculate(getVelocity().in(RPM), velocityRPM.in(RPM));

    return ffEffort + pidEffort;
  }

  public Command runShooter(Supplier<Measure<Velocity<Angle>>> velocityRPM) {
    return run(() -> {
      topShooterMotor.setVoltage(getEffort(velocityRPM.get()));
    });
  }

  public Command stopMotors() {
    return runOnce(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
    });
  }

  private void configShooterSubsys() {
    // Top shooter motor config
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.setSmartCurrentLimit(kTopShooterMotorCurrentLimit);
    topShooterMotor.enableVoltageCompensation(topShooterMotor.getBusVoltage());
    topShooterMotor.setOpenLoopRampRate(0.01);

    // Bottom shooter motor config
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.follow(topShooterMotor, false);
    bottomShooterMotor.setSmartCurrentLimit(kBottomShooterMotorCurrentLimit);
    bottomShooterMotor.enableVoltageCompensation(bottomShooterMotor.getBusVoltage());
    bottomShooterMotor.setOpenLoopRampRate(0.01);

    // Shooter encoder config
    topShooterMotorEncoder.setVelocityConversionFactor(kShooterConversionFactor);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();
  }
}
