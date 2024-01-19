/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.team3602.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem implements Subsystem {
  // Motor controllers
  // private final CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotorId,
  // MotorType.kBrushless);

  // Encoders
  // private final RelativeEncoder intakeMotorEncoder = intakeMotor.getEncoder();

  // PID controllers
  // private final SparkPIDController intakeMotorPIDController =
  // intakeMotor.getPIDController();

  // Sensors
  private boolean hasNote;
  private final DigitalInput colorSensor = new DigitalInput(1);

  public IntakeSubsystem() {
    configIntakeSubsys();

    // SmartDashboard.putBoolean("Color Sensor", hasNote);
  }

  public boolean getColorSensor() {
    return colorSensor.get();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Color Sensor", getColorSensor());

    hasNote = colorSensor.get();
  }

  // public Command runIntake(DoubleSupplier velocity) {
  // return run(() -> {
  // intakeMotorPIDController.setReference(velocity.getAsDouble(),
  // ControlType.kVelocity);
  // }).until(() -> getColorSensor());
  // }

  // public Command runIntake(DoubleSupplier percentage) {
  // return run(() -> {
  // intakeMotorPIDController.setReference(percentage.getAsDouble(),
  // ControlType.kDutyCycle);
  // });
  // }

  public Command runIntake(DoubleSupplier percentage) {
    return run(() -> {
      // intakeMotor.set(percentage.getAsDouble());
    });
  }

  public void stopIntake() {
    // intakeMotor.stopMotor();
  }

  private void configIntakeSubsys() {
    // Intake motor config
    // intakeMotor.setIdleMode(IdleMode.kBrake);
    // intakeMotor.setSmartCurrentLimit(kIntakeMotorCurrentLimit);
    // intakeMotor.enableVoltageCompensation(intakeMotor.getBusVoltage());
    // intakeMotor.burnFlash();

    // intakeMotorEncoder.setVelocityConversionFactor(kIntakeConversionFactor);

    // intakeMotorPIDController.setP(0.0);
    // intakeMotorPIDController.setI(0.0);
    // intakeMotorPIDController.setD(0.0);

  }
}
