/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  private final CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);

  // Sensors
  @Log
  private boolean color;

  @Log
  public boolean anotherBool;

  @Log
  public boolean isChanged;

  private final DigitalInput colorSensor = new DigitalInput(1);
  // private final SparkLimitSwitch colorSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);

  public IntakeSubsystem() {
    SmartDashboard.putBoolean("Color Sensor", getColorSensor());

    configIntakeSubsys();
  }

  public boolean getColorSensor() {
    var colorSensor = !this.colorSensor.get();

    anotherBool = colorSensor;

    return colorSensor;
  }

  public Command runIntake(DoubleSupplier percentage) {
    return runOnce(() -> intakeMotor.set(percentage.getAsDouble()));
  }

  public Command intake(DoubleSupplier percentage) {
    return new FunctionalCommand(() -> {

    }, () -> {
      intakeMotor.set(percentage.getAsDouble());

      if (getColorSensor()) {
        CommandScheduler.getInstance().cancelAll();
      }
    }, (onEnd) -> {
      CommandScheduler.getInstance().cancelAll();
    },
        this::getColorSensor, this);
  }

  // public Command isChanged() {
  // return runOnce(() -> isChanged = true);
  // }

  // public Command intakeParallel() {
  // return Commands.parallel(
  // isChanged(),
  // runIntake()
  // );
  // }

  public Command stopIntake() {
    return runOnce(() -> intakeMotor.stopMotor());
  }

  @Override
  public void periodic() {
    isChanged = getColorSensor();

    color = getColorSensor();
  }

  private void configIntakeSubsys() {
    // Intake motor config
    intakeMotor.restoreFactoryDefaults(true);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(kIntakeMotorCurrentLimit);
    intakeMotor.enableVoltageCompensation(intakeMotor.getBusVoltage());
    intakeMotor.setOpenLoopRampRate(0.000001);

    // colorSensor.enableLimitSwitch(true);
    intakeMotor.burnFlash();
  }
}
