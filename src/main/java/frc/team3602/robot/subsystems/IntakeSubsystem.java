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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.IntakeConstants.*;

public class IntakeSubsystem extends SubsystemBase implements Logged {
  // Motor controllers
  public final CANSparkMax intakeMotor = new CANSparkMax(kIntakeMotorId, MotorType.kBrushless);

  private XboxController xboxController;

  // Sensors
  @Log
  private boolean hasNote;

  // private final SparkLimitSwitch colorSensor = intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);
  public final DigitalInput colorSensor = new DigitalInput(1);

  public IntakeSubsystem(XboxController xboxController) {
    this.xboxController = xboxController;
    configIntakeSubsys();
  }

  public boolean getColorSensor() {
    return colorSensor.get();
  }

  public Command runIntakeTwo(DoubleSupplier percentage) {
    return startEnd(() -> intakeMotor.set(percentage.getAsDouble()), this::stopIntake);
  }

  public Command runIntake() {
    return run(() -> {
      if (xboxController.getAButton() && !getColorSensor()) {
        intakeMotor.set(0.15);
      } else {
        intakeMotor.set(0.0);
      }
    });
  }

  public Command stopIntake() {
    return runOnce(() -> intakeMotor.set(0.0));
  }

  @Override
  public void periodic() {
    hasNote = getColorSensor();
  }

  private void configIntakeSubsys() {
    // Intake motor config
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setSmartCurrentLimit(kIntakeMotorCurrentLimit);
    intakeMotor.enableVoltageCompensation(intakeMotor.getBusVoltage());
    intakeMotor.setOpenLoopRampRate(0.000001);

    // Sensors config
    // colorSensor.enableLimitSwitch(true);

    intakeMotor.burnFlash();
  }
}
