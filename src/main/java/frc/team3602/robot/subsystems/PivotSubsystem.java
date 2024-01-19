// /*
//  * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
//  * is licensed under the terms of the MIT license which can be found
//  * in the root directory of this project.
//  */

// package frc.team3602.robot.subsystems;

// import java.util.function.DoubleSupplier;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.ControlType;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.CANSparkLowLevel.MotorType;

// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// import static frc.team3602.robot.Constants.PivotConstants.*;

// public class PivotSubsystem implements Subsystem {
//   // Motor controllers
//   private final CANSparkMax pivotMotor = new CANSparkMax(kPivotMotorId, MotorType.kBrushless);

//   // Encoders
//   private final RelativeEncoder pivotMotorEncoder = pivotMotor.getEncoder();

//   // PID controllers
//   private final SparkPIDController pivotMotorPIDController = pivotMotor.getPIDController();

//   public PivotSubsystem() {
//     configPivotSubsys();
//   }


//   private void configPivotSubsys() {
//     // Intake motor config
//     pivotMotor.setIdleMode(IdleMode.kBrake);
//     pivotMotor.setSmartCurrentLimit(kPivotMotorCurrentLimit);
//     pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());
//     pivotMotor.burnFlash();

//     pivotMotorEncoder.setVelocityConversionFactor(0.0);

//     pivotMotorPIDController.setP(0.0);
//     pivotMotorPIDController.setI(0.0);
//     pivotMotorPIDController.setD(0.0);
//   }
// }
