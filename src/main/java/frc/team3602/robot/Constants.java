/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
  }

  public final class ShooterConstants {
    public static final int kTopShooterMotorId = 67;
    public static final int kTopShooterMotorCurrentLimit = 30;

    public static final int kBottomShooterMotorId = 68;
    public static final int kBottomShooterMotorCurrentLimit = 30;
  }

  public final class IntakeConstants {
    public static final int kIntakeMotorId = 69;
    public static final int kIntakeMotorCurrentLimit = 30;

    public static final double kIntakeConversionFactor = 0.0;
  }

  public final class DrivetrainConstants {
    public static final Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(6.0);
    public static final Measure<Velocity<Distance>> kMaxAngularRate = MetersPerSecond.of(Math.PI);

    private final static int kPigeonId = 52;
    private final static String kCANBusName = "rio";

    private static final double kDriveGearRatio = 6.122448979591837;
    private static final double kTurnGeatRatio = 21.428571428571427;
    private static final double kWheelRadiusInches = 2.0;
    private static final double kSlipCurrentAmps = 300.0;
    private static final Slot0Configs kDriveGains = new Slot0Configs()
        .withKP(3.0).withKI(0.0).withKD(0.0)
        .withKS(0.0).withKV(0.0).withKA(0.0);
    private static final Slot0Configs kTurnGains = new Slot0Configs()
        .withKP(100.0).withKI(0.0).withKD(0.05)
        .withKS(0.0).withKV(1.5).withKA(0.0);
    private static final double kSpeedAt12VoltsMps = 6.0;
    private static final double kCoupleGearRatio = 3.5714285714285716;
    private static final boolean kTurnMotorInverted = true;

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final SwerveDrivetrainConstants kDrivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(kPigeonId)
        .withCANbusName(kCANBusName);

    private static final SwerveModuleConstantsFactory kModuleConstants = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(kDriveGearRatio)
        .withSteerMotorGearRatio(kTurnGeatRatio)
        .withWheelRadius(kWheelRadiusInches)
        .withSlipCurrent(kSlipCurrentAmps)
        .withDriveMotorGains(kDriveGains)
        .withSteerMotorGains(kTurnGains)
        .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
        .withCouplingGearRatio(kCoupleGearRatio)
        .withSteerMotorInverted(kTurnMotorInverted)
        .withFeedbackSource(SteerFeedbackType.FusedCANcoder);

    // Front left (Module 0)
    private static final int kFrontLeftTurnMotorId = 49;
    private static final int kFrontLeftDriveMotorId = 46;
    private static final int kFrontLeftEncoderId = 41;
    private static final double kFrontLeftEncoderOffset = 0.112548828125;
    private static final double kFrontLeftXPosInches = 12.875;
    private static final double kFrontLeftYPosInches = 12.875;

    private static final SwerveModuleConstants kFrontLeftModuleConstants = kModuleConstants
        .createModuleConstants(
            kFrontLeftTurnMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),
            Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);

    // Front right (Module 1)
    private static final int kFrontRightTurnMotorId = 45;
    private static final int kFrontRightDriveMotorId = 44;
    private static final int kFrontRightEncoderId = 42;
    private static final double kFrontRightEncoderOffset = -0.4921875;
    private static final double kFrontRightXPosInches = 12.875;
    private static final double kFrontRightYPosInches = -12.875;

    private static final SwerveModuleConstants kFrontRightModuleConstants = kModuleConstants
        .createModuleConstants(
            kFrontRightTurnMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);

    // Back left (Module 2)
    private static final int kBackLeftTurnMotorId = 50;
    private static final int kBackLeftDriveMotorId = 51;
    private static final int kBackLeftEncoderId = 40;
    private static final double kBackLeftEncoderOffset = 0.429931640625;
    private static final double kBackLeftXPosInches = -12.875;
    private static final double kBackLeftYPosInches = 12.875;

    private static final SwerveModuleConstants kBackLeftModuleConstants = kModuleConstants
        .createModuleConstants(
            kBackLeftTurnMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches),
            Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);

    // Back right (Module 3)
    private static final int kBackRightTurnMotorId = 47;
    private static final int kBackRightDriveMotorId = 48;
    private static final int kBackRightEncoderId = 43;
    private static final double kBackRightEncoderOffset = 0.26904296875;
    private static final double kBackRightXPosInches = -12.875;
    private static final double kBackRightYPosInches = -12.875;

    private static final SwerveModuleConstants kBackRightModuleConstants = kModuleConstants
        .createModuleConstants(
            kBackRightTurnMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches),
            Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);

    public static final DrivetrainSubsystem kDrivetrainSubsys = new DrivetrainSubsystem(
        kDrivetrainConstants,
        kFrontLeftModuleConstants,
        kFrontRightModuleConstants,
        kBackLeftModuleConstants,
        kBackRightModuleConstants);
  }

  public final class VisionConstants {
    public static final String kPhotonCameraName = "photonvision";

    public static final double kCameraHeightMeters = Units.feetToMeters(1.4375);
    public static final double kTargetHeightMeters = Units.inchesToMeters(46.0);
    public static final double kCameraPitchRadians = Units.degreesToRadians(0.0);

    public static final double kGoalRangeMeters = Units.feetToMeters(3);
  }
}
