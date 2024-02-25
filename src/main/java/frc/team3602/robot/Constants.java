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

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
  }

  public final class ClimberConstants {
    public static final int kRightClimberId = 10;
    public static final int kLeftClimberId = 11;

    public static final int kMotorCurrentLimit = 30;

    /**
     * Conversion factor in inches per turn calculated as follows:
     * Drum diameter*pi/gear ratio
     */
    public static final double kHeightConvFact = (2.0 * Math.PI) / 25.0;

    // Retracted height of arms from floor. The arm postion will be set to this on
    // init
    public static final double kRetractedHeight = 47.75;

    // PID tuning constants :)
    public static double kP = 10.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // Feedforward Constants
    public static final double kS = 6.38;
    public static final double kG = 4.76;
    public static final double kV = 1.53;
    public static final double kA = 0.53;
  }

  public final class ShooterConstants {
    public static final int kTopShooterMotorId = 2;
    public static final int kBottomShooterMotorId = 3;

    public static final int kTopShooterMotorCurrentLimit = 25;
    public static final int kBottomShooterMotorCurrentLimit = 25;

    public static final double kTopConvFactor = (Math.PI * 4.0);

    public static final double kP = 2.0;
    public static final double kI = 0.0;
    public static final double kD = 0.1;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
  }

  public final class IntakeConstants {
    public static final int kIntakeMotorId = 5;
    public static final int kIntakeMotorCurrentLimit = 30;

    public static final int kColorSensorId = 0;
  }

  public final class PivotConstants {
    public static final int kPivotMotorId = 4;
    public static final int kPivotFollowerId = 6; // TODO: CHANGE MOTOR CONTROLLER TO CAN ID 6

    public static final int kPivotMotorCurrentLimit = 20;
    public static final int kPivotFollowerCurrentLimit = 20;

    public static final double kPivotConversionFactor = 360;
    public static final double kAbsoluteOffset = 0;

    public static final double kP = 0.02;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 6.8; // 6.8
    public static final double kG = 0.49; // 0.49
    public static final double kV = 4.68; // 4.68
    public static final double kA = 0.03; // 0.03

    public static final Measure<Angle> kInFramePos = Degrees.of(45);
    public static final Measure<Angle> kPickupPos = Degrees.of(90);

    public static final double kMaxVelocity = 3; // meters/second 0.15
    public static final double kMaxAcceleration = 6; // meters/second^2 0.50
  }

  public final class VisionConstants {
    public static final String kPhotonCameraName = "photonvision";

    // Camera mounted facing forward, half a meter forward of center, half a meter
    // up from center. TODO: Measure this
    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0.0, 0.0, 0.0));

    public static final Measure<Distance> kCameraHeight = Feet.of(1.3);
    public static final Measure<Angle> kCameraPitch = Degrees.of(0.0);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  }

  public final class DrivetrainConstants {
    public static final double kMaxSpeed = 6.0;
    public static final double kMaxAngularRate = Math.PI;

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
    public static final double kSpeedAt12VoltsMps = 6.0;
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
    private static final double kFrontLeftEncoderOffset = 0.098388671875;
    private static final double kFrontLeftXPosInches = 10.25;
    private static final double kFrontLeftYPosInches = 10.25;

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
    private static final double kFrontRightEncoderOffset = -0.48583984375;
    private static final double kFrontRightXPosInches = 10.25;
    private static final double kFrontRightYPosInches = -10.25;

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
    private static final double kBackLeftEncoderOffset = 0.450927734375;
    private static final double kBackLeftXPosInches = -10.25;
    private static final double kBackLeftYPosInches = 10.25;

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
    private static final double kBackRightEncoderOffset = 0.27880859375;
    private static final double kBackRightXPosInches = -10.25;
    private static final double kBackRightYPosInches = -10.25;

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
}
