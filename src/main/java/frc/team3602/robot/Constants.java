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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.subsystems.DrivetrainSubsystem;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
  }
  public final class ClimberConstants {
    public static final int kRightClimberId = 10;
    public static final int kLeftClimberId =11;

    public static final int kMotorCurrentLimit = 30;

    /*Conversion factor in inches per turn calculated as follows:
     Drum diameter*pi/gear ratio
    */
    public static final double kHeightConvFact = (2.0*Math.PI)/5.0;

    //Retracted height of arms from floor. The arm postion will be set to this on init
    public static final double kRetractedHeight=0.0;

    //Right PID tuning constants :)
    public static double kRightP = 0.0; //TODO: set final after tuning
    public static double kRightI = 0.0; //TODO: set final after tuning
    public static double kRightD = 0.0; //TODO: set final after tuning

    //Right feedforward Constants
    public static final double kRS = 0.0;
    public static final double kRG = 0.0;
    public static final double kRV = 0.0;
    public static final double kRA = 0.0;

    //Left PID tuning constants (:
    public static double kLeftP = 0.0; //TODO: set final after tuning
    public static double kLeftI = 0.0; //TODO: set final after tuning
    public static double kLeftD = 0.0; //TODO: set final after tuning

    //Left feedforward Constants
    public static final double kLS = 0.0;
    public static final double kLG = 0.0;
    public static final double kLV = 0.0;
    public static final double kLA = 0.0;

  }

  public final class ShooterConstants {
    public static final int kTopShooterMotorId = 2;
    public static final int kBottomShooterMotorId = 3;

    public static final int kTopShooterMotorCurrentLimit = 30;
    public static final int kBottomShooterMotorCurrentLimit = 30;

    public static final double kShooterConversionFactor = 360;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

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

    public static final int kPivotMotorCurrentLimit = 40;
    public static final int kPivotFollowerCurrentLimit = 40;

    public static final double kPivotConversionFactor = 360;
    public static final double kAbsoluteOffset = 0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 6.8;
    public static final double kG = 5.09;
    public static final double kV = 0.44;
    public static final double kA = 0.46;

    public static final Measure<Angle> kInFramePos = Degrees.of(45);
    public static final Measure<Angle> kPickupPos = Degrees.of(90);
  }

  public final class VisionConstants {
    public static final String kPhotonCameraName = "photonvision";

    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0.0, 0.0, 0.0));

    public static final Measure<Distance> kCameraHeight = Feet.of(1.4375);
    public static final Measure<Angle> kCameraPitch = Degrees.of(0.0);

    public static final Measure<Distance> kGoalRange = Feet.of(9.0);
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
}
