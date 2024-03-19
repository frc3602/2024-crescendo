/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;
import static edu.wpi.first.units.Units.*;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem, Logged {
  // Drivetrain
  @Log
  public double heading;
  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(0.04)
      .withRotationalDeadband(0.04);

  // Vision
  private final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final PhotonCamera photonCamera = new PhotonCamera(kPhotonCameraName);
  private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, kRobotToCamera);

 private double visionLastEstimateTimestamp = 0.0;

  private final PIDController turnController = new PIDController(0.015, 0.0, 0.001);

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, odometryUpdateFrequency, moduleConstants);

    configPathPlanner();
    configDriveSubsys();
  }

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, moduleConstants);

    configPathPlanner();
    configDriveSubsys();
  }

  public PhotonPipelineResult visionGetLatestResult(PhotonCamera photonCamera) {
    return photonCamera.getLatestResult();
  }

  public Optional<EstimatedRobotPose> visionGetEstimatedRobotPose(PhotonCamera photonCamera, PhotonPoseEstimator photonPoseEstimator) {
    var visionEstimate = photonPoseEstimator.update();
    double latestTimestamp = visionGetLatestResult(photonCamera).getTimestampSeconds();
    visionLastEstimateTimestamp = ( Math.abs(latestTimestamp - visionLastEstimateTimestamp) > 1e-5) ? latestTimestamp : visionLastEstimateTimestamp;

    return visionEstimate;
  }

  public double getTargetHeight() {
    var result = visionGetLatestResult(photonCamera);
   
    return result.hasTargets() ? kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ() : 0.0;
  }

  public double getTargetDistance() {
    double targetDistance;

    var result = visionGetLatestResult(photonCamera);

    if (result.hasTargets()) {
      targetDistance = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeight.in(Meters), getTargetHeight(),
          kCameraPitch.in(Radians), Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      targetDistance = 0.0;
    }

    return targetDistance;
  }

  @Override
  public void periodic() {
    heading = this.m_pigeon2.getYaw().getValueAsDouble();
    updateOdometry();
  }

  public Pose2d getPose() {
    return this.m_odometry.getEstimatedPosition();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command alignWithTarget() {
    return run(() -> {
      var result = visionGetLatestResult(photonCamera);

      this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, (result.hasTargets() ? turnController.calculate(result.getBestTarget().getYaw(), 0.0) : 0.0))));
    });
  }

  private void updateOdometry() {
    var pose = visionGetEstimatedRobotPose(photonCamera, photonPoseEstimator);

    if (pose.isPresent()) {
      var estimatedRobotPose = pose.get();

      this.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,
          kMultiTagStdDevs);
    }
  }

  private void configPathPlanner() {
    double driveBaseRadius = 13;

    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose,
        this::seedFieldRelative,
        this::getChassisSpeeds,
        (chassisSpeeds) -> this.setControl(autoRequest.withSpeeds(chassisSpeeds)),
        new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0), new PIDConstants(4, 0, 0), kSpeedAt12VoltsMps,
            driveBaseRadius, new ReplanningConfig()),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);
  }

  private void configDriveSubsys() {
    var moduleZeroDrive = this.Modules[0].getDriveMotor().getConfigurator();
    var moduleOneDrive = this.Modules[1].getDriveMotor().getConfigurator();
    var moduleTwoDrive = this.Modules[2].getDriveMotor().getConfigurator();
    var moduleThreeDrive = this.Modules[3].getDriveMotor().getConfigurator();

    var moduleZeroSteer = this.Modules[0].getSteerMotor().getConfigurator();
    var moduleOneSteer = this.Modules[1].getSteerMotor().getConfigurator();
    var moduleTwoSteer = this.Modules[2].getSteerMotor().getConfigurator();
    var moduleThreeSteer = this.Modules[3].getSteerMotor().getConfigurator();

    moduleZeroDrive.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    moduleOneDrive.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    moduleTwoDrive.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    moduleThreeDrive.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));

    moduleZeroSteer.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    moduleOneSteer.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    moduleTwoSteer.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));
    moduleThreeSteer.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(30));

    photonPoseEstimator.setMultiTagFallbackStrategy((PoseStrategy.LOWEST_AMBIGUITY));
  }
}
