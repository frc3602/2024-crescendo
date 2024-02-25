/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.team3602.robot.Vision;

import monologue.Logged;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem, Logged {
  // Drivetrain
  private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
  public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(0.02 * kMaxSpeed)
      .withRotationalDeadband(0.02 * kMaxAngularRate);

  // Vision
  private final Vision vision = new Vision();

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

  @Override
  public void periodic() {
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
      double rotationSpeed;

      var result = vision.getLatestResult();

      if (result.hasTargets()) {
        rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
      } else {
        rotationSpeed = 0.0;
      }

      this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, rotationSpeed)));
    });
  }

  private void updateOdometry() {
    var pose = vision.getEstimatedRobotPose();

    if (pose.isPresent()) {
      var estimatedRobotPose = pose.get();

      this.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds,
          kMultiTagStdDevs);
    }
  }

  private void configPathPlanner() {
    double driveBaseRadius = 0;

    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose,
        this::seedFieldRelative,
        this::getChassisSpeeds,
        (chassisSpeeds) -> this.setControl(autoRequest.withSpeeds(chassisSpeeds)),
        new HolonomicPathFollowerConfig(new PIDConstants(1, 0, 0), new PIDConstants(1, 0, 0), kSpeedAt12VoltsMps,
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
  }
}
