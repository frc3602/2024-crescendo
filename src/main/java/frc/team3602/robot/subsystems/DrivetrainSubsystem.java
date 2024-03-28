/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.Supplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.team3602.robot.Vision;

import monologue.Logged;
import monologue.Annotations.Log;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;

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
  public final Vision vision = new Vision();
  private CommandXboxController xboxController;

  private final PIDController noteTurnController = new PIDController(0.04, 0.0, 0.001);
  private final PIDController turnController = new PIDController(0.08, 0.0, 0.001);
  private final PIDController strafeController = new PIDController(0.5, 0.0, 0.0);
  private final PIDController speakerTurnController = new PIDController(0.12, 0.0, 0.001);

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      CommandXboxController xboxController, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, odometryUpdateFrequency, moduleConstants);
    this.xboxController = xboxController;
    configPathPlanner();
    configDriveSubsys();
  }

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, CommandXboxController xboxController,
      SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, moduleConstants);
    this.xboxController = xboxController;

    configPathPlanner();
    configDriveSubsys();
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

  public Command turnTowardTarget() {
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

  public Command alignWithTarget() {
    return run(() -> {
      PhotonPipelineResult result = vision.getLatestResult();
      double strafeDistance = strafeController.calculate(
          (result.hasTargets() ? vision.kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getX()
              : 0.0) - getPose().getX(),
          0.0);

      this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(0.0, strafeDistance, 0.0)));
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

  public Command turnTowardNote() {
    return run(() -> {
      double rotationSpeed;

      var result = vision.getNoteResult();

      if (result.hasTargets()) {
        rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
      } else {
        rotationSpeed = 0.0;
      }

      this.setControl(autoRequest
          .withSpeeds(new ChassisSpeeds(xboxController.getLeftY(), xboxController.getLeftX(), rotationSpeed)));
    });
  }

  public Command turnTowardSpeaker() {
    return run(() -> {
      double rotationSpeed = 0.0;

      var result = vision.getLatestResult();

      if (result.hasTargets()) {
        if (result.getBestTarget().getFiducialId() == 7 || result.getBestTarget().getFiducialId() == 4) {
          rotationSpeed = speakerTurnController.calculate(result.getBestTarget().getYaw(), 0.0);
        }
      } else {
        rotationSpeed = 0.0;
      }

      this.setControl(autoRequest
          .withSpeeds(new ChassisSpeeds(xboxController.getLeftY(), xboxController.getLeftX(), rotationSpeed)));
    });
  }

  @Log
  public boolean atHeading() {
    var target = 0;
    var tolerance = 2;
    var heading = 0.0;
    var result = vision.getNoteResult();

    if (result.hasTargets()) {
      heading = (result.getBestTarget().getYaw());
    } else {
      heading = 100.0;
    }

    return ((MathUtil.isNear(target, heading, tolerance)));
  }

  @Log
  double xSpeed = 0;

  public Command driveTowardNote() {
    return run(() -> {
      double rotationSpeed = 0.0;

      var result = vision.getNoteResult();

      if (result.hasTargets()) {
        rotationSpeed = noteTurnController.calculate(result.getBestTarget().getYaw(), 0.0);
        if (atHeading()) {
          xSpeed = 0.3;
        } else {
          xSpeed = 0;
        }
      } else {
        rotationSpeed = 0.0;
        xSpeed = 0.0;
      }

      this.setControl(autoRequest.withSpeeds(new ChassisSpeeds(xSpeed, 0, rotationSpeed)));
    });
  }

  public Command turnTowardAmp() {
    return run(() -> {
      double rotationSpeed = 0.0;

      var result = vision.getLatestResult();

      if (result.hasTargets()) {
        if (result.getBestTarget().getFiducialId() == 6 || result.getBestTarget().getFiducialId() == 5) {
          rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
        }
      } else {
        rotationSpeed = 0.0;
      }

      this.setControl(autoRequest
          .withSpeeds(new ChassisSpeeds(xboxController.getLeftY(), xboxController.getLeftX(), rotationSpeed)));
    });
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
  }
}
