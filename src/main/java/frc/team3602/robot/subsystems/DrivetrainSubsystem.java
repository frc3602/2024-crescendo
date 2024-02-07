/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems;

import java.util.function.Supplier;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import static edu.wpi.first.units.Units.*;

import frc.team3602.robot.Vision;
import static frc.team3602.robot.Constants.DrivetrainConstants.*;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
  // Drivetrain
  private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds();
  public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(0.02 * kMaxSpeed.in(MetersPerSecond))
      .withRotationalDeadband(0.02 * kMaxAngularRate.in(MetersPerSecond));

  // Vision
  private final Vision vision = new Vision();

  private final PIDController turnController = new PIDController(0.015, 0.0, 0.001);

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, odometryUpdateFrequency, moduleConstants);
  }

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, moduleConstants);
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public Pose2d getPose() {
    return this.m_odometry.getEstimatedPosition();
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command choreoSwerveCommand(ChoreoTrajectory trajectory) {
    BooleanSupplier shouldMirror = () -> {
      Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

      return alliance.isPresent() && alliance.get() == Alliance.Red;
    };

    return Choreo.choreoSwerveCommand(
        trajectory,
        () -> getState().Pose,
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        (speeds) -> setControl(autonomousRequest.withSpeeds(speeds)),
        shouldMirror,
        this);
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

      this.setControl(autonomousRequest.withSpeeds(new ChassisSpeeds(0.0, 0.0, rotationSpeed)));
    });
  }

  private void updateOdometry() {
    var pose = vision.getEstimatedRobotPose();

    if (pose.isPresent()) {
      var estimatedRobotPose = pose.get();

      this.m_odometry.addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(),
          estimatedRobotPose.timestampSeconds);
    }
  }
}
