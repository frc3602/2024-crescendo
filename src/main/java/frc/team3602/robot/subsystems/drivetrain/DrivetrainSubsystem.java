/*
 * Copyright (C) 2023, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.team3602.robot.Constants.DrivetrainConstants.*;
import static frc.team3602.robot.Constants.VisionConstants.*;

public class DrivetrainSubsystem extends SwerveDrivetrain implements Subsystem {
  private final SwerveRequest.ApplyChassisSpeeds autonomousRequest = new SwerveRequest.ApplyChassisSpeeds();
  public final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withDeadband(0.02 * MAX_SPEED)
      .withRotationalDeadband(0.02 * MAX_ANGULAR_RATE);

  private final ChoreoTrajectory trajectory = Choreo.getTrajectory("traj");

  // Vision
  private double targetRange;

  private final PhotonCamera camera = new PhotonCamera("photonvision");
  private final PIDController forwardController = new PIDController(0.1, 0.0, 0.05);
  private final PIDController turnController = new PIDController(0.1, 0.0, 0.05);

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
      SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, odometryUpdateFrequency, moduleConstants);
  }

  public DrivetrainSubsystem(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants... moduleConstants) {
    super(drivetrainConstants, moduleConstants);

    this.m_pigeon2.setYaw(0.0); // REMOVE DURING COMP
  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();

    if (result.hasTargets()) {
      targetRange = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS,
          CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    }

    SmartDashboard.putNumber("Distance to Target", Units.metersToFeet(targetRange));
  }

  @Override
  public void simulationPeriodic() {
    // This is an estimate.
    updateSimState(0.02, 12);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  public Command swerveControllerCommand() {
    return Choreo.choreoSwerveCommand(
        trajectory,
        () -> this.getState().Pose,
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        (speeds) -> this.setControl(autonomousRequest.withSpeeds(speeds)),
        () -> true,
        this);
  }

  public Command alignWithTarget() {
    return run(() -> {
      double forwardSpeed;
      double rotationSpeed;

      var result = camera.getLatestResult();

      if (result.hasTargets()) {
        forwardSpeed = -forwardController.calculate(targetRange, GOAL_RANGE_METERS);
        rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0.0);
      } else {
        forwardSpeed = 0.0;
        rotationSpeed = 0.0;
      }

      this.setControl(autonomousRequest
          .withSpeeds(new ChassisSpeeds(forwardSpeed, 0.0, rotationSpeed)));
    });
  }
}
