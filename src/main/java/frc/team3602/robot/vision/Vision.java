/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.vision;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.team3602.robot.Constants.VisionConstants.*;

public class Vision {
  private final AprilTagFieldLayout kFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private final PhotonCamera photonCamera = new PhotonCamera(kPhotonCameraName);
  private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
      PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, kRobotToCamera);

  public static record VisionMeasurement(EstimatedRobotPose estimate, Matrix<N3, N1> confidence) {}

  public Vision() {
    configVision();
  }

  public PhotonPipelineResult getLatestResult() {
    return photonCamera.getLatestResult();
  }

  public double getTargetHeight() {
    double targetHeight;
    var result = getLatestResult();

    if (result.hasTargets()) {
      targetHeight = kFieldLayout.getTagPose(result.getBestTarget().getFiducialId()).get().getZ();
    } else {
      targetHeight = 0.0;
      DriverStation.reportError("PhotonCamera: " + kPhotonCameraName + "has no Targets", false);
    }

    return targetHeight;
  }

  public double getTargetRange(double targetHeightMeters) {
    double targetRange;
    var result = getLatestResult();

    if (result.hasTargets()) {
      targetRange = PhotonUtils.calculateDistanceToTargetMeters(kCameraHeightMeters, targetHeightMeters,
          kCameraPitchRadians, Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      targetRange = 0.0;
      DriverStation.reportError("PhotonCamera: " + kPhotonCameraName + "has no Targets", false);
    }

    return targetRange;
  }

  private void findVisionMeasurements() {
    
  }

  private void configVision() {
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }
}
