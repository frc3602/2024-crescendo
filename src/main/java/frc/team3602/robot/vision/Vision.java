/*
 * Copyright (C) 2023, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.team3602.robot.Constants.VisionConstants.*;

public class Vision {
  private final PhotonCamera photonCamera = new PhotonCamera(PHOTON_CAMERA_NAME);

  public Vision() {
  }

  public double getTargetRange(double targetHeightMeters) {
    double targetRange;
    var result = photonCamera.getLatestResult();

    if (result.hasTargets()) {
      targetRange = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, targetHeightMeters,
          CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    } else {
      targetRange = 0.0;
      DriverStation.reportError("PhotonCamera: " + PHOTON_CAMERA_NAME + "has no Targets", false);
    }

    return targetRange;
  }
}
