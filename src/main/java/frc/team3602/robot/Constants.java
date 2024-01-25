/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.*;

public final class Constants {
  public final class OperatorInterfaceConstants {
    public final static int kXboxControllerPort = 0;
  }

  public final class ShooterConstants {
    public static final int kTopShooterMotorId = 2;
    public static final int kTopShooterMotorCurrentLimit = 27;

    public static final int kBottomShooterMotorId = 3;
    public static final int kBottomShooterMotorCurrentLimit = 27;
  }

  public final class IntakeConstants {
    public static final int kIntakeMotorId = 5;
    public static final int kIntakeMotorCurrentLimit = 29;

    public static final int kColorSensorId = 0;

    public static final double kIntakeSpeedForward = 500;
    public static final double kIntakeSpeedReverse = -500;

    public static final double kIntakeConversionFactor = (Math.PI * Units.inchesToMeters(1.0)) / (3.0 / 1.0)
        * (1 / 60.0);
  }

  public final class PivotConstants {
    public static final int kPivotMotorId = 4;
    public static final int kPivotMotorCurrentLimit = 30;

  }

  public final class VisionConstants {
    public static final String kPhotonCameraName = "photonvision";

    public static final Transform3d kRobotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0.0, 0.0, 0.0));

    public static final Measure<Distance> kCameraHeight = Feet.of(1.4375);
    public static final Measure<Angle> kCameraPitch = Degrees.of(0.0);

    public static final Measure<Distance> kGoalRange = Feet.of(9.0);
  }
}
