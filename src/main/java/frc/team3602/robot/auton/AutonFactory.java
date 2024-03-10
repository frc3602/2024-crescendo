/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;

public class AutonFactory {
  public static Command oneNoteMiddleCmd() {
    return new PathPlannerAuto("oneNoteMiddle");
  }

  public static Command backTwoNoteMiddleCmd() {
    return new PathPlannerAuto("backTwoNoteMiddle");
  }

  public static Command twoNoteMiddleCmd() {
    return new PathPlannerAuto("twoNoteMiddle");
  }

  public static Command twoNoteLeftCmd() {
    return new PathPlannerAuto("twoNoteLeft");
  }

  public static Command twoNoteRightCmd() {
    return new PathPlannerAuto("twoNoteRight");
  }
}
