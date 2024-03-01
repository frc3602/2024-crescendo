/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonFactory {
  public static final Command doNothingCmd = Commands.print("This literally does nothing.");

  public static Command testCmd() {
    return new PathPlannerAuto("test");
  }

  public static Command bruhCmd() {
    return new PathPlannerAuto("bruh");
  }

  public static Command bruh2Cmd() {
    return new PathPlannerAuto("bruh2");
  }

  public static Command straitbackCmd() {
    return new PathPlannerAuto("straitback");
  }

  public static Command rightstraitbackCmd() {
    return new PathPlannerAuto("rightstraitback");
  }
}
