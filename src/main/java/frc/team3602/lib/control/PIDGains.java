/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.lib.control;

public final class PIDGains {
  public double kP;
  public double kI;
  public double kD;
  public double kF;

  public PIDGains() {
  }

  public PIDGains withKp(double kP) {
    this.kP = kP;
    return this;
  }

  public PIDGains withKi(double kI) {
    this.kI = kI;
    return this;
  }

  public PIDGains withKd(double kD) {
    this.kD = kD;
    return this;
  }

  public PIDGains withKf(double kF) {
    this.kF = kF;
    return this;
  }
}
