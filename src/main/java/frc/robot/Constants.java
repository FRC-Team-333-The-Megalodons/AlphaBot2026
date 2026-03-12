// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final double DEFAULT_EPSILON = 0.001;

  public static boolean fuzzyEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  public static boolean fuzzyEqualsZero(double x, double epsilon) {
    return fuzzyEquals(x, 0.0, epsilon);
  }

  public static boolean fuzzyEqualsZero(double x) {
    return fuzzyEqualsZero(x, DEFAULT_EPSILON);
  }

  public static boolean allFuzzyEqualsZero(double... vals) {
    boolean result = true;
    for (double val : vals) {
      result = result && fuzzyEqualsZero(val);
    }
    return result;
  }
}
