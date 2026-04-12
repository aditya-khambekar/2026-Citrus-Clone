/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final boolean tuningMode = true;
  public static final boolean disableHAL = false;

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class RobotConstants {
    public static final double ROBOT_MASS_KG = 50;
    public static final double ROBOT_MOI = 3.97;
    public static final double WHEEL_COF = 1.5;

    public static final Transform2d ORIGIN_TO_DRUM = new Transform2d(0.254, 0, Rotation2d.k180deg);

    public static final double THEORETICAL_X60_KV = 12.0 * 60 / 6000;
    public static final double THEORETICAL_X44_KV = 12.0 * 60 / 7758;
  }
}
