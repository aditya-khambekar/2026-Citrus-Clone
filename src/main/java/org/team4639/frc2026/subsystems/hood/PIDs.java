/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
  public static final LoggedTunableNumber hoodKp =
      new LoggedTunableNumber("HopperExtension/kP").initDefault(0);
  public static final LoggedTunableNumber hoodKi =
      new LoggedTunableNumber("HopperExtension/kI").initDefault(0);
  public static final LoggedTunableNumber hoodKd =
      new LoggedTunableNumber("HopperExtension/kD").initDefault(0);
  public static final LoggedTunableNumber hoodKs =
      new LoggedTunableNumber("HopperExtension/kS").initDefault(0);
  public static final LoggedTunableNumber hoodKv =
      new LoggedTunableNumber("HopperExtension/kV").initDefault(0);
  public static final LoggedTunableNumber hoodKa =
      new LoggedTunableNumber("HopperExtension/kA").initDefault(0);

  public static final LoggedTunableNumber hopperExtensionKpSim =
      new LoggedTunableNumber("HopperExtension/kPSim").initDefault(0.02);
  public static final LoggedTunableNumber hopperExtensionKiSim =
      new LoggedTunableNumber("HopperExtension/kISim").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKdSim =
      new LoggedTunableNumber("HopperExtension/kDSim").initDefault(0);
}
