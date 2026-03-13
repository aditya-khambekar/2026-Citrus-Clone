/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopperextension;

import org.team4639.lib.util.LoggedTunableNumber;

public class PIDs {
  public static final LoggedTunableNumber hopperExtensionKp =
      new LoggedTunableNumber("HopperExtension/kP").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKi =
      new LoggedTunableNumber("HopperExtension/kI").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKd =
      new LoggedTunableNumber("HopperExtension/kD").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKs =
      new LoggedTunableNumber("HopperExtension/kS").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKv =
      new LoggedTunableNumber("HopperExtension/kV").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKa =
      new LoggedTunableNumber("HopperExtension/kA").initDefault(0);

  public static final LoggedTunableNumber hopperExtensionKpSim =
      new LoggedTunableNumber("HopperExtension/kPSim").initDefault(0.02);
  public static final LoggedTunableNumber hopperExtensionKiSim =
      new LoggedTunableNumber("HopperExtension/kISim").initDefault(0);
  public static final LoggedTunableNumber hopperExtensionKdSim =
      new LoggedTunableNumber("HopperExtension/kDSim").initDefault(0);
}
