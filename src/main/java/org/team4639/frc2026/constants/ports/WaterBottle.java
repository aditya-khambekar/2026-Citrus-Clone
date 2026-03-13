/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.ports;

import org.team4639.frc2026.util.CanDeviceId;
import org.team4639.frc2026.util.PortConfiguration;

public class WaterBottle {
  public static final PortConfiguration portConfiguration = new PortConfiguration();

  static {
    portConfiguration.leftIntake = new CanDeviceId(20);
    portConfiguration.rightIntake = new CanDeviceId(21);
    portConfiguration.intakePivot = new CanDeviceId(22);
    portConfiguration.hopper = new CanDeviceId(23);
    portConfiguration.verticalExtension = new CanDeviceId(24);
    portConfiguration.feeder = new CanDeviceId(25);
    portConfiguration.hood = new CanDeviceId(26);
    portConfiguration.leftTopDrum = new CanDeviceId(27);
    portConfiguration.rightTopDrum = new CanDeviceId(28);
    portConfiguration.leftBottomDrum = new CanDeviceId(29);
    portConfiguration.rightBottomDrum = new CanDeviceId(30);
  }
}
