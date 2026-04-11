/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.constants.ports;

import org.team4639.frc2026.util.CanDeviceId;
import org.team4639.frc2026.util.PortConfiguration;

import com.ctre.phoenix6.CANBus;

public class WaterBottle {
  public static final PortConfiguration portConfiguration = new PortConfiguration();

  static {
    var maincanivore = new CANBus("MainCANivore");
    var rio = new CANBus("rio");
    portConfiguration.leftIntake = new CanDeviceId(20, rio);
    portConfiguration.rightIntake = new CanDeviceId(21, rio);
    portConfiguration.intakePivot = new CanDeviceId(22, maincanivore);
    portConfiguration.hopper = new CanDeviceId(23, maincanivore);
    portConfiguration.verticalExtension = new CanDeviceId(24, maincanivore);
    portConfiguration.feeder = new CanDeviceId(25, maincanivore);
    portConfiguration.hood = new CanDeviceId(26, maincanivore);
    portConfiguration.leftTopDrum = new CanDeviceId(27, maincanivore);
    portConfiguration.rightTopDrum = new CanDeviceId(28, maincanivore);
    portConfiguration.leftBottomDrum = new CanDeviceId(29, maincanivore);
    portConfiguration.rightBottomDrum = new CanDeviceId(30, maincanivore);
  }
}
