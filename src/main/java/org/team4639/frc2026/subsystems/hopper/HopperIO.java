/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

    default void setVoltage(double appliedVolts) {}

    default void setSetpointMechanismRotationsPerSecond(double targetVelocity) {}

    default void updateInputs(HopperIOInputs inputs) {}

    @AutoLog
    class HopperIOInputs {
        public boolean connected;
        public double volts;
        public double amps;
        public double mechanismRotationsPerSecond;
        public double celsius;
        public double mechanismRotations;
    }
}
