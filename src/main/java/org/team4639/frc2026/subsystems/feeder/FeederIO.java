/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    default void setVoltage(double appliedVolts) {}

    default void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond) {}

    default void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond, double mechanismRotationsPerSecondPerSecond) {}

    default void updateInputs(FeederIOInputs inputs) {}

    @AutoLog
    class FeederIOInputs {
        public boolean connected = true;
        public double volts;
        public double amps;
        public double mechanismRotationsPerSecond;
        public double celsius;
        public double mechanismRotations;
    }
}