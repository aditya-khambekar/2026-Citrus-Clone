/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {

    default void setVoltage(double appliedVolts) {}

    default void setRotorVelocityRPM(double targetVelocity) {}

    default void updateInputs(FeederIOInputs inputs) {}

    default void applyNewGains() {}

    @AutoLog
    class FeederIOInputs {
        public boolean motorConnected = true;
        public double motorVoltage;
        public double motorCurrent;
        public double motorVelocity;
        public double motorTemperature;
        public double motorPosition;
    }
}