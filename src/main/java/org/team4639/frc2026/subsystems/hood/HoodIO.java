/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    default void setSetpointRotorRotations(double rotorRotations) {
    }

    default void setVoltage(double volts) {
    }

    default void updateInputs(HoodIOInputs inputs) {
    }

    default void applyNewGains() {
    }

    default void setPositionRotorRotations(double rotorRotations) {
    }

    default void setBrakeMode(boolean brake) {
    }

    @AutoLog
    class HoodIOInputs {
        public boolean hoodMotorConnected = true;
        public double hoodVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodTemperature = 0.0;
        public double hoodPositionRotorRotations = 0.0;
        public double hoodVelocityRotorRotations = 0.0;
    }
}
