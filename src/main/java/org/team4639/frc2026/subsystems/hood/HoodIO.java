/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
    default void setSetpointMechanismRotations(double mechanismRotations) {
    }

    default void setVoltage(double volts) {
    }

    default void setSetpointMechanismRotations(double mechanismRotations, double mechanismRotationsPerSecond) {}

    default void updateInputs(HoodIOInputs inputs) {
    }

    default void setPositionMechanismRotations(double mechanismRotations) {
    }

    @AutoLog
    class HoodIOInputs {
        public boolean connected = true;
        public double volts = 0.0;
        public double amps = 0.0;
        public double celsius = 0.0;
        public double mechanismRotations = 0.0;
        public double mechanismRotationsPerSecond = 0.0;
    }
}
