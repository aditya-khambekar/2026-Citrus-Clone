/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drum;

import org.littletonrobotics.junction.AutoLog;

public interface DrumIO {

    default void setVoltage(double appliedVolts) {}

    default void setSetpointMechanismRPM(double mechanismRPM) {}
    default void setSetpointMechanismRPM(double mechanismRPM, double mechanismRPMPerSecond) {}

    default void updateInputs(ShooterIOInputs inputs) {}

    @AutoLog
    class ShooterIOInputs {
        public boolean[] connected = new boolean[4];
        public double[] volts = new double[4];
        public double[] amps = new double[4];
        public double[] celsius = new double[4];
        public double[] mechanismRPMPerSecond = new double[4];
        public double[] mechanismRPM = new double[4];
        public double[] mechanismRotations = new double[4];
    }
}
