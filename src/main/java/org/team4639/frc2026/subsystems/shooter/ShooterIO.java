/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    default void setVoltage(double appliedVolts) {}

    default void setRPM(double targetRPM) {}

    default void updateInputs(ShooterIOInputs inputs) {}

    default void applyNewGains() {}

    @AutoLog
    class ShooterIOInputs {
        public boolean[] connected = new boolean[4];

        public double[] voltage = new double[4];
        public double[] current = new double[4];
        public double[] temperature = new double[4];
        public double[] RPM = new double[4];
        public double[] rotations = new double[4];
    }
}