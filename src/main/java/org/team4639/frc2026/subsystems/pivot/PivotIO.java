package org.team4639.frc2026.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {

    @AutoLog
    public static class PivotIOInputs{
        public double volts;
        public double amps;
        public double celsius;
        public double mechanismRotations;
        public double mechanismRotationsPerSecond;
        public boolean connected;
    }

    default void setVoltage(double volts) {}
    default void setSetpointMechanismRotations(double mechanismRotations) {}
    default void setPositionMechanismRotations(double mechanismRotations) {}
    default void updateInputs(PivotIOInputs inputs) {}
    default void setBrakeMode(boolean isBrakeMode) {}
}
