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

    public void setVoltage(double volts);
    public void setPosition(double mechanismRotations);
    public void updateInputs(PivotIOInputs inputs);
}
