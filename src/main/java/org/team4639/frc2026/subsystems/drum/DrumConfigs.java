/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.Constants;

public class DrumConfigs{
    public static TalonFXConfiguration drumConfig = createDrumConfig();

    private static TalonFXConfiguration createDrumConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimit = 80;

        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = 0.38628;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0.060868;
        config.Slot0.kV = 0.25613;
        config.Slot0.kA = 0.036216;

        config.Feedback.SensorToMechanismRatio = 1.0 / DrumConstants.MOTOR_TO_DRUM_REDUCTION;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // positive -> launch balls, assume leader is a left motor

        return config;
    }

    public static void configureStatusSignals(TalonFX talon){
        BaseStatusSignal.setUpdateFrequencyForAll(250, talon.getPosition(), talon.getVelocity(), talon.getAcceleration(), talon.getMotorVoltage(), talon.getTorqueCurrent());
        talon.optimizeBusUtilization();
    }
}
