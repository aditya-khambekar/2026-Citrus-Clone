package org.team4639.frc2026.subsystems.drum;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DrumConfigs{

    private TalonFXConfiguration createDrumConfig(boolean isLeft) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimit = 80;

        config.Audio.BeepOnConfig = false;

        config.Slot0.kP = 0;
        config.Slot0.kI = 0;
        config.Slot0.kD = 0;
        config.Slot0.kS = 0;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;

        if (isLeft) config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // positive -> launch balls
        else config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        return config;
    }

    public void configureStatusSignals(TalonFX talon){
        BaseStatusSignal.setUpdateFrequencyForAll(200, talon.getPosition(), talon.getVelocity());
        talon.optimizeBusUtilization(4);
    }
}
