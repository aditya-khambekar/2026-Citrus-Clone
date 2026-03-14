/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class FeederIOTalonFX implements FeederIO {
    private final TalonFX feederMotor;

    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public FeederIOTalonFX(PortConfiguration ports) {
        feederMotor = Phoenix6Factory.createDefaultTalon(ports.feeder, false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        //config.Slot0.kP = 1;

        PhoenixUtil.tryUntilOk(5, () -> feederMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                feederMotor.getMotorVoltage(),
                feederMotor.getStatorCurrent(),
                feederMotor.getVelocity(),
                feederMotor.getDeviceTemp()
        ).isOK();
        inputs.motorVoltage = feederMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrent = feederMotor.getStatorCurrent().getValueAsDouble();
        inputs.motorVelocity = feederMotor.getVelocity().getValueAsDouble();
        inputs.motorTemperature = feederMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorPosition = feederMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        feederMotor.setControl(voltageControl.withOutput(appliedVoltage));
    }

    @Override
    public void setRotorVelocityRPM(double targetVelocity) {
        feederMotor.setControl(velocityControl.withVelocity(targetVelocity * 12 / 24 / 60));
    }
}