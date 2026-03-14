/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopperfloor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class HopperFloorIOTalonFX implements HopperFloorIO {
    private final TalonFX hopperFloorMotor;

    private final VoltageOut voltageControl = new VoltageOut(0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);

    public HopperFloorIOTalonFX(PortConfiguration ports) {
        hopperFloorMotor = Phoenix6Factory.createDefaultTalon(ports.hopperFloor, false);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Slot0.kV = 0;
        config.Slot0.kA = 0;
        //config.Slot0.kP = 1;

        PhoenixUtil.tryUntilOk(5, () -> hopperFloorMotor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(HopperFloorIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                hopperFloorMotor.getMotorVoltage(),
                hopperFloorMotor.getStatorCurrent(),
                hopperFloorMotor.getVelocity(),
                hopperFloorMotor.getDeviceTemp()
        ).isOK();
        inputs.motorVoltage = hopperFloorMotor.getMotorVoltage().getValueAsDouble();
        inputs.motorCurrent = hopperFloorMotor.getStatorCurrent().getValueAsDouble();
        inputs.motorVelocity = hopperFloorMotor.getVelocity().getValueAsDouble();
        inputs.motorTemperature = hopperFloorMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorPosition = hopperFloorMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        hopperFloorMotor.setControl(voltageControl.withOutput(appliedVoltage));
    }

    @Override
    public void setRotorVelocityRPM(double targetVelocity) {
        hopperFloorMotor.setControl(velocityControl.withVelocity(targetVelocity * 12 / 24 / 60));
    }
}