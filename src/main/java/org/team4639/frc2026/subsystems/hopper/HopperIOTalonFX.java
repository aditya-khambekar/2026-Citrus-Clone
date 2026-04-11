/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class HopperIOTalonFX implements HopperIO {
    private final TalonFX hopperMotor;

    private final VoltageOut voltageOut;
    private final VelocityVoltage velocityVoltage;

    public HopperIOTalonFX(PortConfiguration ports) {
        hopperMotor = Phoenix6Factory.createDefaultTalon(ports.hopper, true);

        PhoenixUtil.tryUntilOk(5, () -> hopperMotor.getConfigurator().apply(HopperConfigs.hopperConfig));

        voltageOut = new VoltageOut(0);
        velocityVoltage = new VelocityVoltage(0);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                hopperMotor.getMotorVoltage(),
                hopperMotor.getStatorCurrent(),
                hopperMotor.getVelocity(),
                hopperMotor.getDeviceTemp()
        ).isOK();
        inputs.volts = hopperMotor.getMotorVoltage().getValueAsDouble();
        inputs.amps = hopperMotor.getStatorCurrent().getValueAsDouble();
        inputs.mechanismRotationsPerSecond = hopperMotor.getVelocity().getValueAsDouble();
        inputs.celsius = hopperMotor.getDeviceTemp().getValueAsDouble();
        inputs.mechanismRotations = hopperMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setVoltage(double appliedVoltage)  {
        hopperMotor.setControl(voltageOut.withOutput(appliedVoltage));
    }

    @Override
    public void setSetpointMechanismRotationsPerSecond(double mechanismRotationsPerSecond) {
        System.out.println("Hopper Setpoint "+mechanismRotationsPerSecond);
        hopperMotor.setControl(velocityVoltage.withVelocity(mechanismRotationsPerSecond));
    }
}