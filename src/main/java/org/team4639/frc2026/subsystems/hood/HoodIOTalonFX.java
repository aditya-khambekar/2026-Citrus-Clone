/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team4639.frc2026.RobotState;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Commands2;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor;

    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final VoltageOut voltageOut = new VoltageOut(0);

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    private final PIDController hoodController = new PIDController(200, 0, 4);

    private boolean usingWPILibPID = true;

    public HoodIOTalonFX(PortConfiguration ports) {
        hoodMotor = Phoenix6Factory.createDefaultTalon(ports.hood, true);

        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(HoodConfigs.hoodConfig));

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        motorVoltage = hoodMotor.getMotorVoltage();
        motorCurrent = hoodMotor.getStatorCurrent();

        RobotState.disabled.onTrue(Commands2.action(() -> PhoenixUtil.tryUntilOk(5, () -> hoodMotor.setNeutralMode(NeutralModeValue.Coast))));
        RobotState.disabled.onFalse(Commands2.action(() -> PhoenixUtil.tryUntilOk(5, () -> hoodMotor.setNeutralMode(NeutralModeValue.Brake))));

        voltageOut.IgnoreSoftwareLimits = true;
        positionVoltage.IgnoreSoftwareLimits = false;

        if (usingWPILibPID) SmartDashboard.putData("Hood PID", hoodController);
    }

    @Override
    public void setSetpointMechanismRotations(double mechanismRotations) {
        setSetpointMechanismRotations(mechanismRotations, 0);
    }

    @Override
    public void setSetpointMechanismRotations(double mechanismRotations, double mechanismRotationsPerSecond) {
        System.out.println("Hood Setpoint "+mechanismRotations);
         positionVoltage.Position = mechanismRotations;
         positionVoltage.Velocity = mechanismRotationsPerSecond;
        var output = hoodController.calculate(hoodPosition.getValueAsDouble(), mechanismRotations);
        SmartDashboard.putNumber("Hood Controller Output", output);
        if (usingWPILibPID) hoodMotor.setVoltage(output);
        else hoodMotor.setControl(positionVoltage);
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.connected =
                BaseStatusSignal.refreshAll(
                                motorVoltage,
                                motorCurrent,
                                hoodMotor.getDeviceTemp(),
                                hoodVelocity,
                                hoodPosition)
                        .isOK();
        inputs.volts = motorVoltage.getValueAsDouble();
        inputs.amps = motorCurrent.getValueAsDouble();
        inputs.celsius = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.mechanismRotations = hoodPosition.getValueAsDouble();
        inputs.mechanismRotationsPerSecond = hoodVelocity.getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        System.out.println("Hood Volts "+volts);
        voltageOut.Output = volts;
        hoodMotor.setControl(voltageOut);
    }

    @Override
    public void setPositionMechanismRotations(double mechanismRotations) {
        hoodMotor.setPosition(mechanismRotations);
    }
}
