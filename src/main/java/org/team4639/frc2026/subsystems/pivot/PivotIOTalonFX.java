package org.team4639.frc2026.subsystems.pivot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import org.team4639.frc2026.util.PortConfiguration;
import org.team4639.lib.util.Phoenix6Factory;
import org.team4639.lib.util.PhoenixUtil;

public class PivotIOTalonFX implements PivotIO{
    private final TalonFX pivotMotor;
    // requests
    private final VoltageOut voltageOut;
    private final PositionVoltage positionVoltage;
    // StatusSignals
    private final StatusSignal<Angle> mechanismRotations;
    private final StatusSignal<AngularVelocity> mechanismRotationsPerSecond;
    private final StatusSignal<Voltage> volts;
    private final StatusSignal<Current> amps;
    private final StatusSignal<Temperature> celsius;

    public PivotIOTalonFX(PortConfiguration portConfiguration) {
        this.pivotMotor = Phoenix6Factory.createDefaultTalon(portConfiguration.intakePivot);
        PhoenixUtil.tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(PivotConfigs.pivotMotorConfig));
        this.voltageOut = new VoltageOut(0);
        this.positionVoltage = new PositionVoltage(0);

        mechanismRotations = pivotMotor.getPosition();
        mechanismRotationsPerSecond = pivotMotor.getVelocity();
        volts = pivotMotor.getMotorVoltage();
        amps = pivotMotor.getTorqueCurrent();
        celsius = pivotMotor.getDeviceTemp();
    }

    @Override
    public void setVoltage(double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void setPosition(double mechanismRotations) {
        pivotMotor.setControl(positionVoltage.withPosition(mechanismRotations));
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                mechanismRotations,
                mechanismRotationsPerSecond,
                volts,
                amps,
                celsius
        ).isOK();

        inputs.mechanismRotations = mechanismRotations.getValueAsDouble();
        inputs.amps = amps.getValueAsDouble();
        inputs.volts = volts.getValueAsDouble();
        inputs.mechanismRotationsPerSecond = mechanismRotationsPerSecond.getValueAsDouble();
        inputs.celsius = celsius.getValueAsDouble();
    }
}
