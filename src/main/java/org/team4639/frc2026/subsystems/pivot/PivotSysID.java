/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.pivot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;
import org.team4639.frc2026.subsystems.pivot.PivotIO.PivotIOInputs;

import static edu.wpi.first.units.Units.*;

public sealed class PivotSysID {
    @Getter
    private SysIdRoutine routine;

    public static final class PivotSysIDCTRE extends PivotSysID {
        public PivotSysIDCTRE(Pivot pivot){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            pivot::setVoltage,
                            null, // SignalLogger handles logging
                            pivot
                    )
            );
        }
    }

   public static final class PivotSysIDWPI extends PivotSysID {
        public PivotSysIDWPI(Pivot pivot, PivotIOInputs inputs){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            pivot::setVoltage,
                            log -> log.motor("pivot")
                                        .angularPosition(Rotations.of(inputs.mechanismRotations))
                                        .angularVelocity(RotationsPerSecond.of(inputs.mechanismRotationsPerSecond))
                                        .voltage(Volts.of(inputs.volts)),
                            pivot
                    )
            );
        }
    }
}