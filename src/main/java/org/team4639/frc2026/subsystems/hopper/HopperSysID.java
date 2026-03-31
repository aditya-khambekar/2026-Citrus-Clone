/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.hopper;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public sealed class HopperSysID {
    @Getter
    private SysIdRoutine routine;

    public static final class HopperSysIDCTRE extends HopperSysID {
        public HopperSysIDCTRE(Hopper hopper){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> SignalLogger.writeString("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            hopper::setVoltage,
                            null, // SignalLogger handles logging
                            hopper
                    )
            );
        }
    }

    public static final class HopperSysIDWPI extends HopperSysID {
        public HopperSysIDWPI(Hopper hopper, HopperIO.HopperIOInputs inputs){
            super.routine = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.per(Second).of(0.5),
                            Volts.of(5),
                            null,
                            state -> Logger.recordOutput("SysIDTestState", state.toString())
                    ),
                    new SysIdRoutine.Mechanism(
                            hopper::setVoltage,
                            log -> {
                                log.motor("HopperFloor")
                                        .angularPosition(Rotations.of(inputs.mechanismRotations))
                                        .angularVelocity(Rotations.of(inputs.mechanismRotationsPerSecond).per(Second))
                                        .voltage(Volts.of(inputs.volts));
                            }, // SignalLogger handles logging
                            hopper
                    )
            );
        }
    }
}