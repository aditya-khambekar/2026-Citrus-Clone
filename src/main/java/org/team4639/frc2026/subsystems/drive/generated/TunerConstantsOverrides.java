/* Copyright (c) 2025-2026 FRC 4639. */

package org.team4639.frc2026.subsystems.drive.generated;

import com.ctre.phoenix6.configs.Slot0Configs;

import lombok.Builder;

@Builder
public class TunerConstantsOverrides {
    public static TunerConstantsOverrides[] overrides
        = new TunerConstantsOverrides[]{
            TunerConstantsOverrides.builder().build(), // Front Left
            TunerConstantsOverrides.builder().build(), // Front Right
            TunerConstantsOverrides.builder().build(), // Back Left
            TunerConstantsOverrides.builder().build() // Back Right
        };

    public Slot0Configs driveSlot0Configs;
    public Slot0Configs steerSlot0Configs;
}
