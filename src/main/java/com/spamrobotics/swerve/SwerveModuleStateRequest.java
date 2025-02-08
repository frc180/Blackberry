package com.spamrobotics.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleStateRequest implements SwerveRequest {

    private SwerveModuleState[] moduleStates = null;
    private ModuleRequest request = new ModuleRequest();

    public SwerveModuleStateRequest withModuleStates(SwerveModuleState... moduleStates) {
        this.moduleStates = moduleStates;
        return this;
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        for (int i = 0; i < modulesToApply.length; i++) {
            modulesToApply[i].apply(request.withState(moduleStates[i]));
        }
        return StatusCode.OK;
    }
}
