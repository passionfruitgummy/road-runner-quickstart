package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class PivotLowerCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    private final BooleanSupplier overrideLimits, slowMode;
    public PivotLowerCommand(PivotSubsystem pivotSubsystem, BooleanSupplier overrideLimits, BooleanSupplier slowMode){
        this.pivotSubsystem = pivotSubsystem;
        this.overrideLimits = overrideLimits;
        this.slowMode = slowMode;
    }

    @Override
    public void execute(){
        pivotSubsystem.lower(overrideLimits.getAsBoolean(), slowMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        pivotSubsystem.stop();
    }
}
