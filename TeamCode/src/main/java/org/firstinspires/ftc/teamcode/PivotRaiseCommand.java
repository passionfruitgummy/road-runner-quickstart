package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class PivotRaiseCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    private final BooleanSupplier overrideLimits, slowMode;
    public PivotRaiseCommand(PivotSubsystem pivotSubsystem, BooleanSupplier overrideLimits, BooleanSupplier slowMode){
        this.pivotSubsystem = pivotSubsystem;
        this.slowMode = slowMode;
        this.overrideLimits = overrideLimits;
    }

    @Override
    public void execute(){
        pivotSubsystem.raise(overrideLimits.getAsBoolean(), slowMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        pivotSubsystem.stop();
    }
}
