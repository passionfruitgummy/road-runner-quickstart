package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class ArmExtendCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final BooleanSupplier overrideLimits, slowMode;
    public ArmExtendCommand(ArmSubsystem armSubsystem, BooleanSupplier overrideLimits, BooleanSupplier slowMode){
        this.armSubsystem = armSubsystem;
        this.overrideLimits = overrideLimits;
        this.slowMode = slowMode;
    }

    @Override
    public void execute(){
        armSubsystem.extend(overrideLimits.getAsBoolean(), slowMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.stop();
    }

}
