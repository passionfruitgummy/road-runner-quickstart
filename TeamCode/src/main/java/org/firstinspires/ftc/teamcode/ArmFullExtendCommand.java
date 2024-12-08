package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.BooleanSupplier;

public class ArmFullExtendCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final BooleanSupplier slowMode;
    public ArmFullExtendCommand(ArmSubsystem armSubsystem, BooleanSupplier slowMode){
        this.armSubsystem = armSubsystem;
        this.slowMode = slowMode;
    }

    @Override
    public void initialize(){
        armSubsystem.fullExtend(slowMode.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted){
        armSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return armSubsystem.isFinished();
    }

}
