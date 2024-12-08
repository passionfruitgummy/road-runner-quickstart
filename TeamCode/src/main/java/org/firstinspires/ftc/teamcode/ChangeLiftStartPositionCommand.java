package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ChangeLiftStartPositionCommand extends CommandBase {
    private final LiftSubsystem liftSubsystem;
    public ChangeLiftStartPositionCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;
    }

    @Override
    public void initialize(){
        liftSubsystem.autoOffset();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
