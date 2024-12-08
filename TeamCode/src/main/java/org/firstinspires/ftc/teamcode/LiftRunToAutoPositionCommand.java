package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class LiftRunToAutoPositionCommand extends CommandBase {

    private final LiftSubsystem liftSubsystem;

    public LiftRunToAutoPositionCommand(LiftSubsystem liftSubsystem){
        this.liftSubsystem = liftSubsystem;

    }

    @Override
    public void initialize(){
        liftSubsystem.autoPosition();
    }

    @Override
    public void end(boolean interrupted){
        liftSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        return liftSubsystem.isFinished() &&
                Math.abs(liftSubsystem.getLeftPos() - (liftSubsystem.getLeftStartPos() + LiftSubsystem.AUTO_POS)) <= 50 &&
                Math.abs(liftSubsystem.getRightPos() - (liftSubsystem.getRightStartPos() + LiftSubsystem.AUTO_POS)) <= 50;
    }
}
