package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class PivotRunToPositionCommand extends CommandBase {

    private final PivotSubsystem pivotSubsystem;
    private final int position;
    private final double power;

    public PivotRunToPositionCommand(PivotSubsystem pivotSubsystem, int position, double power){
        this.pivotSubsystem = pivotSubsystem;
        this.position = position;
        this.power = power;
    }

    @Override
    public void initialize(){
        pivotSubsystem.runToPosition(position, power);
    }

    @Override
    public void end(boolean interrupted) { pivotSubsystem.stop(); }

    @Override
    public boolean isFinished(){
        return pivotSubsystem.isFinished();
    }
}
