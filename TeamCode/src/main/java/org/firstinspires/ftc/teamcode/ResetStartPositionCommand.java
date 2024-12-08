package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class ResetStartPositionCommand extends CommandBase {

    public enum Type{
        Driver,
        Operator
    };
    private final ArmSubsystem armSubsystem;
    private final PivotSubsystem pivotSubsystem;
    private final LiftSubsystem liftSubsystem;
    private final Type type;

    public ResetStartPositionCommand(ArmSubsystem armSubsystem, PivotSubsystem pivotSubsystem, LiftSubsystem liftSubsystem, Type type){
        this.armSubsystem = armSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.liftSubsystem = liftSubsystem;
        this.type = type;
    }

    @Override
    public void initialize(){ // each gamepad resets their own respective subsystems
        if(type == Type.Driver){
            liftSubsystem.resetStartPositions();
        }
        else {
            armSubsystem.resetStartPosition();
            pivotSubsystem.resetStartPosition();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}
