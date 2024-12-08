package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeRunCommand extends CommandBase {

    public static enum Direction{
        In,
        Out
    };
    private final IntakeSubsystem intakeSubsystem;
    private final Direction direction;
    public IntakeRunCommand(IntakeSubsystem intakeSubsystem, Direction direction){
        this.intakeSubsystem = intakeSubsystem;
        this.direction = direction;
    }

    @Override
    public void initialize(){
        intakeSubsystem.run((direction == Direction.In) ? 1 : -1);
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }

    // is finished always will return false; uses timer to interrupt and end command

}
