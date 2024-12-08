package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeBackwardCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public IntakeBackwardCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.backward();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
