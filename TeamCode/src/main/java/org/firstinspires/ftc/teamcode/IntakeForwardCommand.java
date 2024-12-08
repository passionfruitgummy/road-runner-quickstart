package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

public class IntakeForwardCommand extends CommandBase {

    private final IntakeSubsystem intakeSubsystem;
    public IntakeForwardCommand(IntakeSubsystem intakeSubsystem){
        this.intakeSubsystem = intakeSubsystem;
    }

    @Override
    public void execute(){
        intakeSubsystem.forward();
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.stop();
    }
}
