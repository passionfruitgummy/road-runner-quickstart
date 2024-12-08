package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveStopCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final Telemetry telemetry;

    public DriveStopCommand(DriveSubsystem driveSubsystem, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.stop();
    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("drive stopped");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
