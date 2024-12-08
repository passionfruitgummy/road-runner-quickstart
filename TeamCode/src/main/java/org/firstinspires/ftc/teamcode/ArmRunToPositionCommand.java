package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmRunToPositionCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final Telemetry telemetry;
    private final int position;
    private final double power;

    public ArmRunToPositionCommand(ArmSubsystem armSubsystem, Telemetry telemetry, int position, double power){
        this.armSubsystem = armSubsystem;
        this.telemetry = telemetry;
        this.position = position;
        this.power = power;
    }

    @Override
    public void initialize(){
        armSubsystem.runToPosition(position, power);
    }

    @Override
    public void execute(){
        telemetry.addData("Arm position", armSubsystem.getPosition());
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted){ armSubsystem.stop(); }

    @Override
    public boolean isFinished(){
        return (armSubsystem.isFinished() && Math.abs(armSubsystem.getPosition() - (position + armSubsystem.getStartPos())) <= 50);
    }
}
