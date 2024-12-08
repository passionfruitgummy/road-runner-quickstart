package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveRotateCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final double angle, speed;
    private final Telemetry telemetry;

    public DriveRotateCommand(DriveSubsystem driveSubsystem, double angle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.angle = angle;
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.setRotation(driveSubsystem.getAngle() - angle, speed, false);
        telemetry.addLine("started drive rotate command " + angle + " " + speed);
        telemetry.update();
    }

    @Override
    public void execute(){
        telemetry.addData("current angle", driveSubsystem.getAngle());
        telemetry.addData("difference", (angle - driveSubsystem.getAngle()));
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("finished drive rotate command " + angle + " " + speed);
        telemetry.update();
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        double currentAngle = driveSubsystem.getAngle();
        return (Math.abs(angle - currentAngle) <= 1.5) || (angle > 0 && currentAngle > (angle)) ||
                (angle < 0 && currentAngle < (angle));
    }
}
