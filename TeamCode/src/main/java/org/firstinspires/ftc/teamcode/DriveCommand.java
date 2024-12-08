package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {
    // see DefaultDrive in examples
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier x, y, rx;
    private final BooleanSupplier isSlow;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rx, BooleanSupplier isSlow){
        this.driveSubsystem = driveSubsystem;
        this.x = x;
        this.y = y;
        this.rx = rx;
        this.isSlow = isSlow;
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute(){
        driveSubsystem.drive(x.getAsDouble(), y.getAsDouble(), rx.getAsDouble(), isSlow.getAsBoolean());
    }
}
