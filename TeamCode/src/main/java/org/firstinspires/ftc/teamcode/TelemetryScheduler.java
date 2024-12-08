package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryScheduler extends SubsystemBase {

    private final Telemetry telemetry;
    public TelemetryScheduler(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    public Telemetry getTelemetry(){
        return telemetry;
    }
}
