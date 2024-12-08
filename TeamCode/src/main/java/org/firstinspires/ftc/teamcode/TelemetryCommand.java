package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TelemetryCommand extends CommandBase {
    private final TelemetryScheduler telemetryScheduler;
    private final Pair<String, DoubleSupplier>[] doubleInfo;
    private final Pair<String, BooleanSupplier>[] booleanInfo;
    private final Pair<String, String>[] stringInfo;
    private int count = 0;
    public TelemetryCommand(TelemetryScheduler telemetryScheduler, Pair<String, DoubleSupplier>[] doubleInfo,
                            Pair<String, BooleanSupplier>[] booleanInfo, Pair<String, String>[] stringInfo){
        this.telemetryScheduler = telemetryScheduler;
        this.doubleInfo = doubleInfo;
        this.booleanInfo = booleanInfo;
        this.stringInfo = stringInfo;
        addRequirements(this.telemetryScheduler);
    }

    @Override
    public void execute(){
        Telemetry telemetry = telemetryScheduler.getTelemetry();

        telemetry.addLine(count++ + "\n");

        for(Pair<String, DoubleSupplier> pair : doubleInfo){
            telemetry.addData(pair.first, pair.second.getAsDouble());
        }

        telemetry.addLine("######################");

        for(Pair<String, BooleanSupplier> pair : booleanInfo){
            telemetry.addData(pair.first, pair.second.getAsBoolean());
        }

        telemetry.addLine("######################");

        for(Pair<String, String> pair : stringInfo){
            telemetry.addData(pair.first, pair.second);
        }

        telemetry.update();

    }
}
