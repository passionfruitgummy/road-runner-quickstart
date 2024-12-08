package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveDistanceCommand extends CommandBase {

    public static final double TICKS_PER_REV = 537.7;
    public static final double WHEEL_CIRCUMFERENCE = 9.6 * Math.PI; // in cm
    public static final double MULTIPLIER = 1.414;

    private final DriveSubsystem driveSubsystem;
    private final double distance, angle, speed;
    private double FL_and_BR_TargetChange, FR_and_BL_TargetChange, FL_and_BR_Theta, FR_and_BL_Theta;
    private final Telemetry telemetry;
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double distance, double angle, double speed, Telemetry telemetry){
        this.driveSubsystem = driveSubsystem;
        this.distance = distance;
        this.angle = angle; // angle to drive at (between 90 and -90), flip sign of speed to go opposite direction
        this.speed = speed;
        this.telemetry = telemetry;
    }

    @Override
    public void initialize(){
        driveSubsystem.resetEncoders();
        driveSubsystem.setDrive(Math.toRadians(angle), speed, false);
        FL_and_BR_Theta = (angle <= 45 && angle >= -90) ? Math.abs(angle + 45) : (135 - angle);
        FL_and_BR_TargetChange = distance * Math.cos(Math.toRadians(FL_and_BR_Theta)) * TICKS_PER_REV / WHEEL_CIRCUMFERENCE * MULTIPLIER;

        FR_and_BL_Theta = (angle <= 90 && angle >= -45) ? Math.abs(angle - 45) : (135 - Math.abs(angle));
        FR_and_BL_TargetChange = distance * Math.cos(Math.toRadians(FR_and_BL_Theta)) * TICKS_PER_REV / WHEEL_CIRCUMFERENCE * MULTIPLIER;

        telemetry.addLine("started drive distance command " + distance + " " + angle + " " + speed);
        telemetry.addData("FL and BR Theta", FL_and_BR_Theta);
        telemetry.addData("Fr and BL Theta", FR_and_BL_Theta);
        telemetry.update();
    }
    @Override
    public void execute(){
        telemetry.addData("Front left and Back right target change", FL_and_BR_TargetChange);
        telemetry.addData("Front right and Back left target change", FR_and_BL_TargetChange);
        telemetry.addData("Front left and Back right theta", FL_and_BR_Theta);
        telemetry.addData("Front right and Back left theta", FR_and_BL_Theta);
        for(Pair<String, String> pair : driveSubsystem.getInfo()){
            telemetry.addData(pair.first, pair.second);
        }
        telemetry.addLine("#############");
        telemetry.update();
    }

    @Override
    public void end(boolean interrupted){
        telemetry.addLine("finished drive distance command " + distance + " " + angle + " " + speed);
        telemetry.update();
        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished(){
        // distance measurements done in cm
        // stop once the difference between change and target change is very small or if they all overshot
//        return (Math.abs(driveSubsystem.getFLChange() - FL_and_BR_TargetChange) <= 5 && Math.abs(driveSubsystem.getFRChange() - FR_and_BL_TargetChange) <= 5 &&
//                Math.abs(driveSubsystem.getBRChange() - FL_and_BR_TargetChange) <= 5 && Math.abs(driveSubsystem.getBLChange() - FR_and_BL_TargetChange) <= 5) ||
        return        ((driveSubsystem.getFLChange() >= FL_and_BR_TargetChange) && (driveSubsystem.getFRChange() >= FR_and_BL_TargetChange) &&
                (driveSubsystem.getBRChange() >= FL_and_BR_TargetChange) && (driveSubsystem.getBLChange() >= FR_and_BL_TargetChange));
    }

}
