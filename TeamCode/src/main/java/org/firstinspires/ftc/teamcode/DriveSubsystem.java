package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveSubsystem extends SubsystemBase {
    // see DriveSubsystem in examples
    private final MecanumDrive drive;
    private final DcMotorEx FrontL, FrontR, BackL, BackR;
    private int FLStartPos, FRStartPos, BLStartPos, BRStartPos;

    private final BHI260IMU imu;

    public DriveSubsystem(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu){
        drive = new MecanumDrive(FrontL, FrontR, BackL, BackR, imu);
        this.imu = imu;
        this.FrontL = FrontL;
        this.FrontR = FrontR;
        this.BackL = BackL;
        this.BackR = BackR;
    }

    public void drive(double x, double y, double rx, boolean isSlow){
        drive.driveFieldCentric(x, y, rx, isSlow);
    }

    public void resetEncoders(){
        FLStartPos = FrontL.getCurrentPosition();
        FRStartPos = FrontR.getCurrentPosition();
        BLStartPos = BackL.getCurrentPosition();
        BRStartPos = BackR.getCurrentPosition();
    }

    public int getFLChange(){
        return Math.abs(FrontL.getCurrentPosition() - FLStartPos);
    }

    public int getFRChange(){
        return Math.abs(FrontR.getCurrentPosition() - FRStartPos);
    }

    public int getBLChange(){
        return Math.abs(BackL.getCurrentPosition() - BLStartPos);
    }

    public int getBRChange(){
        return Math.abs(BackR.getCurrentPosition() - BRStartPos);
    }

    public double getAngle(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void stop(){
        drive.stop();
    }

    public void setDrive(double angle, double speed, boolean end){
        drive.setDrive(angle, speed, end);
    }

    public void setRotation(double angle, double speed, boolean end){
        drive.setRotation(angle, speed, end);
    }

    public Pair<String, String>[] getInfo(){
        //return drive.getInfo();
        return new Pair[]{
                new Pair<String, String>("Front Left Change", getFLChange() + ""),
                new Pair<String, String>("Front Right Change", getFRChange() + ""),
                new Pair<String, String>("Back Left Change", getBLChange() + ""),
                new Pair<String, String>("Back Right Change", getBRChange() + "")
        };

    }
}
