package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrive {
    // see MecanumDrive in examples
    final DcMotorEx FrontL;
    final DcMotorEx FrontR;
    final DcMotorEx BackL;
    final DcMotorEx BackR;
    final BHI260IMU imu;
    static final double MAX_DRIVE_PWR = 0.6, AUTO_DRIVE_PWR = 0.5;
    static final double X_AXIS_ADJ = 1.15; // x axis is a bit slower than y axis on strafer wheels
    static final double SLOW_MODE_POWER = 0.5;
    final double powerFactor = 1;

    private double autoFL = 0, autoFR = 0, autoBL = 0, autoBR = 0;

    public MecanumDrive(DcMotorEx FrontL, DcMotorEx FrontR, DcMotorEx BackL, DcMotorEx BackR, BHI260IMU imu){
        this.FrontL = FrontL;
        this.FrontR = FrontR;
        this.BackL = BackL;
        this.BackR = BackR;
        this.imu = imu;
    }

    public void driveFieldCentric(double x, double y, double rx, boolean isSlow){
        Pair<Double, Double> xy = joystickToBotPerspective(x, y);
        x = xy.first;
        y = xy.second;

        // lower joystick sensitivity
        y = adjPwr(y, isSlow);
        x = adjPwr(x, isSlow) * X_AXIS_ADJ;
        rx = adjPwr(rx * MAX_DRIVE_PWR*1.7, isSlow);

        double xPower = x * powerFactor;
        double yPower = y * powerFactor;

        double fLPwr = yPower + xPower + rx;
        double bLPwr = yPower - xPower + rx;
        double fRPwr = yPower - xPower - rx;
        double bRPwr = yPower + xPower - rx;

        // Put powers in the range of -1 to 1 only if they aren't already (not
        // checking would cause us to always drive at full speed)

        if (Math.abs(fLPwr) > 1 || Math.abs(bLPwr) > 1 ||
                Math.abs(fRPwr) > 1 || Math.abs(bRPwr) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(fLPwr), Math.abs(bLPwr));
            max = Math.max(Math.abs(fRPwr), max);
            max = Math.max(Math.abs(bRPwr), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs), scale everything to the max
            fLPwr /= max;
            bLPwr /= max;
            fRPwr /= max;
            bRPwr /= max;
        }

        driveWithMotorPowers(fLPwr, fRPwr, bLPwr, bRPwr);
    }

    public void setDrive(double angle, double speed, boolean end){
        angle += Math.PI / 2; // 0-degrees is forward
        double xPower = Math.cos(angle) * X_AXIS_ADJ;
        double yPower = Math.sin(angle);

        double fLPwr = (yPower + xPower) * speed;
        double bLPwr = (yPower - xPower) * speed;
        double fRPwr = (yPower - xPower) * speed;
        double bRPwr = (yPower + xPower) * speed;

        if(!end) {
            autoFL += fLPwr;
            autoFR += fRPwr;
            autoBL += bLPwr;
            autoBR += bRPwr;
        }
        else{ // undo power changes to end the command
            autoFL -= fLPwr;
            autoFR -= fRPwr;
            autoBL -= bLPwr;
            autoBR -= bRPwr;
        }
        autoDrive();
    }

    public void setRotation(double angle, double speed, boolean end){
        if(angle < 0) speed *= -1;

        if(!end){
            autoFL += speed;
            autoFR -= speed;
            autoBL += speed;
            autoBR -= speed;
        }
        else { // undo power changes to end the command
            autoFL -= speed;
            autoFR += speed;
            autoBL -= speed;
            autoBR += speed;
        }
        autoDrive();
    }

    public void autoDrive(){
        double fLPwr = autoFL; // cap the powers at 1 after making all of the changes based on direction, speed, and rotation
        double fRPwr = autoFR;
        double bLPwr = autoBL;
        double bRPwr = autoBR;
        if (Math.abs(fLPwr) > 1 || Math.abs(bLPwr) > 1 ||
                Math.abs(fRPwr) > 1 || Math.abs(bRPwr) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(fLPwr), Math.abs(bLPwr));
            max = Math.max(Math.abs(fRPwr), max);
            max = Math.max(Math.abs(bRPwr), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs), scale everything to the max
            fLPwr /= max;
            bLPwr /= max;
            fRPwr /= max;
            bRPwr /= max;
        }
        autoDriveWithMotorPowers(fLPwr, fRPwr, bLPwr, bRPwr);
    }

    private double adjPwr(Double n, boolean isSlow) {
        // when fine-tuning, just do linear scale so the max power is 25%
        if(isSlow) {
            return n * SLOW_MODE_POWER;
        } else {
            int nSign = n.compareTo((double) 0);
            return n;
            //return Math.pow(Math.abs(n), 1.5) * nSign;
        }
    }

    private Pair<Double, Double> joystickToBotPerspective(double jsX, double jsY) {
        double jsAngle = Math.atan2(jsY,jsX);

        // since the Java math library uses radians, angles from the IMU are also radians
        double jsMagnitude = Math.sqrt(jsX * jsX + jsY * jsY);
        double botAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // "Math.PI / 2" is for the offset between the driver orientation and the robot
        // orientation at the start of tele-op
        double targetAngle = jsAngle;// - botAngle;// - (Math.PI / 2); - removed field centric driving for now
        double botX = jsMagnitude * Math.cos(targetAngle);
        double botY = jsMagnitude * Math.sin(targetAngle);
        return new Pair<>(botX, botY);
    }

    public void driveWithMotorPowers(double FrontLSpeed, double FrontRSpeed, double BackLSpeed, double BackRSpeed){
        // add any necessary multipliers here
        FrontL.setPower(FrontLSpeed * MAX_DRIVE_PWR);
        FrontR.setPower(FrontRSpeed * MAX_DRIVE_PWR);
        BackL.setPower(BackLSpeed * MAX_DRIVE_PWR);
        BackR.setPower(BackRSpeed * MAX_DRIVE_PWR);
    }

    public void autoDriveWithMotorPowers(double FrontLSpeed, double FrontRSpeed, double BackLSpeed, double BackRSpeed){
        // add any necessary multipliers here
        FrontL.setPower(FrontLSpeed * AUTO_DRIVE_PWR);
        FrontR.setPower(FrontRSpeed * AUTO_DRIVE_PWR);
        BackL.setPower(BackLSpeed * AUTO_DRIVE_PWR);
        BackR.setPower(BackRSpeed * AUTO_DRIVE_PWR);
    }
    public void stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
        autoFL = 0;
        autoFR = 0;
        autoBL = 0;
        autoBR = 0;
    }

    public Pair<String, String>[] getInfo(){
        return new Pair[]{
                new Pair<String, String>("Front Left", autoFL + ""),
                new Pair<String, String>("Front Right", autoFR + ""),
                new Pair<String, String>("Back Left", autoBL + ""),
                new Pair<String, String>("Back Right", autoBR + ""),
        };
    }
}
