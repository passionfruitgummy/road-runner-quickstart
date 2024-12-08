package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Test Motor and Servo", group="Linear OpMode")
@Disabled
public class TestMotorAndServo extends LinearOpMode {

    private CRServo servo;
    private DcMotorEx arm, pivot;

    @Override
    public void runOpMode() {

        waitForStart();
        servo = hardwareMap.get(CRServo.class, "servo");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        pivot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        while(opModeIsActive()){
            if(gamepad1.dpad_left){
                arm.setPower(1);
                telemetry.addLine("Arm power: 1");
            }
            else if(gamepad1.dpad_right){
                arm.setPower(-1);
                telemetry.addLine("Arm power: -1");
            }
            else{
                arm.setPower(0);
                telemetry.addLine("Arm power: 0");
            }
            if(gamepad1.dpad_up){
                pivot.setPower(1);
                telemetry.addLine("Pivot power: 1");
            }
            else if(gamepad1.dpad_down){
                pivot.setPower(-1);
                telemetry.addLine("Pivot power: -1");
            }
            else{
                pivot.setPower(0);
                telemetry.addLine("Pivot power: 0");
            }
            if(gamepad1.x){
                servo.setPower(1);
                telemetry.addLine("Servo power: 1");
            }
            else if(gamepad1.b){
                servo.setPower(-1);
                telemetry.addLine("Servo power: -1");
            }
            else{
                servo.setPower(0);
                telemetry.addLine("Servo power: 0");
            }

            telemetry.update();
        }
    }
}
