package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;

public class IntakeSubsystem {
    private final CRServo intake;

    public IntakeSubsystem(CRServo intake){
        this.intake = intake;
    }
    public void forward(){
        intake.setPower(1);
    }

    public void backward(){
        intake.setPower(-1);
    }
    public void stop(){
        intake.setPower(0);
    }

    public void run(double power) { intake.setPower(power); }
}
