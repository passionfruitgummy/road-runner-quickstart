package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PivotSubsystem extends SubsystemBase {

    public static final int HIGHEST_POS = 2950, LOWEST_POS = -3950;
    private final DcMotorEx pivot;
    private int startPos;
    private ArmSubsystem armSubsystem;

    public PivotSubsystem(DcMotorEx pivot){
        this.pivot = pivot;
        startPos = pivot.getCurrentPosition();
    }

    public void setArmSubsystem(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;
    }

    public void raise(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits){
            pivot.setTargetPosition(startPos + HIGHEST_POS);
            pivot.setPower(power);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(power);
        }
        if(pivot.getCurrentPosition() < startPos && armSubsystem.getPosition() < armSubsystem.getStartPos() + ArmSubsystem.LIMITED_EXTEND){
            armSubsystem.runToPosition(ArmSubsystem.LIMITED_EXTEND, 0.75);
        }
    }

    public void lower(boolean overrideLimits, boolean slowMode){
        double power = slowMode ? 0.5 : 1;
        if(!overrideLimits){
            pivot.setTargetPosition(startPos + LOWEST_POS);
            pivot.setPower(-power);
            pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else {
            pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot.setPower(-power);
        }
        if(pivot.getCurrentPosition() < startPos && armSubsystem.getPosition() < armSubsystem.getStartPos() + ArmSubsystem.LIMITED_EXTEND){
            armSubsystem.runToPosition(ArmSubsystem.LIMITED_EXTEND, 0.75);
        }
    }

    public void stop(){
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setPower(0);
    }

    public void runToPosition(int position, double power){
        pivot.setTargetPosition(startPos + position);
        if(startPos + position > pivot.getCurrentPosition()){
            pivot.setPower(power);
        }
        else{
            pivot.setPower(-power);
        }
        pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public boolean isFinished(){
        return !pivot.isBusy();
    }

    public void resetStartPosition(){
        startPos = pivot.getCurrentPosition();
    }

    public int getStartPos(){
        return startPos;
    }

    public int getCurrentPos() { return pivot.getCurrentPosition(); }

}
