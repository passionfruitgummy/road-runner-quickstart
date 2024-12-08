package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubsystem extends SubsystemBase {

    public static final int EXTEND_POS = -3700, HOLD_POS = -800, AUTO_POS = -1000;
    private final DcMotorEx leftLift;
    private final DcMotorEx rightLift;

    private int leftStartPos;
    private int rightStartPos;

    public LiftSubsystem(DcMotorEx leftLift, DcMotorEx rightLift){
        this.leftLift = leftLift;
        this.rightLift = rightLift;

        leftStartPos = leftLift.getCurrentPosition();
        rightStartPos = rightLift.getCurrentPosition();
    }

    public void extend(boolean overrideLimits, boolean leftDown, boolean rightDown){
        if(!overrideLimits) {
            if(leftDown && !rightDown){
                leftLift.setTargetPosition(EXTEND_POS + leftStartPos);
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setTargetPosition(EXTEND_POS + rightStartPos);
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
           else{
                leftLift.setTargetPosition(EXTEND_POS + leftStartPos);
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setTargetPosition(EXTEND_POS + rightStartPos);
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else{
            if(leftDown && !rightDown){
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setPower(-1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(-1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    public void retract(boolean overrideLimits, boolean leftDown, boolean rightDown){
        if(!overrideLimits) {
            if(leftDown && !rightDown){
                leftLift.setTargetPosition(leftStartPos);
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setTargetPosition(rightStartPos);
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setTargetPosition(leftStartPos);
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                rightLift.setTargetPosition(rightStartPos);
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else{
            if(leftDown && !rightDown){
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(0);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else if(rightDown && !leftDown){
                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                leftLift.setPower(0);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            else{
                leftLift.setPower(1);
                leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                rightLift.setPower(1);
                rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

    }

    public void hold(){
        leftLift.setTargetPosition(leftStartPos + HOLD_POS);
        leftLift.setPower((leftLift.getCurrentPosition() < leftStartPos + HOLD_POS) ? 1 : -1);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightLift.setTargetPosition(rightStartPos + HOLD_POS);
        rightLift.setPower((rightLift.getCurrentPosition() < rightStartPos + HOLD_POS) ? 1 : -1);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void autoPosition(){
        leftLift.setTargetPosition(leftStartPos + AUTO_POS);
        leftLift.setPower((leftLift.getCurrentPosition() < leftStartPos + AUTO_POS) ? 1 : -1);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightLift.setTargetPosition(rightStartPos + AUTO_POS);
        rightLift.setPower((rightLift.getCurrentPosition() < rightStartPos + AUTO_POS) ? 1 : -1);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop(){
        leftLift.setPower(0);
        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightLift.setPower(0);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetStartPositions(){
        leftStartPos = leftLift.getCurrentPosition();
        rightStartPos = rightLift.getCurrentPosition();
    }

    public int getLeftStartPos(){
        return leftStartPos;
    }

    public int getRightStartPos(){
        return rightStartPos;
    }

    public int getLeftPos() { return leftLift.getCurrentPosition(); }
    public int getRightPos() { return rightLift.getCurrentPosition(); }

    public void autoOffset(){
        leftStartPos -= AUTO_POS;
        rightStartPos -= AUTO_POS;
    }

    public boolean isFinished(){
        return !leftLift.isBusy() && !rightLift.isBusy();
    }

}
