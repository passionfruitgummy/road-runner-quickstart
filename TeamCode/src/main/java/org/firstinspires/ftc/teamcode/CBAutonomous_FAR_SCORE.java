package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="CB Autonomous FAR SCORE")
public class CBAutonomous_FAR_SCORE extends CommandOpMode {

    private DcMotorEx FrontL, FrontR, BackL, BackR;
    private DcMotorEx arm, pivot;
    private DcMotorEx leftLift, rightLift;
    private CRServo intake;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;

    private PivotSubsystem pivotSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private LiftSubsystem liftSubsystem;

    @Override
    public void initialize(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        FrontL = hardwareMap.get(DcMotorEx.class, "fl(eet footwork)");
        FrontR = hardwareMap.get(DcMotorEx.class, "fr(ank)");
        BackL = hardwareMap.get(DcMotorEx.class, "bl(itzcrank)");
        BackR = hardwareMap.get(DcMotorEx.class, "br(iar)");

        FrontL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.FORWARD);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.FORWARD);

        FrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        intake = hardwareMap.get(CRServo.class, "spinnything");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        armSubsystem = new ArmSubsystem(arm);

        pivotSubsystem = new PivotSubsystem(pivot);

        intakeSubsystem = new IntakeSubsystem(intake);

        liftSubsystem = new LiftSubsystem(leftLift, rightLift);

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);

        // schedule all commands in this method
        waitForStart();
        new SequentialCommandGroup(
                // starting on far side
                /*
                - third closest tile to observatory
                - facing bucket
                - left wheels against wall
                - front of the robot above inner groove of the tile (closer to bucket)
                 */
                //drive up to be at starting position for close (maybe need to wait first) have ~6 extra seconds
                //new WaitCommand(0), //might need wait
                new DriveDistanceCommand(driveSubsystem, 55, -45, 0.4, telemetry),

                //drop off piece
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 95, 0, 0.6, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, -4000, 0.8),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.5)),
                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 45, 0.25, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND, 0.8)),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 31.5, 0, 0.3, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND, 0.8)),
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 800, 0.5),
                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(2000),

                // park in observatory
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.5),
                new DriveRotateCommand(driveSubsystem, -1, 0.20, telemetry),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 275, 0, -0.75, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 0.75),
                        new PivotRunToPositionCommand(pivotSubsystem, 0, 0.75))
        ).schedule();
    }
}
