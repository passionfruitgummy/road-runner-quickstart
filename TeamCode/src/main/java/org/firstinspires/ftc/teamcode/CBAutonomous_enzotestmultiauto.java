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

@Autonomous(name="CB Autonomous CLOSE SCORE")
public class CBAutonomous_enzotestmultiauto extends CommandOpMode {

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
                // starting on close side
                /*
                - second closest tile to bucket
                - facing bucket
                - left wheels against wall
                - back of the robot above inner groove of the tile
                 */
                //drop off piece (left as is, maybe bump up speed?)
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 55, -45, 0.4, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, -4000, 0.8),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.5)),
                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 45, 0.25, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, , 0.8)),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 27, 0, 0.3, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND, 0.8)),
                new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 800, 0.5),
                        new DriveDistanceCommand(driveSubsystem, 3, -90, 0.4, telemetry)),
                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(2000),

                //idk what im doing (enzo)
                //arm positions for easy adjustment
                private static final int FIRSTYELLOWSAMPLEEXTEND = -800
                private static final int SECONDYELLOWSAMPLEEXTEND = -3000
                private static final int THIRDYELLOWSAMPLEEXTEND = -4500

                //arm up a bit
                new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 600, 0.5),
                //while moving backwards, rotate 45 degrees to the right so parallel with sample, retract arm, lower pivot
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, -54, 0, 0.4, telemetry),
                        new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 45, 0.25, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, FIRSTYELLOWSAMPLEEXTEND, 1)),
                        new PivotRunToPositionCommand(pivotSubsystem, pivotSubsystem.LOWEST_POS + 400, 0.5),
                //pick up piece
                new ParallelCommandGroup((new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 200, 0.5)))
                        new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(1000),
                //go back to basket (rotate the robot: while doing that, pivot extends to max, arm goes to max
                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, -45, 0.25, telemetry),
                        new PivotRunToPositionCommand(pivotSubsystem, pivotSubsystem.HIGHEST_POS, 0.5),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, -4000, 1),
                        new DriveDistanceCommand(driveSubsystem, 54, 0, 0.4, telemetry),
                new ParallelCommandGroup(
                        new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 800, 0.5),
                        new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(1000),
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 600, 0.5),
                //2nd sample?
                new ParallelCommandGroup(
                        new DriveRotateCommand(driveSubsystem, -45, 0.25, telemetry),
                        new DriveDistanceCommand(driveSubsystem, 54, 0, 0.4, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, SECONDYELLOWSAMPLEEXTEND, 1)),
                        new PivotRunToPositionCommand(pivotSubsystem, pivotSubsystem.LOWEST_POS + 500, 0.5),
                new ParallelCommandGroup(
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 300, 0.5),
                        new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(1000),




                // park by submersible
                new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 200, 0.5),
                        new DriveDistanceCommand(driveSubsystem, 14, 90, 0.3, telemetry)),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 54, 0, -0.4, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND, 1)),
                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 90, 0.25, telemetry),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND / 2, 0.75)),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 125, 0, -0.6, telemetry),
                    new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 0.75)),
                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 0, 0.25, telemetry),
                    new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS / 2, 0.5)),
                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 40, 0, -0.6, telemetry),
                        new PivotRunToPositionCommand(pivotSubsystem, 0, 0.5)),
                        new DriveStopCommand(driveSubsystem, telemetry),
                new LiftRunToAutoPositionCommand(liftSubsystem)
        ).schedule();
    }
}
