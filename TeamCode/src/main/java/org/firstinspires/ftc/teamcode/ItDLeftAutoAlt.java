package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Disabled
@Config
@Autonomous
public class ItDLeftAutoAlt extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "LiftMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(1);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 8250.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new Lift.LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-1);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 0.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new Lift.LiftDown();
        }
    }

    public class Arm {
        private DcMotorEx Arm;

        public Arm(HardwareMap hardwareMap) {
            Arm = hardwareMap.get(DcMotorEx.class, "ArmMotor");
            Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            Arm.setDirection(DcMotorSimple.Direction.FORWARD);
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Arm.setPower(-0.9);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("ArmPos",pos);
                if (pos > 300) {
                    return true;
                }
                else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armUp() {
            return new ArmUp();
        }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Arm.setPower(0.9);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("ArmPos", pos);
                if (pos < 900) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }

        public Action armDown() {
            return new ArmDown();
        }
        public class ArmBalance implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Arm.setPower(0.9);
                    initialized = true;
                }

                double pos = Arm.getCurrentPosition();
                packet.put("ArmPos", pos);
                if (pos < 400) {
                    return true;
                } else {
                    Arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armBalance() {
            return new ArmBalance();
        }
    }
    DcMotorEx Intake;

    Servo Bucket;

    AprilTagProcessor myAprilTagProcessor;
    AprilTagDetection myAprilTagDetection;
    List<AprilTagDetection> myAprilTagDetections;
    double myAprilTagRobotPoseX = 0;
    double myAprilTagRobotPoseY = 0;
    double myAprilTagRobotPoseYaw;
    Integer myAprilTagIdCode;
    boolean AprilTagDetected = false;

    ElapsedTime AutoPeriodTime;
    ElapsedTime myTimer;

    int ticksperRevolution = 550;
    double wheelCircumference = 12.56;
    double ticksPerInch = ticksperRevolution / wheelCircumference;
    int tickstoDestination;
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 0.75;     // Maximum FWD power applied to motor
    static final double MAX_REV = -0.75;     // Maximum REV power applied to motor
    boolean rampUp = true;
    double power;
    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx RearLeft;
    DcMotorEx RearRight;


    String VincentsMessage = "My job is done. You guys better score a million points in TeleOp - Vincent";

    @Override
    public void runOpMode() {
        Pose2d startpose = new Pose2d(36, 66, Math.toRadians(-90));
        MecanumDriveOld drive = new MecanumDriveOld(hardwareMap, startpose);

        FrontLeft = drive.leftFront;
        FrontRight = drive.rightFront;
        RearLeft = drive.leftBack;
        RearRight = drive.rightBack;

        Arm arm = new Arm(hardwareMap);
        Lift lift = new Lift(hardwareMap);


        Intake = hardwareMap.get(DcMotorEx.class,"IntakeMotor");

        Bucket = hardwareMap.get(Servo.class,"bucket");


        AutoPeriodTime = new ElapsedTime();
        myTimer = new ElapsedTime();


//        Init_VisionPortal();
//
//        DetectAprilTags();
//        // TODO: I'm not sure if I should stop the program automatically if it can't find an AprilTag
//        if(AprilTagDetected) {
//            startpose = new Pose2d(myAprilTagRobotPoseX,myAprilTagRobotPoseY,myAprilTagRobotPoseYaw);
//            telemetry.addData("PoseEstimate",startpose);
//            telemetry.update();
//        }
//        else {
//            telemetry.addLine("No AprilTag Detected!");
//            telemetry.addLine("Please restart the program");
//            telemetry.update();
//        }


        TrajectoryActionBuilder PreloadTraj = drive.actionBuilder(startpose)
                .splineToSplineHeading(new Pose2d(59.5,53.5,Math.toRadians(45)),Math.toRadians(0));

        TrajectoryActionBuilder TopSampleTraj = PreloadTraj.fresh()
                .turnTo(-135)
                .splineToLinearHeading(new Pose2d(56,24,Math.toRadians(180)),Math.toRadians(0)) //This may need to be spline heading
                .splineToConstantHeading(new Vector2d(62,24),Math.toRadians(180));

        TrajectoryActionBuilder MiddleSampleTraj = PreloadTraj.fresh()
                .turnTo(-135)
                .splineToLinearHeading(new Pose2d(60,48,Math.toRadians(180)),Math.toRadians(0)); //This may need to be spline heading
//                .splineToConstantHeading(new Vector2d(54,24),Math.toRadians(180));

        TrajectoryActionBuilder BottomSampleTraj = PreloadTraj.fresh()
//                .turnTo(Math.toRadians(270))
//                .lineToY(36)
//                .strafeTo(new Vector2d(-24, 36));
                .splineToLinearHeading(new Pose2d(30.75,44,Math.toRadians(135)),Math.toRadians(135)); //This may need to be spline heading


        TrajectoryActionBuilder MoveBackwardBottom = null;
        TrajectoryActionBuilder MoveBackwardMiddle = null;

        String state = "ScorePreload";

        // Create a list of the possible states where the samples are. This is to keep track of which states have been used already.
        List<String> PickupSampleStates = new ArrayList<>();
        PickupSampleStates.add("PickupBottomSample");
        PickupSampleStates.add("PickupMiddleSample");
//        PickupSampleStates.add("PickupTopSample");
        //Middle and top are disabled for now since the systems do not move fast enough


        Bucket.setPosition(1);

        waitForStart();

        if(isStopRequested()) return;
        if(opModeIsActive()) {
            AutoPeriodTime.reset();
            while(opModeIsActive()) {
                switch(state) {
                    case "Test":
//                        EncoderMoveToPosition(Lift,3975,0.6);
//                        ScoreOnHighBasket();
                        Actions.runBlocking(new SequentialAction(
                                arm.armBalance()
                        ));
                        sleep(1000);
                        Actions.runBlocking(arm.armDown());
                        state = "AAAAAAAAAAAAAAAAAAA";
                        break;
                    case "ScorePreload":
                        Actions.runBlocking(new ParallelAction(
                                PreloadTraj.build(),
                                new SequentialAction(
                                        arm.armBalance(),lift.liftUp()
                                )
                        ));
                        myTimer.reset();
                        while(myTimer.milliseconds() < 1000 && opModeIsActive()) {
                            Bucket.setPosition(0.4);
                        }
                        Bucket.setPosition(1);
                        drive.updatePoseEstimate();
                        Actions.runBlocking(new SequentialAction(
                                lift.liftDown(),
                                arm.armUp()
                        ));
//                        ScoreOnHighBasket();
                        state = "PickupBottomSample";
                        break;
                    case "PickupTopSample":
                        Actions.runBlocking(new SequentialAction(TopSampleTraj.build()));
//                        PickupSample(10);
                        PickupSampleStates.remove("PickupTopSample");
                        state = "Score";
                        break;
                    case "PickupMiddleSample":
                        Actions.runBlocking(new SequentialAction(
                                MiddleSampleTraj.build(),
                                arm.armDown()
                        ));
                        Intake.setPower(-0.7);
                        MoveBackwardMiddle = drive.actionBuilder(new Pose2d(60,48,Math.toRadians(180)))
                                .setReversed(true)
                                .lineToY(35,new TranslationalVelConstraint(20),new ProfileAccelConstraint(-10,20))
                                .waitSeconds(0.2);
                        Actions.runBlocking(new SequentialAction(
                                MoveBackwardMiddle.build()
                        ));
                        Actions.runBlocking(new SequentialAction(arm.armUp()));
                        telemetry.addLine("Arm done moving");
                        telemetry.update();
                        Intake.setPower(0);
                        PickupSampleStates.remove("PickupMiddleSample");
                        state = "Score";
                        break;
                    case "PickupBottomSample":
                        Actions.runBlocking(new SequentialAction(
                                BottomSampleTraj.build(),
                                arm.armDown()
                        ));
                        Intake.setPower(-0.7);
                        MoveBackwardBottom = drive.actionBuilder(new Pose2d(31,44,Math.toRadians(135)))
                                .setReversed(true)
                                .lineToY(35,new TranslationalVelConstraint(20),new ProfileAccelConstraint(-10,20))
                                .waitSeconds(0.2);
                        Actions.runBlocking(new SequentialAction(
                                MoveBackwardBottom.build()
                        ));
                        Actions.runBlocking(new SequentialAction(arm.armUp()));
                        telemetry.addLine("Arm done moving");
                        telemetry.update();
                        Intake.setPower(0);
                        PickupSampleStates.remove("PickupBottomSample");
                        state = "Score";
                        break;
                    case "Score":
                        telemetry.addLine("Score");
                        telemetry.update();
                        TrajectoryActionBuilder Score;
//                        DetectAprilTags();
                        if(AprilTagDetected) {
                            Score = drive.actionBuilder(new Pose2d(myAprilTagRobotPoseX,myAprilTagRobotPoseY,myAprilTagRobotPoseYaw))
                                    .splineToSplineHeading(new Pose2d(58,49,Math.toRadians(45)),Math.toRadians(0));
                            telemetry.addLine("AprilTag found");
                            telemetry.update();
                        }
                        else {
                            drive.updatePoseEstimate();
                            Score = drive.actionBuilder(drive.pose)
                                    .splineToSplineHeading(new Pose2d(58,49,Math.toRadians(45)),Math.toRadians(0));
                            telemetry.addLine("No AprilTag found");
                            telemetry.update();
                        }
//                        Score = MoveBackwardBottom.fresh()
//                                .splineToSplineHeading(new Pose2d(56,60,Math.toRadians(30)),Math.toRadians(0));
//                        telemetry.addLine("No AprilTag found");
//                        telemetry.update();

                        Actions.runBlocking(new SequentialAction(
                                Score.build(),
                                arm.armDown(),
                                lift.liftUp()
                        ));
                        myTimer.reset();
                        while(myTimer.milliseconds() < 1000 && opModeIsActive()) {
                            Bucket.setPosition(0.3);
                        }
                        Bucket.setPosition(1);
                        Actions.runBlocking(new SequentialAction(
                                lift.liftDown(),
                                arm.armUp()
                        ));
                        //TODO: Add AprilTag localization here; it will likely need a revamp of the sample trajectories

//                        TrajectoryActionBuilder TurnTo210 = Score.fresh()
//                                        .turnTo(Math.toRadians(210));
//
//                        Actions.runBlocking(new SequentialAction(TurnTo210.build()));


//                        state = "AAAAAAAAAAAAAAAAAAAA";
                        // Goes park if there are no more samples to collect
                        if(PickupSampleStates.isEmpty()) {
                            state = "Park";
                        }
                        else {
                            state = "PickupBottomSample";
                        }
                        break;
                    case "Park":
                        drive.updatePoseEstimate();
                        TrajectoryActionBuilder ParkTraj = drive.actionBuilder(drive.pose)
                                .splineTo(new Vector2d(0,24), Math.toRadians(270));
                        Actions.runBlocking(new SequentialAction(ParkTraj.build()));
                        state = VincentsMessage;
                        break;
                }

                // If time is running out for the autonomous period, immediately go park.
                // TODO: Adjust time limit based on how much time is needed to park.
//                if(AutoPeriodTime.seconds() >= 25 && state != VincentsMessage) {
//                    state = "Park";
//                }
//                telemetry.addData("Current State: ",state);
//                telemetry.update();
            }
        }
    }

    private void Init_VisionPortal() {
        VisionPortal myVisionPortal;
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        // Create a VisionPortal, with the specified webcam name, AprilTag processor,
        // and assign it to a variable.
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);
    }

    private void DetectAprilTags() {
        // Get a list containing the latest detections, which may be stale.
        myAprilTagDetections = myAprilTagProcessor.getDetections();
        for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection_item;
            if (myAprilTagDetection.metadata != null) {
                AprilTagDetected = true;
                myAprilTagIdCode = myAprilTagDetection.id;
                // TODO: Test if the absolute value part of this is actually what is needed
                myAprilTagRobotPoseX = Math.abs(myAprilTagDetection.robotPose.getPosition().x);
                myAprilTagRobotPoseY = Math.abs(myAprilTagDetection.robotPose.getPosition().y);
                //TODO: Update the yaw value to reflect the actual orientation of the robot (the camera is on the back rn)
                myAprilTagRobotPoseYaw = myAprilTagDetection.robotPose.getOrientation().getYaw();
                telemetry.addData("AprilTag ID: ",myAprilTagIdCode);
                telemetry.addData("Robot X: ",myAprilTagRobotPoseX);
                telemetry.addData("Robot Y: ",myAprilTagRobotPoseY);
                telemetry.addData("Robot Angle: ",myAprilTagRobotPoseYaw);
                telemetry.update();
            }
            else {
                AprilTagDetected = false;
                return;
            }
        }
    }

    private void EncoderMoveToPosition(DcMotorEx motor, int ticks, double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(ticks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(ticks > motor.getCurrentPosition()) {
            motor.setPower(power);
        }
        else {
            motor.setPower(-power);
        }
        while(motor.isBusy() && opModeIsActive()) {
            telemetry.addData("motor ticks ", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void MoveForwardEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int) (Distance * ticksPerInch);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(0.1);
        FrontRight.setPower(0.1);
        RearLeft.setPower(0.1);
        RearRight.setPower(0.1);
        power = 0.1;
        while (FrontRight.isBusy() && FrontLeft.isBusy() && opModeIsActive()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(power);
            FrontRight.setPower(power);
            RearLeft.setPower(power);
            RearRight.setPower(power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        RearLeft.setPower(0);
        RearRight.setPower(0);
        DisableEncoders();
    }
    private void MoveBackwardEncoder(int Distance) {
        ResetEncoder();
        tickstoDestination = (int) -(Distance * ticksPerInch);
        FrontLeft.setTargetPosition(tickstoDestination);
        FrontRight.setTargetPosition(tickstoDestination);
        RearLeft.setTargetPosition(tickstoDestination);
        RearRight.setTargetPosition(tickstoDestination);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeft.setPower(-0.1);
        FrontRight.setPower(-0.1);
        RearLeft.setPower(-0.1);
        RearRight.setPower(-0.1);
        power = 0.1;
        while (RearLeft.isBusy()) {
            if (rampUp) {
                // Keep stepping up until we hit the max value.
                power += INCREMENT;
                if (power >= MAX_FWD) {
                    power = MAX_FWD;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else {
                // Keep stepping down until we hit the min value.
                power -= INCREMENT;
                if (power <= MAX_REV) {
                    power = MAX_REV;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }
            FrontLeft.setPower(-power);
            FrontRight.setPower(-power);
            RearLeft.setPower(-power);
            RearRight.setPower(-power);
            myTimer.reset();
            while (myTimer.milliseconds() <= CYCLE_MS) {
                telemetry.update();
            }
        }
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        DisableEncoders();
    }
    private void DisableEncoders() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    private void ResetEncoder() {
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
