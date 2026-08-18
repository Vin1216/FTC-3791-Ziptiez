package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous
public class ItDLeftAutoV2 extends LinearOpMode {
    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.5);
                return false;
            }
        }
        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0);
                return false;
            }
        }
        public Action openClaw() {
            return new OpenClaw();
        }
    }
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
                //TODO: Lift Up Encoder Position
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
                //TODO: Lift Down Encoder Position
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
        private DcMotorEx arm;

        public Arm(HardwareMap hardwareMap) {
            arm = hardwareMap.get(DcMotorEx.class, "LiftMotor");
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.FORWARD);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class ArmUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                //TODO: Arm Up Encoder Position
                if (pos < 8250.0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armUp() { return new Arm.ArmUp(); }

        public class ArmDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    arm.setPower(1);
                    initialized = true;
                }

                double pos = arm.getCurrentPosition();
                packet.put("armPos", pos);
                //TODO: Arm Down Encoder Position
                if (pos > 0) {
                    return true;
                } else {
                    arm.setPower(0);
                    return false;
                }
            }
        }
        public Action armDown() { return new Arm.ArmDown(); }

    }
    DcMotorEx FrontLeft;
    DcMotorEx FrontRight;
    DcMotorEx RearLeft;
    DcMotorEx RearRight;
    MecanumDrive drive;
    @Override
    public void runOpMode() {
        Pose2d startpose = new Pose2d(72, 132, Math.toRadians(-90));
        Lift lift = new Lift(hardwareMap);
        Arm arm = new Arm(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        drive = new MecanumDrive(hardwareMap, startpose);

        FrontLeft = drive.leftFront;
        FrontRight = drive.rightFront;
        RearLeft = drive.leftBack;
        RearRight = drive.rightBack;

        TrajectoryActionBuilder PreloadTraj = drive.actionBuilder(startpose)
                .splineToSplineHeading(new Pose2d(116,116,Math.toRadians(45)),Math.toRadians(0));


        waitForStart();

        if(isStopRequested()) return;
        if(opModeIsActive()) {
            Actions.runBlocking(new SequentialAction(
                    PreloadTraj.build(),
                    arm.armUp(),
                    lift.liftUp(),
                    claw.openClaw(),
                    lift.liftDown()
            ));
            drive.updatePoseEstimate();
            TrajectoryActionBuilder BottomSampleTraj = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(96,58,Math.toRadians(270)),Math.toRadians(270));
//                    .splineToConstantHeading(new Vector2d(62,24),Math.toRadians(90));
            Actions.runBlocking(new SequentialAction(
                    BottomSampleTraj.build(),
                    arm.armDown(),
                    lift.liftUp(),
                    claw.closeClaw()
            ));
            drive.updatePoseEstimate();
            TrajectoryActionBuilder Score;
            Score = drive.actionBuilder(drive.pose)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(116,116,Math.toRadians(45)),Math.toRadians(0));
            Actions.runBlocking(new SequentialAction(
                    Score.build(),
                    arm.armUp(),
                    lift.liftUp(),
                    claw.openClaw()
            ));
        }
    }
}
