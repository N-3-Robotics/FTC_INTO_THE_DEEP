package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// RR-specific imports
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;


@Autonomous(name="Auto", group="Autos")
public class Auto extends LinearOpMode {
    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "LIFT");
            lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorEx.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    lift.setPower(1);
                }

                double pos = lift.getCurrentPosition();
                telemetryPacket.put("liftPos", pos);

                if (pos < 3000.0) {
                    return true;
                }
                else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;
                    lift.setPower(-1);
                }

                double pos = lift.getCurrentPosition();
                telemetryPacket.put("liftPos", pos);

                if (pos > 0.0) {
                    return true;
                }
                else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
    }

    public class PIVOT {
        private DcMotorEx pivot;

        public PIVOT(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(DcMotorEx.class, "PIVOT");
        }

        public class PivotUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;

                }

                return true;
            }
        }

        public Action pivotUp() {
            return new PivotUp();
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    initialized = true;

                }

                return true;
            }
        }

        public Action pivotDown() {
            return new PivotDown();
        }


    }



    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(36, -(60+((double) (24-17)/2)), Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift lift = new Lift(hardwareMap);
        PIVOT pivot = new PIVOT(hardwareMap);

        Action step1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(36, -42))
                .strafeTo(new Vector2d(-48.0, -42.0))
                .strafeTo(new Vector2d(-48.0, -48.0))
                .turnTo(Math.toRadians(-135.0))
                .build();

        Action step2 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(-135)))
                .strafeTo(new Vector2d(-36.0, -24.0-2.25))
                .turnTo(Math.toRadians(180.0))
                .build();

        Action step3 = drive.actionBuilder(new Pose2d(-36, -24.0-2.25, Math.toRadians(180)))
                .strafeTo(new Vector2d(-48, -48))
                .turnTo(Math.toRadians(-135.0))
                .build();

        Action step4 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(-135)))
                .strafeTo(new Vector2d(-34, 0))
                .build();


        waitForStart();
        Actions.runBlocking(new SequentialAction(step1, step2, step3, step4));
    }
}
