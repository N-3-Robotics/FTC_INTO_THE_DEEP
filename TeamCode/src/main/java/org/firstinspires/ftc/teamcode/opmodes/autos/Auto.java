package org.firstinspires.ftc.teamcode.opmodes.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Disabled
@Autonomous(name="Auto", group="Autos")
public class Auto extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "LIFT");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
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
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    // Pivot uses a motor called PIVOT

    public class Pivot {
        private DcMotorEx pivot;

        private Integer pivotDefault = 1980;

        public Pivot(HardwareMap hardwareMap) {
            pivot = hardwareMap.get(DcMotorEx.class, "PIVOT");
            pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            pivot.setDirection(DcMotorSimple.Direction.FORWARD);
            pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class PivotUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot.setPower(-0.8);
                    initialized = true;
                }

                double pos = pivot.getCurrentPosition();
                packet.put("pivotPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    pivot.setPower(0);
                    return false;
                }
            }
        }
        public Action pivotUp() {
            return new PivotUp();
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    pivot.setPower(0.8);
                    initialized = true;
                }

                double pos = pivot.getCurrentPosition();
                packet.put("pivotPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    pivot.setPower(0);
                    return false;
                }
            }
        }
        public Action pivotDown(){
            return new PivotDown();
        }

        public class PivotReset implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                   if (pivot.getCurrentPosition() > pivotDefault) {
                       pivot.setPower(-0.8);
                   }
                   else {
                       pivot.setPower(0.8);
                   }

                   initialized = true;
                }

                double pos = pivot.getCurrentPosition();
                packet.put("pivotPos", pos);
                telemetry.addData("pivotPos", pos);
                telemetry.update();

                if (Math.abs(pos - pivotDefault) < 10) {
                    pivot.setPower(0);
                    return false;
                }
                return true;
            }
        }

        public Action pivotReset() {
            return new PivotReset();
        }
    }

    public class Claw {
        private CRServo INTAKE;
        private CRServo LINTAKE;

        public Claw(HardwareMap hardwareMap) {
            INTAKE = hardwareMap.get(CRServo.class, "INTAKE");
            LINTAKE = hardwareMap.get(CRServo.class, "LINTAKE");
        }

        public class Intake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                INTAKE.setPower(1.0);
                LINTAKE.setPower(-1.0);
                sleep(1000);
                INTAKE.setPower(0);
                LINTAKE.setPower(0);
                return false;
            }
        }
        public Action intake() {
            return new Intake();
        }

        public class Outtake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                INTAKE.setPower(-1.0);
                LINTAKE.setPower(1.0);
                sleep(1000);
                INTAKE.setPower(0);
                LINTAKE.setPower(0);
                return false;
            }
        }
        public Action outtake() {
            return new Outtake();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(24, -(60+((double) (24-17)/2)), Math.toRadians(90.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Lift LIFT = new Lift(hardwareMap);
        Pivot PIVOT = new Pivot(hardwareMap);
        Claw CLAW = new Claw(hardwareMap);



        TrajectoryActionBuilder step1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(24, -42))
                .strafeTo(new Vector2d(-48.0, -42.0))
                .strafeTo(new Vector2d(-48.0, -48.0))
                .turnTo(Math.toRadians(-135.0));

        TrajectoryActionBuilder step2 = step1.endTrajectory().fresh()
                .strafeTo(new Vector2d(-36.0, -24.0-2.25))
                .turnTo(Math.toRadians(180.0));

        TrajectoryActionBuilder step3 = step2.endTrajectory().fresh()
                .strafeTo(new Vector2d(-48, -48))
                .turnTo(Math.toRadians(-135.0));

        Action step4 = drive.actionBuilder(new Pose2d(-48, -48, Math.toRadians(-135)))
                .strafeTo(new Vector2d(-34, -12))
                .build();



        Actions.runBlocking(PIVOT.pivotReset());

        waitForStart();

        Action s1 = step1.build();
        Action s2 = step2.build();
        Action s3 = step3.build();

        Actions.runBlocking(new SequentialAction(
                s1,
//                PIVOT.pivotUp(),
//                LIFT.liftUp(),
//                CLAW.outtake(),
//                LIFT.liftDown(),
                s2,
//                PIVOT.pivotDown(),
//                CLAW.intake(),
//                PIVOT.pivotReset(),
                s3,
//                PIVOT.pivotUp(),
//                LIFT.liftUp(),
//                CLAW.outtake(),
//                LIFT.liftDown(),

                step4
            ));
    }
}
