// ArmState.java
package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.jetbrains.annotations.NotNull;

import kotlin.Metadata;
import kotlin.Triple;
import kotlin.jvm.internal.DefaultConstructorMarker;
import kotlin.jvm.internal.Intrinsics;
import kotlin.ranges.RangesKt;

@Metadata(
        mv = {1, 8, 0},
        k = 1,
        xi = 48,
        d1 = {"\u0000\f\n\u0002\u0018\u0002\n\u0002\u0010\u0010\n\u0002\b\b\b\u0082\u0001\u0018\u00002\b\u0012\u0004\u0012\u00020\u00000\u0001B\u0007\b\u0002¢\u0006\u0002\u0010\u0002j\u0002\b\u0003j\u0002\b\u0004j\u0002\b\u0005j\u0002\b\u0006j\u0002\b\u0007j\u0002\b\b¨\u0006\t"},
        d2 = {"Lorg/firstinspires/ftc/teamcode/opmodes/teleops/ArmState;", "", "(Ljava/lang/String;I)V", "MANUAL", "INTAKING", "OUTTAKING", "SAMPLE", "RESETTING", "HANGING", "TeamCode_debug"}
)
enum ArmState {
    MANUAL,
    INTAKING,
    OUTTAKING,
    SAMPLE,
    RESETTING,
    HANGING;

    // $FF: synthetic method
    private static final ArmState[] $values() {
        ArmState[] var0 = new ArmState[]{MANUAL, INTAKING, OUTTAKING, SAMPLE, RESETTING, HANGING};
        return var0;
    }
}
// NewTeleOP.java
package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import kotlin.Metadata;
import kotlin.Triple;
import kotlin.jvm.internal.DefaultConstructorMarker;
import kotlin.jvm.internal.Intrinsics;
import kotlin.ranges.RangesKt;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.jetbrains.annotations.NotNull;

@TeleOp(
        name = "TeleOp"
)
@Metadata(
        mv = {1, 8, 0},
        k = 1,
        xi = 48,
        d1 = {"\u0000>\n\u0002\u0018\u0002\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0010\u0006\n\u0002\b\u0004\n\u0002\u0010\u000b\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\n\u0002\u0010\b\n\u0002\b\u0002\n\u0002\u0010\u0002\n\u0000\n\u0002\u0018\u0002\n\u0000\n\u0002\u0018\u0002\n\u0002\b\u0002\b\u0007\u0018\u0000 \u00162\u00020\u0001:\u0001\u0016B\u0005¢\u0006\u0002\u0010\u0002J \u0010\u0003\u001a\u00020\u00042\u0006\u0010\u0005\u001a\u00020\u00042\u0006\u0010\u0006\u001a\u00020\u00042\u0006\u0010\u0007\u001a\u00020\u0004H\u0002J(\u0010\b\u001a\u00020\t2\u0006\u0010\n\u001a\u00020\u000b2\u0006\u0010\f\u001a\u00020\u00042\u0006\u0010\r\u001a\u00020\u000e2\u0006\u0010\u000f\u001a\u00020\u0004H\u0002J\b\u0010\u0010\u001a\u00020\u0011H\u0016J*\u0010\u0012\u001a\u0014\u0012\u0004\u0012\u00020\u0004\u0012\u0004\u0012\u00020\u0004\u0012\u0004\u0012\u00020\u00040\u00132\u0006\u0010\n\u001a\u00020\u000b2\u0006\u0010\u0014\u001a\u00020\u0015H\u0002¨\u0006\u0017"},
        d2 = {"Lorg/firstinspires/ftc/teamcode/opmodes/teleops/NewTeleOP;", "Lcom/qualcomm/robotcore/eventloop/opmode/LinearOpMode;", "()V", "lerp", "", "a", "b", "t", "moveToPosition", "", "robot", "Lorg/firstinspires/ftc/teamcode/robot/Robot;", "targetDist", "targetLift", "", "targetWrist", "runOpMode", "", "updateManualControl", "Lkotlin/Triple;", "gamepad", "Lcom/qualcomm/robotcore/hardware/Gamepad;", "Companion", "TeamCode_debug"}
)
public final class NewTeleOP extends LinearOpMode {
    @NotNull
    private static final Companion Companion = new Companion((DefaultConstructorMarker)null);
    /** @deprecated */
    @Deprecated
    public static final double MIN_PIVOT_DIST = 72.0;
    /** @deprecated */
    @Deprecated
    public static final double MAX_PIVOT_DIST = 150.0;
    /** @deprecated */
    @Deprecated
    public static final double INTAKE_PIVOT_DIST = 72.0;
    /** @deprecated */
    @Deprecated
    public static final double OUTTAKE_PIVOT_DIST = 145.0;
    /** @deprecated */
    @Deprecated
    public static final double RESET_PIVOT_DIST = 72.0;
    /** @deprecated */
    @Deprecated
    public static final int INTAKE_LIFT_POS = 0;
    /** @deprecated */
    @Deprecated
    public static final int OUTTAKE_LIFT_POS = 2910;
    /** @deprecated */
    @Deprecated
    public static final int RESET_LIFT_POS = 0;
    /** @deprecated */
    @Deprecated
    public static final double INTAKE_WRIST_POS = 0.38;
    /** @deprecated */
    @Deprecated
    public static final double OUTTAKE_WRIST_POS = 0.955;
    /** @deprecated */
    @Deprecated
    public static final double RESET_WRIST_POS = 0.0;
    /** @deprecated */
    @Deprecated
    public static final int HANGING_THRESHOLD = 10;
    /** @deprecated */
    @Deprecated
    public static final boolean ENABLE_CONSTRAINTS = true;

    private final Triple updateManualControl(Robot robot, Gamepad gamepad) {
        double currentPivotDist = robot.getPIV_DIST().getDistance(DistanceUnit.MM);
        double pivotPower = (double)gamepad.left_stick_y;
        double liftPower = (double)gamepad.right_trigger - (double)gamepad.left_trigger;
        double intakePower = gamepad.right_bumper ? 1.0 : (gamepad.left_bumper ? -1.0 : 0.0);
        double wristDelta = (double)(-gamepad.right_stick_y) * 0.05;
        robot.getWRIST().setPosition(RangesKt.coerceIn(robot.getWRIST().getPosition() + wristDelta, 0.0, 1.0));
        if (currentPivotDist >= 150.0 && pivotPower > 0.0) {
            pivotPower = 0.0;
        } else if (currentPivotDist <= 72.0 && pivotPower < 0.0) {
            pivotPower = 0.0;
        }

        double pivotRatio = (currentPivotDist - 72.0) / 78.0;
        double maxSlideExtension = this.lerp(1700.0, 3300.0, pivotRatio);
        if ((double)robot.getLIFT().getCurrentPosition() > maxSlideExtension && liftPower > 0.0) {
            liftPower = 0.0;
        } else if (robot.getLIFT().getCurrentPosition() < 0 && liftPower < 0.0) {
            liftPower = 0.0;
        }

        return new Triple(pivotPower, liftPower, intakePower);
    }

    private final boolean moveToPosition(Robot robot, double targetDist, int targetLift, double targetWrist) {
        double currentDist = robot.getPIV_DIST().getDistance(DistanceUnit.MM);
        double distError = targetDist - currentDist;
        int liftError = targetLift - robot.getLIFT().getCurrentPosition();
        double pivotPower = RangesKt.coerceIn(distError * 0.1, -0.5, 0.5);
        robot.getPIVOT().setPower(pivotPower);
        robot.getCPIVOT().setPower(pivotPower);
        double liftPower = RangesKt.coerceIn((double)liftError * 0.02, -0.8, 0.8);
        robot.getLIFT().setPower(liftPower);
        robot.getWRIST().setPosition(targetWrist);
        return Math.abs(distError) < 2.0 && Math.abs(liftError) < 30;
    }

    public void runOpMode() {
        ElapsedTime timer = new ElapsedTime();
        Telemetry[] var2 = new Telemetry[]{this.telemetry, FtcDashboard.getInstance().getTelemetry()};
        this.telemetry = (Telemetry)(new MultipleTelemetry(var2));
        Robot ROBOT = new Robot(this.hardwareMap);
        Pose2d startPose = new Pose2d(-32.375, -63.5, Math.toRadians(90.0));
        MecanumDrive LOCALIZER = new MecanumDrive(this.hardwareMap, startPose);
        double driveSpeed = 0.5;
        ArmState currentState = ArmState.MANUAL;
        double lastIntakePower = 0.0;
        ROBOT.getPIVOT().setPower(0.0);
        ROBOT.getCPIVOT().setPower(0.0);
        ROBOT.getWRIST().setPosition(0.0);
        this.telemetry.addData("Status", "Initialized", new Object[0]);
        this.telemetry.update();
        this.waitForStart();

        while(this.opModeIsActive()) {
            this.telemetry.addData("Loop Time", timer.milliseconds());
            timer.reset();
            if (this.gamepad2.dpad_up) {
                currentState = ArmState.MANUAL;
            } else if (this.gamepad2.dpad_left) {
                currentState = ArmState.RESETTING;
            } else if (this.gamepad2.dpad_right) {
                currentState = ArmState.OUTTAKING;
            } else if (this.gamepad2.dpad_down) {
                currentState = ArmState.HANGING;
            } else if (this.gamepad2.triangle) {
                currentState = ArmState.SAMPLE;
            }

            int var11 = NewTeleOP.WhenMappings.$EnumSwitchMapping$0[currentState.ordinal()];
            switch (var11) {
                case 1:
                    Gamepad var13 = this.gamepad2;
                    Intrinsics.checkNotNullExpressionValue(var13, "gamepad2");
                    Triple var12 = this.updateManualControl(ROBOT, var13);
                    double pivotPower = ((Number)var12.component1()).doubleValue();
                    double liftPower = ((Number)var12.component2()).doubleValue();
                    double intakePower = ((Number)var12.component3()).doubleValue();
                    ROBOT.getPIVOT().setPower(pivotPower);
                    ROBOT.getCPIVOT().setPower(pivotPower);
                    ROBOT.getLIFT().setPower(liftPower);
                    if (intakePower != lastIntakePower) {
                        ROBOT.getINTAKE().setPower(intakePower);
                        ROBOT.getLINTAKE().setPower(intakePower);
                        lastIntakePower = intakePower;
                    }
                    break;
                case 2:
                    if (this.moveToPosition(ROBOT, 72.0, 0, 0.0)) {
                        currentState = ArmState.MANUAL;
                    }
                    break;
                case 3:
                    this.moveToPosition(ROBOT, 72.0, 0, 0.38);
                    ROBOT.getINTAKE().setPower(1.0);
                    ROBOT.getLINTAKE().setPower(1.0);
                    break;
                case 4:
                    if (this.moveToPosition(ROBOT, 70.0, 0, 0.395)) {
                        currentState = ArmState.MANUAL;
                    }
                    break;
                case 5:
                    this.moveToPosition(ROBOT, 145.0, 2910, 0.955);
                    ROBOT.getINTAKE().setPower(-1.0);
                    ROBOT.getLINTAKE().setPower(-1.0);
                    break;
                case 6:
                    if (Math.abs(ROBOT.getLIFT().getCurrentPosition()) > 10) {
                        ROBOT.getLIFT().setPower(-0.5);
                        ROBOT.getPIVOT().setPower(0.0);
                        ROBOT.getCPIVOT().setPower(0.0);
                    } else {
                        ROBOT.getLIFT().setPower(0.0);
                    }
            }

            if (this.gamepad1.cross) {
                driveSpeed = 0.25;
            } else if (this.gamepad1.circle) {
                driveSpeed = 0.5;
            } else if (this.gamepad1.square) {
                driveSpeed = 0.75;
            } else if (this.gamepad1.triangle) {
                driveSpeed = 1.0;
            }

            Gamepad var10 = this.gamepad1;
            Intrinsics.checkNotNullExpressionValue(var10, "gamepad1");
            ROBOT.gamepadDrive(var10, driveSpeed);
            LOCALIZER.updatePoseEstimate();
            Pose2d pose = LOCALIZER.localizer.getPose();
            this.telemetry.addData("Current State", currentState);
            this.telemetry.addData("Pivot Distance (mm)", ROBOT.getPIV_DIST().getDistance(DistanceUnit.MM));
            this.telemetry.addData("Lift Position", ROBOT.getLIFT().getCurrentPosition());
            this.telemetry.addData("Wrist Position", ROBOT.getWRIST().getPosition());
            this.telemetry.addData("Drive Speed", driveSpeed);
            this.telemetry.addLine();
            this.telemetry.addData("X", pose.position.x);
            this.telemetry.addData("Y", pose.position.y);
            this.telemetry.addData("Heading", Math.toDegrees(pose.heading.toDouble()));
            this.telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }

    private final double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    @Metadata(
            mv = {1, 8, 0},
            k = 1,
            xi = 48,
            d1 = {"\u0000\"\n\u0002\u0018\u0002\n\u0002\u0010\u0000\n\u0002\b\u0002\n\u0002\u0010\u000b\n\u0000\n\u0002\u0010\b\n\u0002\b\u0002\n\u0002\u0010\u0006\n\u0002\b\n\b\u0082\u0003\u0018\u00002\u00020\u0001B\u0007\b\u0002¢\u0006\u0002\u0010\u0002R\u000e\u0010\u0003\u001a\u00020\u0004X\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u0005\u001a\u00020\u0006X\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u0007\u001a\u00020\u0006X\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\b\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\n\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u000b\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\f\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\r\u001a\u00020\u0006X\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u000e\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u000f\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u0010\u001a\u00020\u0006X\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u0011\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000R\u000e\u0010\u0012\u001a\u00020\tX\u0086T¢\u0006\u0002\n\u0000¨\u0006\u0013"},
            d2 = {"Lorg/firstinspires/ftc/teamcode/opmodes/teleops/NewTeleOP$Companion;", "", "()V", "ENABLE_CONSTRAINTS", "", "HANGING_THRESHOLD", "", "INTAKE_LIFT_POS", "INTAKE_PIVOT_DIST", "", "INTAKE_WRIST_POS", "MAX_PIVOT_DIST", "MIN_PIVOT_DIST", "OUTTAKE_LIFT_POS", "OUTTAKE_PIVOT_DIST", "OUTTAKE_WRIST_POS", "RESET_LIFT_POS", "RESET_PIVOT_DIST", "RESET_WRIST_POS", "TeamCode_debug"}
    )
    private static final class Companion {
        private Companion() {
        }

        // $FF: synthetic method
        public Companion(DefaultConstructorMarker $constructor_marker) {
            this();
        }
    }

    // $FF: synthetic class
    @Metadata(
            mv = {1, 8, 0},
            k = 3,
            xi = 48
    )
    public class WhenMappings {
        // $FF: synthetic field
        public static final int[] $EnumSwitchMapping$0;

        static {
            int[] var0 = new int[ArmState.values().length];

            try {
                var0[ArmState.MANUAL.ordinal()] = 1;
            } catch (NoSuchFieldError var7) {
            }

            try {
                var0[ArmState.RESETTING.ordinal()] = 2;
            } catch (NoSuchFieldError var6) {
            }

            try {
                var0[ArmState.INTAKING.ordinal()] = 3;
            } catch (NoSuchFieldError var5) {
            }

            try {
                var0[ArmState.SAMPLE.ordinal()] = 4;
            } catch (NoSuchFieldError var4) {
            }

            try {
                var0[ArmState.OUTTAKING.ordinal()] = 5;
            } catch (NoSuchFieldError var3) {
            }

            try {
                var0[ArmState.HANGING.ordinal()] = 6;
            } catch (NoSuchFieldError var2) {
            }

            $EnumSwitchMapping$0 = var0;
        }
    }
}
