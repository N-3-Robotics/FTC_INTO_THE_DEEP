package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import kotlin.ranges.RangesKt;
import kotlin.Triple;
import kotlin.jvm.internal.Intrinsics;

enum ArmState{
    MANUAL,
    INTAKING,
    OUTTAKING,
    SAMPLE,
    RESETTING,
    HANGING;
}

@TeleOp

public class JavaTele extends LinearOpMode{
    //dist sensor constants
    int MIN_PIVOT_DIST = 72;
    int MAX_PIVOT_DIST = 150;

    //preset positions using dist sensor values
    int INTAKE_PIVOT_DIST = 72;
    int OUTTAKE_PIVOT_DIST = 145;
    int RESET_PIVOT_DIST = 72;

    //lift positions using encoder values
    int INTAKE_LIFT_POS = 0;
    int OUTTAKE_LIFT_POS = 2910;
    int RESET_LIFT_POS = 0;

    //wrist positions (servo vals 0-1)
    double INTAKE_WRIST_POS = 0.38;
    double OUTTAKE_WRIST_POS = 0.955;
    double RESET_WRIST_POS = 0.0;

    int HANGING_THRESHOLD = 10;
    boolean ENABLE_CONSTRAINTS = true;

    private final Triple updateManualControl(Robot robot, Gamepad gamepad){
        double currentPivotDist = robot.getPIV_DIST().getDistance(DistanceUnit.MM);
        double pivotPower = (double)gamepad.left_stick_y;
        double liftPower = (double)gamepad.right_trigger - (double)gamepad.left_trigger;
        double intakePower = gamepad.right_bumper ? 1.0 : (gamepad.left_bumper ? -1.0 : 0.0);
        double wristDelta = (double)(-gamepad.right_stick_y) * 0.05;
        robot.getWRIST().setPosition(RangesKt.coerceIn(robot.getWRIST().getPosition() + wristDelta, 0.0, 1.0));
        if(currentPivotDist >= 150.0 && pivotPower > 0.0){
            pivotPower = 0.0;
        }else if(currentPivotDist <= 72.0 && pivotPower < 0.0){
            pivotPower = 0.0;
        }

        double pivotRatio = (currentPivotDist - 72.0) / 78.0;
        double maxSlideExtension = this.lerp(1700.0,3300.0,pivotRatio);
        if((double)robot.getLIFT().getCurrentPosition() > maxSlideExtension && liftPower > 0.0) {
            liftPower = 0.0;
        }else if(robot.getLIFT().getCurrentPosition() < 0 && liftPower < 0.0){
            liftPower = 0.0;
        }

        return new Triple(pivotPower, liftPower, intakePower);
    }

    private final boolean moveToPosition(Robot robot, double targetDist, int targetLift, double targetWrist){
        //find distance error and lift error
        double curentDist = robot.getPIV_DIST().getDistance(DistanceUnit.MM);
        double distError = targetDist - curentDist;
        int liftError = targetLift - robot.getLIFT().getCurrentPosition();

        //rectify pivot error
        double pivotPower = RangesKt.coerceIn(distError * 0.1, -0.5, 0.5);
        robot.getPIVOT().setPower(pivotPower);
        robot.getCPIVOT().setPower(pivotPower);

        //rectify lift error
        double liftPower = RangesKt.coerceIn((double)liftError * 0.02, -0.8, 0.8);
        robot.getLIFT().setPower(liftPower);
        robot.getWRIST().setPosition(targetWrist);

        //return whether or not the robot is within the threshold
        return Math.abs(distError) < 2.0 && Math.abs(liftError) < 30;
    }

    public void runOpMode(){
        ElapsedTime timer = new ElapsedTime();
        Telemetry[] var2 = new Telemetry[]{this.telemetry, FtcDashboard.getInstance().getTelemetry()};
        this.telemetry = (Telemetry)(new MultipleTelemetry(var2));
        Robot robot = new Robot(hardwareMap);
        Pose2d startPose = new Pose2d(-32.375, -63.5, Math.toRadians(90.0));
        MecanumDrive LOCALIZER = new MecanumDrive(hardwareMap, startPose);
        double driveSpeed = 0.5;
        ArmState currentState = ArmState.MANUAL;
        double lastIntakePower = 0.0;
        robot.getPIVOT().setPower(0.0);
        robot.getCPIVOT().setPower(0.0);
        robot.getWRIST().setPosition(0.0);
        this.telemetry.addData("Status", "Initialized", new Object[0]);
        this.telemetry.update();
        this.waitForStart();

        while(this.opModeIsActive()){
            this.telemetry.addData("Loop Time", timer.milliseconds());
            timer.reset();
            if(this.gamepad2.dpad_up){
                currentState = ArmState.MANUAL;
            } else if (this.gamepad2.dpad_left) {
                currentState = ArmState.RESETTING;
            }else if(this.gamepad2.dpad_right){
                currentState = ArmState.OUTTAKING;
            }else if(this.gamepad2.dpad_down){
                currentState = ArmState.HANGING;
            }else if(this.gamepad2.triangle){
                currentState = ArmState.SAMPLE;
            }

            int stateSelector = JavaTele.WhenMappings.$EnumSwitchMapping$0[currentState.ordinal()];
            switch(stateSelector){
                case 1:
                    Gamepad GP2 = this.gamepad2;
                    Intrinsics.checkNotNullExpressionValue(GP2, "gamepad2");

                    //grab the pivot power, lift power, and intake power from the manual control method
                    Triple var12 = this.updateManualControl(robot, GP2);
                    double pivotPower = (Double)var12.component1();
                    double liftPower = (Double)var12.component2();
                    double intakePower = (Double)var12.component3();

                    //set the pivot, lift, and intake power
                    robot.getPIVOT().setPower(pivotPower);
                    robot.getCPIVOT().setPower(pivotPower);
            }
        }
    }

    private final double lerp(double a, double b, double t){
        return a + (b - a) * t;
    }

    public class WhenMappings{
        public static final int[] $EnumSwitchMapping$0;

        static{
            int[] stateCode = new int[ArmState.values().length];

            try{
                stateCode[ArmState.MANUAL.ordinal()] = 1;   //see if code matches with an enum value
            }catch (NoSuchFieldError ignored){  //if code doesn't match with an enum value, ignore
            }

            try{
                stateCode[ArmState.RESETTING.ordinal()] = 2;
            }catch(NoSuchFieldError ignored){
            }

            try{
                stateCode[ArmState.INTAKING.ordinal()] = 3;
            }catch(NoSuchFieldError ignored){
            }

            try{
                stateCode[ArmState.SAMPLE.ordinal()] = 4;
            }catch(NoSuchFieldError ignored){
            }

            try{
                stateCode[ArmState.OUTTAKING.ordinal()] = 5;
            }catch (NoSuchFieldError ignored){
            }

            try{
                stateCode[ArmState.HANGING.ordinal()] = 6;
            }catch(NoSuchFieldError ignored){
            }

            $EnumSwitchMapping$0 = stateCode;   //set states array value to corresponding code
        }
    }
}