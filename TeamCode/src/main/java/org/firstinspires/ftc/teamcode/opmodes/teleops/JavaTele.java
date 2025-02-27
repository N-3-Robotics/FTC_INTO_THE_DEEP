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

    @Override
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
            }
        }
    }

}