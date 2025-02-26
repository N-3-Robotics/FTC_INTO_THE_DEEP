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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

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
        Robot robot = new Robot(hardwareMap);
        double startPose = Pose2d(-24 - (16.75 / 2), -(60 + ((24-17))) )





    }

}