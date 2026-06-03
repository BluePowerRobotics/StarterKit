package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * 底盘测试器，用于手动测试底盘驱动功能
 * 左摇杆控制前后左右，右摇杆控制旋转
 */
@TeleOp(name = "ChassisTester", group = "Tests")
public class ChassisTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            double vx = -gamepad1.left_stick_y;
            double vy = -gamepad1.left_stick_x;
            double omega = -gamepad1.right_stick_x;

            chassis.update(vx, vy, omega);

            telemetry.addData("vx (forward)", vx);
            telemetry.addData("vy (strafe)", vy);
            telemetry.addData("omega (rotate)", omega);
            telemetry.addData("fL Power", chassis.leftFront.getPower());
            telemetry.addData("fR Power", chassis.rightFront.getPower());
            telemetry.addData("bL Power", chassis.leftBack.getPower());
            telemetry.addData("bR Power", chassis.rightBack.getPower());
            telemetry.update();
        }
    }
}