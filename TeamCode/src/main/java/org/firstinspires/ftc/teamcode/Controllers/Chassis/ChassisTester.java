package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Processors.Localization.PinpointLocalizer;

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
        double mmPerTick = 1; //根据实测值决定
        PinpointLocalizer pinpointLocalizer = new PinpointLocalizer(hardwareMap, mmPerTick);
        waitForStart();

        while (opModeIsActive()) {
            chassis.update(gamepad1);
            telemetry.addData("vx (forward)", pinpointLocalizer.getVx());
            telemetry.addData("vy (strafe)", pinpointLocalizer.getVy());
            telemetry.addData("omega (rotate)", pinpointLocalizer.getOmega());
            telemetry.addData("fL Power", chassis.leftFront.getPower());
            telemetry.addData("fR Power", chassis.rightFront.getPower());
            telemetry.addData("bL Power", chassis.leftBack.getPower());
            telemetry.addData("bR Power", chassis.rightBack.getPower());
            telemetry.update();
        }
    }
}