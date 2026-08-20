package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.RoadRunner.Drawing;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

@TeleOp(name = "FullTest", group = "Tests")
public class FullTest extends LinearOpMode {
    private Chassis chassis;
    private Sweeper sweeper;
    private ActionRunner actionRunner;

    // 当前目标 AprilTag ID（根据队伍颜色自动选择）
    private int targetTagId;

    // 队伍颜色
    private TeamColor teamColor;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ---- init 阶段：选择队伍颜色 ----
        telemetry.addData("Select Team Color", "");
        telemetry.addData("Press A (Blue)", "ResetPose: (63, 60.7, pi/2)");
        telemetry.addData("Press B (Red)", "ResetPose: (63, -60.7, -pi/2)");
        telemetry.update();

        while (!isStopRequested() && !gamepad1.a && !gamepad1.b) {
            idle();
        }

        if (gamepad1.a) {
            teamColor = TeamColor.BLUE;
            targetTagId = 20; // 蓝队球门 AprilTag ID
        } else {
            teamColor = TeamColor.RED;
            targetTagId = 24; // 红队球门 AprilTag ID
        }

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, true);
        sweeper = new Sweeper(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Team Color", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
        telemetry.addData("--- P1 Controls ---", "");
        telemetry.addData("Left Stick", "Chassis Drive");
        telemetry.addData("Right Stick X", "Chassis Rotation");
        telemetry.addData("X", "Toggle No-Head Mode");
        telemetry.addData("A", "Reset Pose to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red") + " ResetPose");
        telemetry.addData("Left Bumper", "Sweeper Eat");
        telemetry.addData("Right Bumper", "Sweeper Output + Flywheel Reverse + Trigger Launch");
        telemetry.addData("Y", "Sweeper Stop");
        telemetry.addData("--- P2 Controls ---", "");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            RobotPosition.getInstance().update();

            // ======== P1 Controls ========

            // 底盘移动（左摇杆 + 右摇杆 X）
            chassis.update(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            // 切换无头模式
            if (gamepad1.xWasReleased()) {
                chassis.exchangeUseNoHeadMode();
            }

            // 重置定位到对应颜色的 ResetPose
            if (gamepad1.aWasReleased()) {
                Pose2d resetPose = (teamColor == TeamColor.BLUE) ?
                        HypParams.ResetPoseBlue : HypParams.ResetPoseRed;
                RobotPosition.getInstance().ResetPoseTo(resetPose);
                telemetry.addData("ResetPose", "Reset to " + (teamColor == TeamColor.BLUE ? "Blue" : "Red"));
            }

            // 吸取器控制
            // 一操右 bumper 按下时：飞轮反转 + sweeper 反转 + 扳机舵机到发射位置
            if (gamepad1.right_bumper) {
                sweeper.setOutput();
            }

            // ======== P2 Controls ========



            // ======== 更新 & 遥测 ========

            sweeper.update();

            telemetry.addData("Team", teamColor == TeamColor.BLUE ? "BLUE" : "RED");
            telemetry.addData("useNoHeadMode", chassis.getUseNoHeadMode());

            // 位姿信息
            telemetry.addData("Pose X", "%.2f in", RobotPosition.getInstance().getX());
            telemetry.addData("Pose Y", "%.2f in", RobotPosition.getInstance().getY());
            telemetry.addData("Pose Theta", "%.2f deg", Math.toDegrees(RobotPosition.getInstance().getTheta()));

            // 目标信息
            telemetry.addData("Target Tag ID", targetTagId);
            double[] goalPos = HypParams.getGoalPosition(targetTagId);
            if (goalPos != null) {
                telemetry.addData("Target Coords", "(%.1f, %.1f)", goalPos[0], goalPos[1]);
            }

            chassis.telemetry();
            sweeper.setTelemetry();
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), RobotPosition.getInstance().getPose2d());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        chassis.stop();
        sweeper.setStop();
        sweeper.update();
    }
}