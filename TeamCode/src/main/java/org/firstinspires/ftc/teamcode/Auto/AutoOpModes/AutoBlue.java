package org.firstinspires.ftc.teamcode.Auto.AutoOpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Controllers.Chassis.Chassis;
import org.firstinspires.ftc.teamcode.Processors.RobotPosition.RobotPosition;
import org.firstinspires.ftc.teamcode.Controllers.Sweeper.Sweeper;
import org.firstinspires.ftc.teamcode.Auto.AutoActions.*;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ActionRunner;
import org.firstinspires.ftc.teamcode.Parameter.HypParams;
import org.firstinspires.ftc.teamcode.Parameter.TeamColor;

@Autonomous(name = "AutoActionBlue", group = "Auto")
public class AutoBlue extends LinearOpMode {
    private enum Phase {
        GOTO_START, PARK
    }

    private Chassis chassis;
    private Sweeper sweeper;
    private ActionRunner actionRunner;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        TeamColor teamColor = TeamColor.BLUE;
        int targetTagId = HypParams.targetTagIdBlue;

        actionRunner = new ActionRunner();
        chassis = new Chassis(hardwareMap, teamColor, actionRunner, telemetry, HypParams.StartPoseFarBlue);
        sweeper = new Sweeper(hardwareMap, telemetry);
        drive = RobotPosition.getInstance().getDrive();

        telemetry.addData("Status", "AutoActionBlue Initialized");
        telemetry.addData("StartPose", HypParams.StartPoseFarBlue);
        telemetry.update();

        waitForStart();

        Phase currentPhase = Phase.GOTO_START;
        boolean parkingStarted = false;

        while (opModeIsActive()) {
            // 检查时间：剩余不足且未开始停车 → 立即清空并启动GoToStopPose
            if (isTimeToPark() && !parkingStarted) {
                actionRunner.clear();
                sweeper.setStop();
                actionRunner.add(new GoToStopPose(drive, HypParams.StopPoseBlue));
                currentPhase = Phase.PARK;
                parkingStarted = true;
            }

            RobotPosition.getInstance().update();
            actionRunner.update();

            // action未完成，继续等待
            if (actionRunner.isBusy()) {
                telemetry.addData("Phase", currentPhase);
                telemetry.addData("Time", "%.1fs", getRuntime());
                telemetry.update();
                continue;
            }

            // 停车完成，退出
            if (currentPhase == Phase.PARK) {
                break;
            }

            // 根据当前phase启动下一个action
            switch (currentPhase) {
                case GOTO_START:
                    actionRunner.add(new GoToStartPose(drive, HypParams.StartPoseFarBlue));
                    currentPhase = Phase.PARK;
                    parkingStarted = true;
                    break;
            }

            telemetry.addData("Phase", currentPhase);
            telemetry.addData("Time", "%.1fs", getRuntime());
            telemetry.update();
        }

        chassis.stop();
    }

    private boolean isTimeToPark() {
        return getRuntime() * 1000 > (HypParams.AUTONOMOUS_DURATION_MS - HypParams.PARK_TIME_THRESHOLD_MS);
    }
}