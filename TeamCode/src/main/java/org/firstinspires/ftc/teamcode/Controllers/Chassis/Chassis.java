package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.Localizer;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.HypParams;

public class Chassis {
    private final MecanumDrive drive;

    public Chassis(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose2d(0, 0, 0));
    }

    public Chassis(HardwareMap hardwareMap, Pose2d initialPose) {
        this.drive = new MecanumDrive(hardwareMap, initialPose);
        drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update(double vx, double vy, double omega) {
        double forward = vx * HypParams.maxV;
        double strafe = vy * HypParams.maxV;
        double rotation = omega * HypParams.maxOmega;

        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(forward, strafe), rotation));
    }

    public void update(Gamepad gamepad) {
        double vx = -gamepad.left_stick_y;
        double vy = -gamepad.left_stick_x;
        double omega = -gamepad.right_stick_x;
        update(vx, vy, omega);
    }

    public void stop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }
}