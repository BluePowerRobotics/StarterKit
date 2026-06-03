package org.firstinspires.ftc.teamcode.Controllers.Chassis;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * 简单的麦轮底盘控制器
 * 通过update(vx, vy, omega)手动控制速度，不使用RoadRunner
 */
public class Chassis {
    /** 左前电机 */
    public final DcMotorEx leftFront;
    /** 左后电机 */
    public final DcMotorEx leftBack;
    /** 右后电机 */
    public final DcMotorEx rightBack;
    /** 右前电机 */
    public final DcMotorEx rightFront;

    /** 最大线速度 (m/s) */
    private double maxV = 1.0;
    /** 最大角速度 (rad/s) */
    private double maxOmega = Math.PI;

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param maxV 最大线速度，默认1.0
     * @param maxOmega 最大角速度，默认PI
     */
    public Chassis(HardwareMap hardwareMap, double maxV, double maxOmega) {
        this.maxV = maxV;
        this.maxOmega = maxOmega;

        leftFront = hardwareMap.get(DcMotorEx.class, "fL");
        leftBack = hardwareMap.get(DcMotorEx.class, "bL");
        rightBack = hardwareMap.get(DcMotorEx.class, "bR");
        rightFront = hardwareMap.get(DcMotorEx.class, "fR");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Chassis(HardwareMap hardwareMap) {
        this(hardwareMap, 1.0, Math.PI);
    }

    /**
     * 每帧更新底盘速度
     * @param vx 相对坐标系x轴方向速度（前进，-1~1）
     * @param vy 相对坐标系y轴方向速度（左移，-1~1）
     * @param omega 角速度（逆时针，-1~1）
     */
    public void update(double vx, double vy, double omega) {
        double forward = vx * maxV;
        double strafe = vy * maxV;
        double rotation = omega * maxOmega;

        double lfPower = forward + strafe + rotation;
        double lbPower = forward - strafe + rotation;
        double rfPower = forward - strafe - rotation;
        double rbPower = forward + strafe - rotation;

        double maxPower = 1.0;
        maxPower = Math.max(maxPower, Math.abs(lfPower));
        maxPower = Math.max(maxPower, Math.abs(lbPower));
        maxPower = Math.max(maxPower, Math.abs(rfPower));
        maxPower = Math.max(maxPower, Math.abs(rbPower));

        leftFront.setPower(lfPower / maxPower);
        leftBack.setPower(lbPower / maxPower);
        rightFront.setPower(rfPower / maxPower);
        rightBack.setPower(rbPower / maxPower);
    }

    /**
     * 停止所有电机
     */
    public void stop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public double getMaxV() { return maxV; }
    public void setMaxV(double maxV) { this.maxV = maxV; }
    public double getMaxOmega() { return maxOmega; }
    public void setMaxOmega(double maxOmega) { this.maxOmega = maxOmega; }
}