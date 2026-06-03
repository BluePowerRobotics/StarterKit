package org.firstinspires.ftc.teamcode.utility.PID;

/**
 * PIDController类实现了基本的PID控制器功能
 * 用于根据目标值和测量值计算控制输出
 */
public class PIDController {
    /** 比例系数 */
    private double kP;
    /** 积分系数 */
    private double kI;
    /** 微分系数 */
    private double kD;
    /** 积分值 */
    private double integral;
    /** 上一次的误差值 */
    private double previousError;
    /** 积分上限 */
    private double maxI = 1;

    /**
     * 构造函数，初始化PID参数
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     */
    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 1);
    }

    /**
     * 构造函数，初始化PID参数和积分上限
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     * @param maxI 积分上限
     */
    public PIDController(double kP, double kI, double kD, double maxI) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxI = maxI;
        this.integral = 0;
        this.previousError = 0;
    }

    /**
     * 计算PID控制器输出
     * @param setpoint 目标值
     * @param measurement 当前测量值
     * @param dt 时间间隔（秒）
     * @return PID控制器输出
     */
    public double calculate(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral += error * dt;
        if (integral > maxI) {
            integral = maxI;
        } else if (integral < -maxI) {
            integral = -maxI;
        }
        double derivative = (error - previousError) / dt;
        previousError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    /**
     * 重置控制器状态
     */
    public void reset() {
        integral = 0;
        previousError = 0;
    }

    /**
     * 设置PID参数
     * @param kP 比例系数
     * @param kI 积分系数
     * @param kD 微分系数
     */
    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        if (kI == 0) integral = 0;
    }

    /**
     * 设置积分上限
     * @param maxI 积分上限
     */
    public void setMaxI(double maxI) {
        this.maxI = maxI;
    }
}