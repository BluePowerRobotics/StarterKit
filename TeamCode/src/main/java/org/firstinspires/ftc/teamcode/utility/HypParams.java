package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
public class HypParams {
    /**
     * 底盘最大速度（英寸/秒）
     */
    public static double maxV = 2.0;
    /**
     * 底盘最大角速
     */
    public static double maxOmega = Math.PI;
}