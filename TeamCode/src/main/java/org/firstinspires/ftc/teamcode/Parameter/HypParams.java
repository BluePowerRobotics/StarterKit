package org.firstinspires.ftc.teamcode.Parameter;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.utility.Geometry.ConvexPolygon;
import org.firstinspires.ftc.teamcode.utility.Vector2D;

@Config
/**
 * 全局超参数配置类
 * 用于集中管理机器人控制系统中的所有超参数，超参应为无需拟合的常量，如工程参数等
 */
public class HypParams {
    /**
     * todo:机器人碰撞框（单位：英寸）
     * 定义机器人在场地中的碰撞边界，用于避障和边界检测
     */
    public static ConvexPolygon BoundingBox = new ConvexPolygon(
            new Vector2D(6.8, 4.8),
            new Vector2D(-6.8, 4.8),
            new Vector2D(-6.8, -4.8),
            new Vector2D(6.8, -4.8)
    );

    /**
     * 左方得分区域（单位：英寸）
     * 机器人在此区域内可以进行得分
     */
    public static ConvexPolygon AREA_LEFT = new ConvexPolygon(
            new Vector2D(0, 0),
            new Vector2D(-72, 72),
            new Vector2D(-72, -72)
    );

    /**
     * 右方射击区域（单位：英寸）
     * 机器人在此区域内可以进行射击
     */
    public static ConvexPolygon AREA_RIGHT = new ConvexPolygon(
            new Vector2D(72, 24),
            new Vector2D(72, -24),
            new Vector2D(48, 0)
    );

    /**
     * todo:相机仰角（单位：度）
     */
    public static double WebcamTheta = 20;

    /**
     * todo:相机在地面投影与车基准点的水平距离（单位：英寸）
     * 相机位于车基准点正前方该距离处，用于校正视觉瞄准时的水平距离计算
     */
    public static double WebCamCenterDistance = 30;

    /**
     * 底盘最大速度（英寸/秒）
     */
    public static double maxV = 2.0;

    /**
     * 底盘最大角速
     */
    public static double maxOmega = Math.PI;

    /**
     * todo:红队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseRed = new Pose2d(-41.3, 55,0);

    /**
     * todo:蓝队初始姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d startPoseBlue = new Pose2d(-41.3, -55, 0);

    /**
     * 红队远距离起始姿态（单位：英寸，弧度）
     */
    public static Pose2d StartPoseFarRed = new Pose2d(64.2, 29.4, Math.PI);

    /**
     * 蓝队远距离起始姿态（单位：英寸，弧度）
     */
    public static Pose2d StartPoseFarBlue = new Pose2d(64.2, -29.4, Math.PI);

    public static Pose2d StopPoseBlue = new Pose2d(0, -24, Math.PI);

    public static Pose2d StopPoseRed = new Pose2d(0, 24, Math.PI);

    /**
     * todo:红队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseRed = new Pose2d(63, -60.7, -Math.PI/2);

    /**
     * todo:蓝队重置姿态（单位：英寸，弧度）
     * 包含初始位置(x, y)和初始朝向(theta)
     */
    public static Pose2d ResetPoseBlue = new Pose2d(63, 60.7, Math.PI/2);

    /**
     * 初始操控模式标志
     * true=无头模式（场心地坐标系），false=有头模式（机器人坐标系）
     */
    public static boolean InitialUseNoHeadMode = false;

    /**
     * 球的质量（单位：千克）
     * 用于RK4弹道动力学计算
     */
    public static double ballMass = 0.06;

    /**
     * todo:停车时间阈值（单位：毫秒）
     * 自动阶段剩余时间小于此值时执行停车
     */
    public static long PARK_TIME_THRESHOLD_MS = 3000;

    /**
     * 自动阶段总时长（单位：毫秒）
     */
    public static long AUTONOMOUS_DURATION_MS = 30000;

    /**
     * 红队目标 AprilTag ID
     */
    public static int targetTagIdRed = 24;

    /**
     * 蓝队目标 AprilTag ID
     */
    public static int targetTagIdBlue = 20;

    /**
     * 根据 AprilTag ID 获取球门位置
     * @param id AprilTag ID
     * @return {x, y} 坐标数组，如果找不到返回 null
     */
    public static double[] getGoalPosition(int id) {
        if (id==targetTagIdBlue) {
            return new double[]{-72, -72};
        }else if (id==targetTagIdRed){
            return new double[]{-72, 72};
        }
        return null;
    }

    //可能无用的东西：
    /**
     * todo:颜色低通滤波系数
     * 用于Tracker中对目标角度的滤波处理，值越小滤波越平滑
     */
    public static double ColorAlpha = 0.4;
    //此处结束

}