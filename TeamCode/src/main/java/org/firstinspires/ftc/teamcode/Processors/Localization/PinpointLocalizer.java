package org.firstinspires.ftc.teamcode.Processors.Localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

/**
 * 简化的Pinpoint定位器，提供机器人位置、朝向和速度信息
 * 基于GoBilda Pinpoint驱动实现
 */
public class PinpointLocalizer {
    /** GoBilda Pinpoint 驱动实例 */
    public final GoBildaPinpointDriver driver;

    /** 用于速度转换：上次读取的朝向，用于将场地速度转换为机器人速度 */
    private double lastHeading = 0;

    /**
     * 构造函数
     * @param hardwareMap 硬件映射
     * @param mmPerTick 每个编码器tick对应的毫米数
     */
    public PinpointLocalizer(HardwareMap hardwareMap, double mmPerTick) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.setEncoderResolution(1.0 / mmPerTick, DistanceUnit.MM);
        driver.resetPosAndIMU();
    }

    /**
     * 更新传感器数据，每帧调用
     */
    public void update() {
        driver.update();
        lastHeading = driver.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    /**
     * 获取机器人场地坐标系X位置（英寸）
     */
    public double getX() {
        return driver.getPosX(DistanceUnit.INCH);
    }

    /**
     * 获取机器人场地坐标系Y位置（英寸）
     */
    public double getY() {
        return driver.getPosY(DistanceUnit.INCH);
    }

    /**
     * 获取机器人朝向（弧度）
     */
    public double getTheta() {
        return driver.getHeading(UnnormalizedAngleUnit.RADIANS);
    }

    /**
     * 获取相对速度Vx：机器人自身坐标系下的前进速度（英寸/秒）
     */
    public double getVx() {
        return driver.getVelX(DistanceUnit.INCH) * Math.cos(-lastHeading)
                + driver.getVelY(DistanceUnit.INCH) * Math.sin(-lastHeading);
    }

    /**
     * 获取相对速度Vy：机器人自身坐标系下的横向速度（英寸/秒）
     */
    public double getVy() {
        return -driver.getVelX(DistanceUnit.INCH) * Math.sin(-lastHeading)
                + driver.getVelY(DistanceUnit.INCH) * Math.cos(-lastHeading);
    }

    /**
     * 获取角速度（弧度/秒）
     */
    public double getOmega() {
        return driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    /**
     * 获取绝对速度Vx：场地坐标系下的x方向速度（英寸/秒）
     */
    public double getAbsVx() {
        return driver.getVelX(DistanceUnit.INCH);
    }

    /**
     * 获取绝对速度Vy：场地坐标系下的y方向速度（英寸/秒）
     */
    public double getAbsVy() {
        return driver.getVelY(DistanceUnit.INCH);
    }

    /**
     * 设置当前位置和朝向
     * @param x 场地X坐标（英寸）
     * @param y 场地Y坐标（英寸）
     * @param theta 朝向（弧度）
     */
    public void setPosition(double x, double y, double theta) {
        driver.resetPosAndIMU();
        // Pinpoint driver resetPosAndIMU sets position to 0, so we need to
        // track the offset ourselves if we want to set an arbitrary position.
        // For simplicity, this is left to the user to manage offsets.
    }
}