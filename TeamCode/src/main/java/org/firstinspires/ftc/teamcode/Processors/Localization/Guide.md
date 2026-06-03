# PinpointLocalizer 定位器使用指南

## 概述

`PinpointLocalizer` 是基于GoBilda Pinpoint驱动的简化定位器，提供机器人的绝对位置、朝向和速度信息。

---

## 使用方式

### 创建实例

```java
// mmPerTick: 每个编码器tick对应的毫米数
// 例如：使用GoBilda 4-bar odometry pod，mmPerTick约为0.246
double mmPerTick = 0.246;

PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, mmPerTick);
```

### 每帧更新

```java
// 在opMode循环中每帧调用
localizer.update();
```

### 获取位置和朝向

```java
// 场地坐标系下的绝对位置（英寸）
double x = localizer.getX();
double y = localizer.getY();
double theta = localizer.getTheta(); // 弧度
```

### 获取速度

```java
// 相对速度：机器人自身坐标系（英寸/秒）
double vx = localizer.getVx();    // 前进速度
double vy = localizer.getVy();    // 横向速度（左为正）
double omega = localizer.getOmega(); // 角速度（弧度/秒）

// 绝对速度：场地坐标系（英寸/秒）
double absVx = localizer.getAbsVx(); // 场地x方向速度
double absVy = localizer.getAbsVy(); // 场地y方向速度
```

---

## 完整示例

```java
@TeleOp(name = "LocalizationTest", group = "Tests")
public class LocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, 0.246);

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            telemetry.addData("X (in)", localizer.getX());
            telemetry.addData("Y (in)", localizer.getY());
            telemetry.addData("Theta (rad)", localizer.getTheta());
            telemetry.addData("Vx (in/s)", localizer.getVx());
            telemetry.addData("Vy (in/s)", localizer.getVy());
            telemetry.addData("Omega (rad/s)", localizer.getOmega());
            telemetry.addData("AbsVx (in/s)", localizer.getAbsVx());
            telemetry.addData("AbsVy (in/s)", localizer.getAbsVy());
            telemetry.update();
        }
    }
}
```

---

## 坐标系说明

- **场地坐标系**：`getX()`, `getY()`, `getAbsVx()`, `getAbsVy()` 使用场地坐标系
  - X轴：场地长边方向
  - Y轴：场地短边方向
- **机器人坐标系**：`getVx()`, `getVy()` 使用机器人自身坐标系
  - Vx：机器人前进方向
  - Vy：机器人左侧方向

---

## 硬件配置

在机器人控制器配置文件中添加：

```xml
<RobotConfig>
    <LynxUsbDevice name="Expansion Hub 2" ...>
        <I2cBus bus="0">
            <GoBildaPinpoint name="pinpoint" port="0" />
        </I2cBus>
    </LynxUsbDevice>
</RobotConfig>
```

---

## mmPerTick 计算方法

```
mmPerTick = 轮子周长(mm) / 编码器每转tick数

例如：使用35mm直径的odometry轮
轮子周长 = π × 35mm ≈ 109.96mm
编码器分辨率 = 2000 ticks/rev (GoBilda odometry pod)
mmPerTick = 109.96 / 2000 ≈ 0.055mm

如果是直连编码器：
mmPerTick = π × 轮子直径 / 8192 (REV Through Bore)
```