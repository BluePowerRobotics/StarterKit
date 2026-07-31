# Chassis 底盘控制器使用指南

## 概述

`Controllers/Chassis` 目录提供基于 RoadRunner 的麦轮底盘控制器及测试工具。底盘通过 `update(vx, vy, omega)` 手动控制速度，同时支持 RoadRunner 轨迹规划和定位功能。

---

## 类说明

### 1. Chassis

基于 RoadRunner MecanumDrive 的麦轮底盘控制器，提供手动控制和轨迹规划功能。

```java
// 创建底盘（默认初始位姿为原点）
Chassis chassis = new Chassis(hardwareMap);

// 或指定初始位姿
Chassis chassis = new Chassis(hardwareMap, new Pose2d(0, 0, 0));

// 每帧更新底盘速度（手动控制）
chassis.update(vx, vy, omega);

// 停止所有电机
chassis.stop();

// 更新位姿估计（用于定位）
chassis.updatePoseEstimate();

// 获取当前位姿
Pose2d pose = chassis.getPose();

// 设置当前位姿
chassis.setPose(new Pose2d(10, 10, Math.PI / 2));

// 创建轨迹动作构建器（自动模式）
chassis.actionBuilder()
        .strafeTo(new Vector2d(24, 0))
        .build();
```

**电机配置：**
- `fL`: 左前电机
- `fR`: 右前电机
- `bL`: 左后电机
- `bR`: 右后电机

**参数说明：**
- `vx`: 前进速度（-1~1），正值前进
- `vy`: 横向速度（-1~1），正值左移
- `omega`: 角速度（-1~1），正值逆时针旋转

**速度参数配置：**
速度参数在 `HypParams` 中配置：
- `maxV`: 最大线速度（inch/s），默认 2.0
- `maxOmega`: 最大角速度（rad/s），默认 PI

### 2. ChassisTester

底盘测试 OpMode，用于手动测试底盘驱动功能和定位系统。

```java
@TeleOp(name = "ChassisTester", group = "Tests")
public class ChassisTester extends LinearOpMode { ... }
```

**操作方式：**
- 左摇杆：控制前后左右移动
  - 上/下：前进/后退
  - 左/右：左移/右移
- 右摇杆左右：控制旋转

**Telemetry 显示：**
- RR x, y, heading: RoadRunner 定位器的当前位姿
- RR vx, vy, omega: RoadRunner 速度估计
- Pinpoint vx, vy, omega: Pinpoint 传感器速度估计
- 四个电机的实际功率

---

## 典型用法

### 在 TeleOp 中使用

```java
@TeleOp(name = "MyTeleOp", group = "Main")
public class MyTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            double vx = -gamepad1.left_stick_y;
            double vy = -gamepad1.left_stick_x;
            double omega = -gamepad1.right_stick_x;

            chassis.update(vx, vy, omega);
            chassis.updatePoseEstimate();

            telemetry.addData("x", chassis.getPose().position.x);
            telemetry.addData("y", chassis.getPose().position.y);
            telemetry.update();
        }
    }
}
```

### 在 Autonomous 中使用

#### 手动控制方式

```java
// 前进
chassis.update(0.5, 0, 0);
sleep(1000);
chassis.stop();

// 左移
chassis.update(0, 0.5, 0);
sleep(1000);
chassis.stop();

// 旋转
chassis.update(0, 0, 0.5);
sleep(1000);
chassis.stop();
```

#### 轨迹规划方式（推荐）

```java
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Vector2d;

@Autonomous(name = "MyAuto", group = "Main")
public class MyAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Chassis chassis = new Chassis(hardwareMap);

        Action trajectory = chassis.actionBuilder()
                .strafeTo(new Vector2d(24, 0))
                .forward(12)
                .turn(Math.PI / 2)
                .build();

        waitForStart();

        trajectory.runBlocking();
    }
}
```

---

## RoadRunner 参数配置

所有底盘参数通过 `MecanumDrive.PARAMS` 静态类配置，可通过 FTC Dashboard 实时调整：

```java
// 驱动模型参数
MecanumDrive.PARAMS.inPerTick = 0.001;        // 每个编码器 tick 对应的英寸数
MecanumDrive.PARAMS.trackWidthTicks = 1000;   // 轮距（编码器 tick 单位）

// 电机前馈参数（需通过 TuningOpModes 校准）
MecanumDrive.PARAMS.kS = 0.1;
MecanumDrive.PARAMS.kV = 0.05;
MecanumDrive.PARAMS.kA = 0.01;

// 路径规划参数
MecanumDrive.PARAMS.maxWheelVel = 50;
MecanumDrive.PARAMS.maxProfileAccel = 50;

// 转向规划参数
MecanumDrive.PARAMS.maxAngVel = Math.PI;
MecanumDrive.PARAMS.maxAngAccel = Math.PI;

// 路径控制器增益（需通过 TuningOpModes 校准）
MecanumDrive.PARAMS.axialGain = 0.0;
MecanumDrive.PARAMS.lateralGain = 0.0;
MecanumDrive.PARAMS.headingGain = 0.0;
```

---

## 调优流程

1. 使用 `quickstart` 组中的调优 OpMode 进行参数校准：
   - `MecanumMotorDirectionDebugger`: 校准电机方向
   - `ForwardRampLogger`: 校准前进方向参数
   - `LateralRampLogger`: 校准横向移动参数
   - `AngularRampLogger`: 校准转向参数
   - `ManualFeedforwardTuner`: 校准电机前馈参数
   - `ManualFeedbackTuner`: 校准控制器增益

2. 将校准得到的参数填入 `MecanumDrive.PARAMS`

3. 使用 `SplineTest` 测试轨迹跟随效果

4. 使用 `LocalizationTest` 测试定位精度