package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Processors.Vision.ColorSegCam;
import org.opencv.core.Point;

/**
 * ColorSegCam 调试 OpMode：创建色块分割对象，并在 Driver Station 上实时显示
 * 最大连通域重心的归一化坐标（-1..1，画面中心为原点）与原始像素坐标/面积。
 *
 * <p>用法：Robot Controller 配置中需存在一个名为 "Webcam 1" 的摄像头；
 * 运行前可先按需要标定 {@code setColorRange(...)} / {@code setLightSource(...)}。
 */
@TeleOp(name = "ColorSeg Test", group = "Vision")
public class ColorSegTest extends LinearOpMode {

    // 摄像头配置名需与 Robot Controller 中的设备名一致
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int CAM_WIDTH = 320;
    private static final int CAM_HEIGHT = 240;

    @Override
    public void runOpMode() {
        // 创建并接好色块分割对象（内部封装 VisionPortal 摄像头）
        ColorSegCam cam = ColorSegCam.forWebcam(hardwareMap, WEBCAM_NAME, CAM_WIDTH, CAM_HEIGHT);

       ColorSegCam cam = ColorSegCam.forWebcam(hardwareMap, "Webcam 1", 1280, 720);

    // 降噪
    cam.setBlur(1, 0.2);

    // 环境光源（LAB 空间）
    cam.setLightSource(ColorSegCam.ColorSpace.LAB, new ColorSegCam.Vec3(242, 128, 127));

    // 吸光模式
    cam.setAbsorbMode(ColorSegCam.AbsorbMode.RATIO);

    // 二值化范围（多组取并集）
    cam.setColorRange(new ColorSegCam.Range(ColorSegCam.ColorSpace.HSL, new ColorSegCam.Vec3(171, 0, 23), new ColorSegCam.Vec3(12, 238, 77)));

    // 后处理
    cam.setMorph(ColorSegCam.MorphOp.OPEN, ColorSegCam.KernShape.ELLIPSE, 7);   

        telemetry.addData("Status", "Initialized, waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean detected = cam.update();
            Point norm = cam.getNormalizedCentroid();

            telemetry.addData("Detected", detected);
            if (detected) {
                telemetry.addData("Centroid (norm)", "(%.2f, %.2f)", norm.x, norm.y);
                telemetry.addData("Centroid (raw)", "(%.1f, %.1f)", cam.rawX, cam.rawY);
                telemetry.addData("Area", "%.0f px", cam.area);
            } else {
                telemetry.addData("Centroid (norm)", "(NaN, NaN)");
            }
            telemetry.update();
        }

        cam.close();
    }
}