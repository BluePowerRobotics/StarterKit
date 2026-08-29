package org.firstinspires.ftc.teamcode.Processors.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

/**
 * 色块定位器：内部封装摄像头与 ColorBlobLocatorProcessor，
 * 对外提供实时结果接口。
 *
 * 用法：
 *   ColorLocator locator = new ColorLocator(hardwareMap);
 *   while (opModeIsActive()) {
 *       if (locator.update()) {
 *           double x = locator.getCenterX();  // 归一化坐标
 *       }
 *   }
 */
public class ColorLocator {

    // 摄像头分辨率（已降采样，需与 VisionPortal 设置一致）
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // 摄像头与处理器
    private final List<ColorBlobLocatorProcessor> processors = new ArrayList<>();
    private VisionPortal portal;

    // 最新结果（调用 update() 后更新）
    private boolean targetFound = false;
    private double centerX = 0.0;   // 归一化 [-1, 1]，右为正
    private double centerY = 0.0;   // 归一化 [-1, 1]，上为正
    private double boxWidth = 0.0;
    private double boxHeight = 0.0;
    private double boxAngle = 0.0;
    private double radius = 0.0;

    public ColorLocator(HardwareMap hardwareMap) {
        this(hardwareMap, "Webcam 1");
    }

    public ColorLocator(HardwareMap hardwareMap, String webcamName) {
        // 构建各颜色处理器
        ColorBlobLocatorProcessor proc0 = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                                    new Scalar(0, 0, 0),
                                    new Scalar(255, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(5)
                .setErodeSize(0)
                .setDilateSize(0)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        processors.add(proc0);

        // 封装摄像头：把所有处理器挂到同一个 VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(proc0)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName));
        portal = builder.build();
    }

    /**
     * 读取最新一帧并计算目标色块，返回是否找到。
     * 拟合轮廓中心坐标会被归一化到 [-1, 1]，以右上为正：
     * x 向右增大、y 向上增大，画面中心为 (0, 0)。
     */
    public boolean update() {
        List<ColorBlobLocatorProcessor.Blob> allBlobs = new ArrayList<>();

        // 处理器 0：读取并按规则过滤
        List<ColorBlobLocatorProcessor.Blob> blobs0 = processors.get(0).getBlobs();
        allBlobs.addAll(blobs0);

        // 全局排序
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, allBlobs);

        if (allBlobs.isEmpty()) {
            targetFound = false;
            centerX = 0.0;
            centerY = 0.0;
            return false;
        }

        targetFound = true;
        ColorBlobLocatorProcessor.Blob first = allBlobs.get(0);
        Circle circle = first.getCircle();
        // 归一化到 [-1, 1]：x 向右为正、y 向上为正
        centerX = (circle.getX() / CAMERA_WIDTH) * 2.0 - 1.0;
        centerY = 1.0 - (circle.getY() / CAMERA_HEIGHT) * 2.0;
        radius = circle.getRadius();
        return true;
    }

    public boolean isTargetFound() { return targetFound; }
    public double getCenterX() { return centerX; }
    public double getCenterY() { return centerY; }
    public double getRadius() { return radius; }

    public void close() {
        if (portal != null) {
            portal.close();
        }
    }
}
