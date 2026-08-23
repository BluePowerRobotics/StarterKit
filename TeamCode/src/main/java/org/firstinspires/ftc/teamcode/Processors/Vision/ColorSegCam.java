package org.firstinspires.ftc.teamcode.Processors.Vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * 色块分割视觉任务封装：每帧实时返回"最大连通域重心"的归一化坐标（-1..1，画面中心为原点）。
 *
 * <p>说明：本项目（FTC SDK 11.0）未引入 EasyOpenCV，官方样例统一使用 {@link VisionPortal} +
 * {@link VisionProcessor} 驱动摄像头并把 OpenCV 帧（{@code org.opencv.core.Mat}）回调进来。
 * 因此提示词中的 "WebCam 硬件" 在此被抽象为 {@link VisionPortal}（由本类持有并封装），
 * 算法层保持不变，通过 {@link #processFrame(Mat, long)} 在每个相机回调线程的帧上执行。
 *
 * <p>线程模型：相机回调线程调用 {@link #processFrame} 跑完整算法链并发布结果；
 * 主程序循环线程通过 {@link #update()} / {@link #getNormalizedCentroid()} 仅读取最近一帧结果，
 * 通过各 setter 实时改参，改动自下一帧起生效。
 */
public class ColorSegCam implements VisionProcessor {

    /* ==================== 枚举定义 ==================== */

    /** 颜色空间，映射到 OpenCV 双向转换码（图像恒为 BGR，故需正/反向两套码）。 */
    public enum ColorSpace {
        RGB(Imgproc.COLOR_BGR2RGB, Imgproc.COLOR_RGB2BGR),
        LAB(Imgproc.COLOR_BGR2Lab, Imgproc.COLOR_Lab2BGR),
        HSV(Imgproc.COLOR_BGR2HSV, Imgproc.COLOR_HSV2BGR),
        /** 注意 HLS 通道序为 H,L,S 且 H 范围 0~180。 */
        HSL(Imgproc.COLOR_BGR2HLS, Imgproc.COLOR_HLS2BGR);

        final int toCode;   // BGR -> 本空间
        final int fromCode; // 本空间 -> BGR

        ColorSpace(int to, int from) {
            this.toCode = to;
            this.fromCode = from;
        }
    }

    /** 吸光模式：RATIO=比例/线性；INTENSITY=强度/非线性。 */
    public enum AbsorbMode { RATIO, INTENSITY }

    /** 形态学后处理操作。 */
    public enum MorphOp {
        NONE(-1),
        ERODE(Imgproc.MORPH_ERODE),
        DILATE(Imgproc.MORPH_DILATE),
        OPEN(Imgproc.MORPH_OPEN),
        CLOSE(Imgproc.MORPH_CLOSE);

        final int code;

        MorphOp(int code) {
            this.code = code;
        }
    }

    /** 形态学结构元素形状。 */
    public enum KernShape {
        RECT(Imgproc.MORPH_RECT),
        ELLIPSE(Imgproc.MORPH_ELLIPSE),
        CROSS(Imgproc.MORPH_CROSS);

        final int code;

        KernShape(int code) {
            this.code = code;
        }
    }

    /* ==================== 组合类型 ==================== */

    /** 三通道 double 值，分量顺序与该色空间的通道顺序一致。 */
    public static final class Vec3 {
        public final double x, y, z;

        public Vec3(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z;
        }
    }

    /** 单个二值化区间：指定色空间 + lower..upper（含端点），不同区间可各自使用不同色空间。 */
    public static final class Range {
        public final ColorSpace space;
        public final Vec3 lower, upper;

        public Range(ColorSpace space, Vec3 lower, Vec3 upper) {
            this.space = space;
            this.lower = lower;
            this.upper = upper;
        }
    }

    /**
     * 环境色：环境内固有色为中灰的物体，在当前光照下被观测到的颜色（中灰参考物的物体色）。
     * 注意它描述的是"物体呈现的颜色"而非光源本身，与光源色不同；
     * 用于把每帧图像还原到固有色，须携带所属色空间以便转换回 BGR 参与计算。
     */
    public static final class LightSource {
        public final ColorSpace space;
        public final Vec3 val;

        public LightSource(ColorSpace space, Vec3 val) {
            this.space = space;
            this.val = val;
        }
    }

    /** 相机视场角（度）：水平 + 竖直。不可变，便于线程安全地整体设置/读取。 */
    public static final class Fov {
        public final double horizontalDeg;
        public final double verticalDeg;

        public Fov(double horizontalDeg, double verticalDeg) {
            this.horizontalDeg = horizontalDeg;
            this.verticalDeg = verticalDeg;
        }
    }

    /* ==================== 可实时配置字段（volatile 保证跨线程可见性） ==================== */

    private volatile int blurK = 3;
    private volatile double blurSigma = 1.0;
    private volatile LightSource lightSource = new LightSource(ColorSpace.LAB, new Vec3(137, 128, 128));
    private volatile AbsorbMode absorbMode = AbsorbMode.RATIO;
    /** 多区间并集：任意一个区间内的像素即判为目标，需现场标定后调用 setColorRange 修正。 */
    private volatile Range[] ranges = {
            new Range(ColorSpace.HSL, new Vec3(0, 0, 0), new Vec3(180, 255, 255))
    };
    private volatile MorphOp morphOp = MorphOp.OPEN;
    private volatile KernShape kernShape = KernShape.ELLIPSE;
    private volatile int kernSize = 5;
    /** 相机视场角；null 表示未设置（此时方位角返回 null）。 */
    private volatile Fov fov = null;

    /* ==================== 输出状态字段（volatile 发布最近一帧结果） ==================== */

    public volatile boolean detected = false;
    public volatile double normX = Double.NaN;
    public volatile double normY = Double.NaN;
    public volatile double rawX = 0, rawY = 0, area = 0;
    public volatile long frameStamp = 0;
    public volatile Double bearing = null;   // 水平方位角（度），左为正
    public volatile Double elevation = null; // 竖直仰角（度），上为正

    /* ==================== 复用的 Mat 缓存（避免每帧 new 大量对象 / GC 抖动） ==================== */

    private Mat work;      // 高斯降噪输出
    private Mat restored;  // 还原固有色输出
    private Mat bin;       // 变换到某个区间色空间后的图
    private Mat mask;      // 二值化掩码（后同作形态学与连通域输入）
    private Mat labels, stats, centroids;
    private Mat tmpRangeMask; // 单个区间二值化的临时掩码
    private Mat kernel;    // 结构元素缓存，形状/尺寸变化时才重建
    private Mat publishedMask; // 发布给外部的二值图快照（CV_8UC1，maskLock 保护）
    private final Object maskLock = new Object();

    // 1x1 临时图：用于把单个环境色（中灰参考物观测色）从它的色空间转换回 BGR
    private final Mat envSrc = new Mat(1, 1, CvType.CV_8UC3);
    private final Mat envDst = new Mat(1, 1, CvType.CV_8UC3);
    private static final Scalar WHITE128 = new Scalar(128, 128, 128); // 白色参考值（128）
    private static final Scalar ZERO = new Scalar(0, 0, 0);

    // 预览层十字标，便于现场标定二值化区间
    private final Paint crossPaint = new Paint();

    // 内核缓存对应项（用于判断是否需要重建结构元素）
    private KernShape cachedKernShape;
    private int cachedKernSize = -1;

    // 封装的摄像头硬件（VisionPortal）
    private volatile VisionPortal portal;

    public ColorSegCam() {
        crossPaint.setColor(Color.GREEN);
        crossPaint.setStrokeWidth(2f);
    }

    /**
     * 一键创建并接好摄像头：内部构建 {@link VisionPortal} 并把本类作为处理器挂载。
     * 主程序只与本类交互。
     */
    public static ColorSegCam forWebcam(HardwareMap hardwareMap, String webcamName, int width, int height) {
        ColorSegCam cam = new ColorSegCam();
        cam.portal = new VisionPortal.Builder()
                .addProcessor(cam)
                .setCameraResolution(new android.util.Size(width, height))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName))
                .build();
        return cam;
    }

    /** 供主程序循环读取最近一帧结果；算法链已由相机回调线程在 processFrame 中完成。 */
    public boolean update() {
        return detected;
    }

    public boolean isDetected() {
        return detected;
    }

    /** 返回归一化坐标（画面中心为原点，范围 -1..1，normY 以向上为正）；未检出时返回 NaN。 */
    public Point getNormalizedCentroid() {
        return new Point(normX, normY);
    }

    /** 重心水平方位角（度），左为正；未设 FOV 或未检出时返回 null。 */
    public Double getBearing() {
        return bearing;
    }

    /** 重心竖直仰角（度），上为正；未设 FOV 或未检出时返回 null。 */
    public Double getElevation() {
        return elevation;
    }

    /**
     * 实时返回最近一帧形态学后处理完成的二值图（CV_8UC1：前景 255 / 背景 0）。
     * 返回的是线程安全的深拷贝，调用方使用完毕后需自行 release()；
     * 尚未产出任何帧时返回 null。
     */
    public Mat getMask() {
        synchronized (maskLock) {
            return publishedMask == null ? null : publishedMask.clone();
        }
    }

    /* ==================== 实时参数设置 ==================== */

    public void setBlur(int k, double sigma) {
        this.blurK = k;
        this.blurSigma = sigma;
    }

    public void setLightSource(ColorSpace space, Vec3 val) {
        this.lightSource = new LightSource(space, val);
    }

    public void setAbsorbMode(AbsorbMode m) {
        this.absorbMode = m;
    }

    /** 设置一个或多个二值化区间；任一区间命中即置 1（多区间取并集）。 */
    public void setColorRange(Range... rs) {
        this.ranges = rs.clone(); // 防御性拷贝，避免调用方后续修改数组影响内部状态
    }

    public void setMorph(MorphOp op, KernShape shape, int size) {
        this.morphOp = op;
        this.kernShape = shape;
        this.kernSize = size;
    }

    /** 设置相机视场角（度）。未设置前 getBearing()/getElevation() 返回 null。 */
    public void setFov(double horizontalDeg, double verticalDeg) {
        this.fov = new Fov(horizontalDeg, verticalDeg);
    }

    /* ==================== WebCam（VisionPortal）参数透传 ==================== */

    public void stopStreaming() {
        if (portal != null) portal.stopStreaming();
    }

    public void resumeStreaming() {
        if (portal != null) portal.resumeStreaming();
    }


    /** 释放摄像头与全部 Mat 缓存。 */
    public void close() {
        if (portal != null) {
            portal.close();
            portal = null;
        }
        releaseMats();
    }

    /* ==================== VisionProcessor 接口实现 ==================== */

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // 预留：如后续需要镜头内参可在此读取 calibration
        ensureAllocated(width, height);
    }

    /**
     * 相机回调线程：每帧跑完整算法链，并发布结果（volatile 字段）。
     * 顺序严格对应：高斯降噪 -> 还原固有色 -> 变换+二值化 -> 形态学 -> 最大连通域重心 -> 归一化。
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // 快照：一次性读取全部配置，保证本帧内参数一致，改动下一帧生效
        int k = blurK;
        double sigma = blurSigma;
        LightSource ls = lightSource;
        AbsorbMode am = absorbMode;
        Range[] rs = ranges;
        MorphOp mo = morphOp;
        KernShape ks = kernShape;
        int kSize = kernSize;
        Fov f = fov;

        ensureAllocated(frame.width(), frame.height());

        // 2. 高斯降噪
        Imgproc.GaussianBlur(frame, work, new org.opencv.core.Size(k, k), sigma);

        // 3. 还原固有色：先把环境色（中灰参考物观测色）从它的色空间转回 BGR
        envSrc.put(0, 0, ls.val.x, ls.val.y, ls.val.z);
        Imgproc.cvtColor(envSrc, envDst, ls.space.fromCode);
        double[] e = envDst.get(0, 0);
        Scalar envBgr = new Scalar(e[0], e[1], e[2]);

        if (am == AbsorbMode.RATIO) {
            // out = src * 128 / env
            Core.divide(work, envBgr, restored, 128.0);
        } else {
            // out = sat(src - env + 128)
            Core.subtract(work, envBgr, restored);
            Core.add(restored, WHITE128, restored);
        }

        // 4. 依次按各区间自身色空间变换并二值化，命中任一区间即置 1（多 mask 取并集）
        mask.setTo(ZERO);
        for (Range r : rs) {
            Imgproc.cvtColor(restored, bin, r.space.toCode);

            // HSV/HSL 的 H 通道环绕：下限>上限表示目标色跨过 0/180（如红色环绕），
            // 拆成 [lower.x, 180] 与 [0, upper.x] 两段取并集，其余通道仍取原区间
            if ((r.space == ColorSpace.HSV || r.space == ColorSpace.HSL) && r.lower.x > r.upper.x) {
                Core.inRange(bin,
                        new Scalar(r.lower.x, r.lower.y, r.lower.z),
                        new Scalar(180, r.upper.y, r.upper.z),
                        tmpRangeMask);
                Core.bitwise_or(mask, tmpRangeMask, mask);
                Core.inRange(bin,
                        new Scalar(0, r.lower.y, r.lower.z),
                        new Scalar(r.upper.x, r.upper.y, r.upper.z),
                        tmpRangeMask);
                Core.bitwise_or(mask, tmpRangeMask, mask);
            } else {
                Core.inRange(bin,
                        new Scalar(r.lower.x, r.lower.y, r.lower.z),
                        new Scalar(r.upper.x, r.upper.y, r.upper.z),
                        tmpRangeMask);
                Core.bitwise_or(mask, tmpRangeMask, mask);
            }
        }

        // 5. 形态学后处理
        if (mo != MorphOp.NONE) {
            Imgproc.morphologyEx(mask, mask, mo.code, getKernel(ks, kSize));
        }

        // 发布二值图快照：深拷贝到独立缓冲，供其他 CV 算法实时读取（避免与下一帧写入竞争）
        synchronized (maskLock) {
            mask.copyTo(publishedMask);
        }

        // 6. 最大连通域重心（遍历 CC_STAT_AREA 取面积最大者）
        int n = Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8, CvType.CV_32S);
        int best = -1;
        double bestArea = -1;
        for (int i = 1; i < n; i++) {
            double a = stats.get(i, Imgproc.CC_STAT_AREA)[0];
            if (a > bestArea) {
                bestArea = a;
                best = i;
            }
        }

        // 7. 归一化：画面中心为原点，范围 -1..1；normY 以向上为正（图像 y 向下，故取反）；未检出返回 NaN
        boolean det = best > 0;
        if (det) {
            double[] c = centroids.get(best, 0);
            rawX = c[0];
            rawY = c[1];
            area = bestArea;
            double halfW = frame.width() * 0.5;
            double halfH = frame.height() * 0.5;
            normX = (rawX - halfW) / halfW;
            normY = (halfH - rawY) / halfH;
        } else {
            rawX = rawY = area = 0;
            normX = normY = Double.NaN;
        }

        detected = det;
        frameStamp = captureTimeNanos;

        // 8. 由 FOV 与归一化坐标用三角公式算方位角：θ = atan(norm * tan(FOV/2))。
        //    normX 取负号使左为正；normY 已上为正，elevation 直接取 atan；未设 FOV 或未检出时为 null
        if (f != null && det) {
            double hHalfRad = Math.toRadians(f.horizontalDeg * 0.5);
            double vHalfRad = Math.toRadians(f.verticalDeg * 0.5);
            bearing = -Math.toDegrees(Math.atan(normX * Math.tan(hHalfRad)));
            elevation = Math.toDegrees(Math.atan(normY * Math.tan(vHalfRad)));
        } else {
            bearing = null;
            elevation = null;
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        // 便于现场标定：把检出重心画成交叉线
        if (!detected) return;
        float cx = (float) rawX * scaleBmpPxToCanvasPx;
        float cy = (float) rawY * scaleBmpPxToCanvasPx;
        canvas.drawLine(cx - 10f, cy, cx + 10f, cy, crossPaint);
        canvas.drawLine(cx, cy - 10f, cx, cy + 10f, crossPaint);
    }

    /* ==================== 内部工具 ==================== */

    /** 按帧尺寸一次性分配/复用 Mat 缓存（尺寸变化时才重建）。 */
    private void ensureAllocated(int w, int h) {
        if (work != null && work.cols() == w && work.rows() == h) return;

        releaseMats();
        work = new Mat(h, w, CvType.CV_8UC3);
        restored = new Mat(h, w, CvType.CV_8UC3);
        bin = new Mat(h, w, CvType.CV_8UC3);
        mask = new Mat(h, w, CvType.CV_8UC1);
        tmpRangeMask = new Mat(h, w, CvType.CV_8UC1);
        publishedMask = new Mat(h, w, CvType.CV_8UC1);
        labels = new Mat(h, w, CvType.CV_32S);
        stats = new Mat();
        centroids = new Mat();
    }

    /** 结构元素缓存：仅当形状/尺寸变化时才重建，避免每帧 new。 */
    private Mat getKernel(KernShape shape, int size) {
        if (kernel == null || cachedKernShape != shape || cachedKernSize != size) {
            if (kernel != null) kernel.release();
            kernel = Imgproc.getStructuringElement(shape.code, new org.opencv.core.Size(size, size));
            cachedKernShape = shape;
            cachedKernSize = size;
        }
        return kernel;
    }

    private void releaseMats() {
        release(work);
        release(restored);
        release(bin);
        release(mask);
        release(tmpRangeMask);
        release(labels);
        release(stats);
        release(centroids);
        release(kernel);
        release(publishedMask);
        work = restored = bin = mask = tmpRangeMask = labels = stats = centroids = kernel = publishedMask = null;
    }

    private void release(Mat m) {
        if (m != null) m.release();
    }

    /*
     * ==================== 用法示例（LinearOpMode 内） ====================
     *
     *   ColorSegCam cam = ColorSegCam.forWebcam(hardwareMap, "Webcam 1", 320, 240);
     *   // 现场标定目标色区间与环境色（中灰参考物观测色），可同时给多个色空间/区间（命中任一即检出，取并集）
     *   cam.setLightSource(ColorSegCam.ColorSpace.LAB, new ColorSegCam.Vec3(128, 0, 0));
     *   cam.setColorRange(
     *       new ColorSegCam.Range(ColorSegCam.ColorSpace.HSL, new ColorSegCam.Vec3(0, 0, 120), new ColorSegCam.Vec3(180, 255, 255)),
     *       new ColorSegCam.Range(ColorSegCam.ColorSpace.HSV, new ColorSegCam.Vec3(100, 80, 80), new ColorSegCam.Vec3(140, 255, 255)));
     *
     *   waitForStart();
     *   while (opModeIsActive()) {
     *       if (cam.update()) {
     *           double turn = Kp * (-cam.getNormalizedCentroid().x); // 目标偏左 -> 负 normX -> 反向转向
     *           // 将 turn 叠加到底盘旋转量
     *       }
     *       telemetry.addData("normX", cam.normX);
     *   }
     *   cam.close();
     */
}