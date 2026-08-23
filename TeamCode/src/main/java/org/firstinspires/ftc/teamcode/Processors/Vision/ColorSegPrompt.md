请用 Java（Android/FTC 项目，依赖 OpenCV或EasyOpenCV）实现ColorSegCam类，
用于 FTC 主程序中的色块分割视觉任务。要求代码可直接编译，风格清晰，关键处有简短注释。

【一、总体目标】
ColorSegCam 封装一个WebCam硬件，实现传统计算机视觉中的"色块分割"算法：
每帧实时返回"最大连通域重心"在画面中的归一化坐标（范围 -1..1，画面中心为原点），
同时支持在运行过程中实时修改所有算法参数。

【二、ColorSegCam 类（算法封装层）】
持有一个WebCam硬件，主程序只调用本类。

1) 枚举定义
- ColorSpace { RGB, LAB, HSV, HSL }，内部映射到 OpenCV 转换码
  （注意：HSL 用 COLOR_BGR2HLS，通道序为 H,L,S 且 H 范围 0~180）
- AbsorbMode { RATIO, INTENSITY }   // RATIO=比例/正片叠底(线性)；INTENSITY=强度/线性加深(非线性)
- MorphOp     { NONE, ERODE, DILATE, OPEN, CLOSE }
- KernShape   { RECT, ELLIPSE, CROSS }

1) 组合类型
- Vec3：三通道 double 值
- Range：Vec3 lower + Vec3 upper（超矩形区间）
- LightSource：ColorSpace + Vec3（光源色必须携带所属色空间）

1) 可实时配置字段（全部线程安全，默认值如下）
- blurK=3, blurSigma=1.0                        降噪
- lightSource：默认 Lab，值 (0,0,0)            环境光源色
- absorbMode：默认 RATIO                        吸光模式
- binSpace：默认 HSL                            二值化色空间
- binRange：Range（默认给占位，需现场标定）     二值化区间
- morphOp：默认 OPEN                            后处理操作
- kernShape：默认 ELLIPSE                       后处理核形状
- kernSize：默认 5                              后处理核大小

1) 核心方法
- ColorSegCam(WebCam cam) 构造
- boolean update()：取一帧并跑完整算法链，返回是否检出目标
- Point2d getNormalizedCentroid()：返回归一化坐标
- boolean isDetected()
- setBlur(k, sigma) / setLightSource(space, val) / setAbsorbMode(m)
  / setBinSpace(s) / setColorRange(lower, upper) / setMorph(op, shape, size)
- WebCam 参数透传

1) 输出状态字段
- detected(boolean)、normX/normY(double)、rawX/rawY/area、frameStamp

【三、update() 内部算法流程（严格按此顺序）】
1 取帧（默认与 WebCam 设定分辨率相同，转灰度 Mat 复用缓存，不要每帧 new 大量对象）
2 高斯降噪：Imgproc.GaussianBlur(src, dst, new Size(blurK,blurK), blurSigma)
3 还原固有色：按 lightSource 的色空间与 absorbMode 计算
   RATIO    ：out = src * 128 / env
   INTENSITY：out = sat(src - env + 128)
   （env 先转换到与当前图像一致的色空间；128 为白色参考）
4 变换到 binSpace 并二值化：Imgproc.cvtColor + Core.inRange(src, lower, upper, mask)
5 后处理（morphOp != NONE 时）：Imgproc.getStructuringElement(shape, new Size(k,k))
   再 Imgproc.morphologyEx(mask, mask, op, kernel)
6 最大连通域重心：Imgproc.connectedComponentsWithStats(mask, labels, stats, centroids, 8)，
   遍历 stats 的 CC_STAT_AREA 取面积最大连通域，读取 centroids 中对应质心
7 归一化：normX=(cx-W/2)/(W/2)，normY=(cy-H/2)/(H/2)；未检出时 detected=false 且坐标返回 NaN

【四、实时配置与线程安全】
- 相机回调线程产帧，主程序循环线程读结果/改参数。
- 配置字段用 volatile 或 AtomicReference；update() 开头一次性读取全部配置为局部快照，本帧内一致，改动下一帧生效。
- 输出字段用 volatile 发布最近一帧结果。

【五、代码约束】
- 复用 Mat 缓存避免 GC 抖动；使用 try/finally 释放临时 Mat（release()）。
- OpenCV 转换码映射正确（BGR<->RGB、Lab、HSV、HLS）。
- 关键处加简短中文注释（说明"为什么"而非"做了什么"）。
- 提供 main 或示例用法片段，演示 OpMode 中如何 update() 后用 normX 做转向控制（如 turn = Kp * (-normX)）。

【六、验收标准】
- 能编译通过；逻辑步骤与上面流程一一对应。
- 参数可在运行中实时修改并生效。
- 归一化坐标规则与"画面中心为原点、范围 -1..1"一致，未检出返回 NaN 且带 detected 标志。

若你对语言/框架有不同要求可说明，否则按 Java + EasyOpenCV + OpenCV 实现。