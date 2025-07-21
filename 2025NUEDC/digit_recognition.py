import sensor
import image
import time
import ml

# 摄像头初始化
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((200, 200))  # 使用较小窗口减少内存消耗
sensor.skip_frames(time=2000)

# 加载模型和归一化器
model = ml.Model("trained.tflite", load_to_fb=True)
norm = ml.Normalization(scale=(0, 1.0))

# 预分配内存
resized_img = image.Image(28, 28, sensor.GRAYSCALE, copy_to_fb=True)
thresholded_img = image.Image(200, 200, sensor.GRAYSCALE)

# 参数配置 - 针对白底黑字优化
MIN_AREA = 300       # 最小区域面积
MIN_WIDTH = 15       # 最小宽度
MIN_HEIGHT = 25      # 最小高度
EXPAND_PIXELS = 3    # 边界框扩展像素
ADAPTIVE_THRESHOLD = True  # 启用自适应阈值

clock = time.clock()

def get_adaptive_threshold(img, roi=None):
    """计算自适应阈值"""
    if roi:
        hist = img.get_histogram(roi=roi)
    else:
        hist = img.get_histogram()

    # 获取直方图的谷底作为阈值
    return hist.get_threshold().value()

def detect_digits(img):
    """检测白底黑字的数字区域，增强中心区域检测"""
    # 计算全局阈值（用于边缘区域）
    if ADAPTIVE_THRESHOLD:
        global_threshold = get_adaptive_threshold(img)
        # 调整阈值范围
        THRESHOLD = (0, min(70, global_threshold))
    else:
        THRESHOLD = (0, 70)

    # 复制图像
    thresholded_img.replace(img)

    # 对中心区域应用特殊处理（增强对比度）
    center_roi = (50, 50, 100, 100)  # 中心100x100区域

    # 计算中心区域的局部阈值
    if ADAPTIVE_THRESHOLD:
        local_threshold = get_adaptive_threshold(img, center_roi)
        # 通常中心区域需要更严格的阈值
        center_threshold = (0, min(60, local_threshold))
    else:
        center_threshold = (0, 65)

    # 分别处理边缘和中心区域
    thresholded_img.binary([THRESHOLD], invert=True)  # 全局反相二值化

    # 增强中心区域处理
    center_img = img.copy(roi=center_roi)
    center_binary = center_img.binary([center_threshold], invert=True)
    thresholded_img.draw_image(center_binary, 50, 50)
    del center_img, center_binary

    # 轻微膨胀以连接数字笔画
    thresholded_img.dilate(1)

    # 查找白色区域（即原图像中的黑色数字）
    blobs = thresholded_img.find_blobs(
        [(100, 255)],                 # 查找白色区域
        pixels_threshold=MIN_AREA,    # 最小像素数
        area_threshold=MIN_AREA,      # 最小区域面积
        merge=True                    # 合并相邻区域
    )

    digit_boxes = []
    for blob in blobs:
        x, y, w, h = blob.rect()

        # 过滤不符合条件的区域
        if w >= MIN_WIDTH and h >= MIN_HEIGHT and h > w * 1.2:
            # 扩展边界框以确保包含完整数字
            expand_x = min(EXPAND_PIXELS, x, img.width()-x-w)
            expand_y = min(EXPAND_PIXELS, y, img.height()-y-h)
            digit_boxes.append((x-expand_x, y-expand_y, w+2*expand_x, h+2*expand_y))

    return digit_boxes

def recognize_digit(img, roi):
    """识别单个数字"""
    global resized_img

    # 提取数字区域并调整大小
    digit_img = img.copy(roi=roi)

    # 调整为28x28大小（模型输入要求）
    resized_img.clear()
    resized_img.draw_image(digit_img, 0, 0,
                          x_scale=28/digit_img.width(),
                          y_scale=28/digit_img.height())

    # 模型预测
    result = model.predict([norm(resized_img)])[0].flatten().tolist()
    digit = result.index(max(result))
    confidence = max(result)

    # 释放临时图像内存
    del digit_img
    return digit, confidence

while True:
    clock.tick()

    # 捕获图像
    img = sensor.snapshot()

    # 检测数字位置
    digit_boxes = detect_digits(img)

    # 识别每个数字并标记
    for box in digit_boxes:
        digit, conf = recognize_digit(img, box)

        # 在原图上绘制边界框和识别结果
        img.draw_rectangle(box, color=127)  # 绘制矩形框
        img.draw_string(box[0], box[1]-15, f"{digit}({conf:.1f})", color=255)  # 绘制识别结果

    # 输出帧率和检测数量
    print(f"检测到 {len(digit_boxes)} 个数字 | FPS: {clock.fps():.1f}")

    # 强制垃圾回收
    import gc
    gc.collect()
