import cv2
import os
import shutil  # 用于处理文件操作

# 创建输出目录结构
os.makedirs('processed', exist_ok=True)
for i in range(10):  # 创建0-9的数字文件夹
    os.makedirs(f'processed/{i}', exist_ok=True)


# 预处理函数
def preprocess_image(img_path, output_dir):
    img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"警告：无法读取 {img_path}，已跳过")
        return

    # 统一处理为28x28二值化图像
    img = cv2.resize(img, (28, 28))
    _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    return img

total_files = len([f for f in os.listdir('raw_data') if f.endswith(('.jpg','.png'))])
processed_count = 0

# 处理所有文件
for filename in os.listdir('raw_data'):
    if filename.endswith(('.jpg', '.png')):
        try:
            # 解析文件名格式：test_132_5.jpg → 数字类别是最后一个下划线后的数字
            digit_class = filename.split('_')[-1].split('.')[0]

            # 验证类别有效性（0-9）
            if not digit_class.isdigit() or int(digit_class) not in range(10):
                print(f"无效类别：{filename}，已跳过")
                continue

            # 预处理并保存
            img = preprocess_image(f'raw_data/{filename}', 'processed')
            if img is not None:
                cv2.imwrite(f'processed/{digit_class}/{filename}', img)

            processed_count += 1
            if processed_count % 1000 == 0:
                print(f"进度：{processed_count}/{total_files} ({(processed_count / total_files) * 100:.1f}%)")

        except Exception as e:
            print(f"处理 {filename} 时出错：{str(e)}")
            continue

print("预处理完成！")
print(f"各类别统计：")
for i in range(10):
    count = len(os.listdir(f'processed/{i}'))
    print(f"数字 {i}：{count} 张")