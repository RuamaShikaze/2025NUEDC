import os
import random
import shutil

# 配置参数
TOTAL_IMAGES = 500  # 总图片数（每类200张）
SOURCE_DIR = 'processed'  # 原始数据路径
TARGET_DIR = 'mini_dataset'  # 精简数据集路径

# 创建目标目录
os.makedirs(TARGET_DIR, exist_ok=True)
for i in range(10):
    os.makedirs(f'{TARGET_DIR}/{i}', exist_ok=True)

# 随机选择图片
for digit in range(10):
    src_dir = f'{SOURCE_DIR}/{digit}'
    all_files = [f for f in os.listdir(src_dir) if f.endswith(('.jpg', '.png'))]
    selected = random.sample(all_files, min(200, len(all_files)))

    for filename in selected:
        shutil.copy(f'{src_dir}/{filename}', f'{TARGET_DIR}/{digit}/{filename}')

print(f"数据集精简完成！共{TOTAL_IMAGES}张图片保存在 {TARGET_DIR}")