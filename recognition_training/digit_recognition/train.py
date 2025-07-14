import tensorflow as tf
from tensorflow.keras import layers
import numpy as np
import cv2
import os


# 1. 加载本地数据集
def load_local_data(data_dir='processed'):
    images = []
    labels = []

    for label in range(10):  # 遍历0-9数字文件夹
        label_dir = os.path.join(data_dir, str(label))
        for filename in os.listdir(label_dir):
            if filename.endswith(('.jpg', '.png')):
                img = cv2.imread(os.path.join(label_dir, filename), cv2.IMREAD_GRAYSCALE)
                if img is not None:
                    img = img.astype('float32') / 255.0
                    images.append(img)
                    labels.append(label)

    # 转换为numpy数组并调整形状
    images = np.array(images).reshape(-1, 28, 28, 1)
    labels = np.array(labels)
    return images, labels


print("正在加载本地数据集...")
train_images, train_labels = load_local_data()

# 2. 定义模型
model = tf.keras.Sequential([
    layers.Conv2D(16, (3, 3), activation='relu', input_shape=(28, 28, 1)),
    layers.MaxPooling2D((2, 2)),
    layers.Flatten(),
    layers.Dense(64, activation='relu'),
    layers.Dense(10)
])

# 3. 编译和训练
model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

print("开始训练...")
model.fit(train_images, train_labels, epochs=5)

# 4. 保存模型
model.save('digit_model.h5')

# 转换为TFLite格式
converter = tf.lite.TFLiteConverter.from_keras_model(model)
tflite_model = converter.convert()
with open('digit_model.tflite', 'wb') as f:
    f.write(tflite_model)

print("训练完成！模型已保存为 digit_model.tflite")