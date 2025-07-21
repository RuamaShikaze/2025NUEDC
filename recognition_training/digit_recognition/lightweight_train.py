import tensorflow as tf
from tensorflow.keras import layers, callbacks
import numpy as np
import cv2
import os

# 超参数配置
IMG_SIZE = 28
BATCH_SIZE = 32
EPOCHS = 10

# 1. 加载精简数据集
def load_data(data_dir='mini_dataset'):
    images, labels = [], []
    for digit in range(10):
        digit_dir = os.path.join(data_dir, str(digit))
        for filename in os.listdir(digit_dir)[:200]:  # 确保每类不超过200张
            img = cv2.imread(os.path.join(digit_dir, filename), cv2.IMREAD_GRAYSCALE)
            if img is not None:
                images.append(img.astype('float32') / 255.0)
                labels.append(digit)
    return np.array(images).reshape(-1, IMG_SIZE, IMG_SIZE, 1), np.array(labels)

print("加载数据...")
train_images, train_labels = load_data()

# 2. 定义轻量化模型
model = tf.keras.Sequential([
    layers.Conv2D(4, (3,3), activation='relu', input_shape=(28,28,1)),
    layers.MaxPooling2D((2,2)),
    layers.Flatten(),
    layers.Dense(16, activation='relu'),
    layers.Dense(10)
])

# 3. 编译与训练
model.compile(optimizer='adam',
              loss=tf.keras.losses.SparseCategoricalCrossentropy(from_logits=True),
              metrics=['accuracy'])

# 添加早停法防止过拟合
early_stop = callbacks.EarlyStopping(monitor='loss', patience=3)

print("开始训练...")
history = model.fit(train_images, train_labels,
                    batch_size=BATCH_SIZE,
                    epochs=EPOCHS,
                    callbacks=[early_stop],
                    validation_split=0.2)

# 4. 保存模型
model.save('mini_digit_model.h5')

# 定义代表性数据集生成器
def representative_dataset():
    for image in train_images[:100]:  # 使用前100张图像作为代表性数据集
        image = np.expand_dims(image, axis=0).astype(np.float32)
        yield [image]

# 量化模型为int8（关键步骤！）
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
# 确保输入和输出也被量化为int8
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

tflite_model = converter.convert()

with open('mini_digit_model_int8.tflite', 'wb') as f:
    f.write(tflite_model)

print("训练完成！int8模型大小:", len(tflite_model)/1024, "KB")