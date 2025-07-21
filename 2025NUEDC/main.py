import sensor
import time
import tf
import image
import math
import ml
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((240, 240))
sensor.skip_frames(time=1000)
model_path = "mini_digit_model_int8.tflite"
labels = ['0','1','2','3','4','5','6','7','8','9']
min_confidence = 0.6
try:
	net = ml.Model(model_path, load_to_fb=False)
	print("模型加载成功！输入尺寸:", net.input_shape())
except Exception as e:
	print("模型加载失败:", e)
	raise
def digit_post_process(img, outputs):
	detections = []
	output_data = outputs[0]
	for i in range(len(labels)):
		confidence = output_data[i]
		if confidence > min_confidence:
			x, y = 120, 120
			w, h = 40, 40
			detections.append({
				'x': x,
				'y': y,
				'w': w,
				'h': h,
				'label': labels[i],
				'confidence': confidence
			})
	return detections
clock = time.clock()
while True:
	clock.tick()
	img = sensor.snapshot()
	try:
		roi = (80, 80, 80, 80)
		img.draw_rectangle(roi, color=255)
		patch = img.copy(roi=roi)
		patch = patch.resize(28, 28)
		patch = patch.to_grayscale()
		tf_input = tf.reshape(patch, (28,28,1))
		outputs = net.predict(tf_input)
		detections = digit_post_process(img, outputs)
		for d in detections:
			x, y, w = d['x'], d['y'], d['w']
			label = d['label']
			confidence = d['confidence']
			img.draw_rectangle((x-w//2, y-w//2, w, w), color=255)
			img.draw_string(x+10, y-10, f"{label}:{confidence:.2f}", color=255)
			print(f"检测到数字: {label} 置信度: {confidence:.2f}")
	except Exception as e:
		print("推理出错:", e)
	img.draw_string(5, 5, f"FPS:{clock.fps():.1f}", color=255)