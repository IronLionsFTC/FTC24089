import tensorflow as tf
import sys

filename = sys.argv[1]

model = tf.keras.models.load_model(filename)
converter = tf.lite.TFLiteConverter.from_saved_model(filename)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()
with open('quantized_model.tflite', 'wb') as f:
    f.write(tflite_model)
