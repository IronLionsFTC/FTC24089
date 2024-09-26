# if running make sure to cop "pip install tensorflor"  # TensorFlow is required for Keras to work
from keras.models import load_model
import cv2
import numpy as np

np.set_printoptions(suppress=True)
model = load_model("laptop_test_model.h5", compile=False)
class_names = [x.strip("\n") for x in open("labels.txt", "r").readlines()]

# 1 is for the USB camera - will not work without it.
camera = cv2.VideoCapture(1)

while True:
    ret, image = camera.read()
    # Standard OPEN-CV size.
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imshow("Harrison_test", image)
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
    image = (image / 127.5) - 1
    prediction = model.predict(image, verbose=0)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]
    keyboard_input = cv2.waitKey(1)

    print(f"DETECTED: {class_name} | {confidence_score}", end="\r")

    # 27 is the ASCII for escape
    if keyboard_input == 27:
        break

camera.release()
cv2.destroyAllWindows()
