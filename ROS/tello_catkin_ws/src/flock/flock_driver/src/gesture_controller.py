'''
This script provides a class in order to connect the built-camera of the computer with a .h5 Keras model for classification. This model 
analyses the image and classifies hand gestures, providing a way to controll the Tello drone.
The GestureController calls for basic movement functions of the Tello...
'''

import cv2
from keras.models import load_model
import numpy as np

class GestureController:
    def __init__(self, model_path, class_labels):
        self.model = load_model(model_path)
        self.class_labels = class_labels
        self.cap = cv2.VideoCapture(0)

    def preprocess_frame(self, frame):
        # Check preprocessing steps
        frame = cv2.resize(frame, (224, 224))
        frame = frame / 255.0
        frame = np.expand_dims(frame, axis=0)
        return frame

    def recognize_class(self, frame):
        frame = self.preprocess_frame(frame)
        predictions = self.model.predict(frame)
        predicted_class = np.argmax(predictions)
        return self.class_labels[predicted_class]

    def send_command_to_robot(self, class_label):
        # Implement your robot control logic here
        # This is a placeholder; replace with actual commands
        print(f"Sending command to robot based on class: {class_label}")

    def run(self):
        while True:
            ret, frame = self.cap.read()
            
            if not ret:
                print("Error: Could not capture a frame.")
                break

            class_label = self.recognize_class(frame)
            print(f"Recognized class: {class_label}")

            self.send_command_to_robot(class_label)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    model_path = 'your_model.h5'
    class_labels = ['class1', 'class2', 'class3', 'class4', 'class5', 'class6', 'class7', 'class8']
    
    controller = GestureController(model_path, class_labels)
    controller.run()
