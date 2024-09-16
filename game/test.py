import cv2
from ultralytics import YOLO

# Load a pretrained YOLOv8n model
model = YOLO("best (3).pt")

# Open a camera feed (0 is typically the default camera)
cap = cv2.VideoCapture(0)

# Set frame size (optional)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)

# Loop to read and process each frame
while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    # Run inference on the frame
    results = model.predict(frame, imgsz=640, conf=0.5)

    # Visualize the results on the frame
    annotated_frame = results[0].plot()  # YOLOv8 plotting method for visualizing results

    # Display the frame with YOLO predictions
    cv2.imshow("YOLOv8 Inference", annotated_frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close windows
cap.release()
cv2.destroyAllWindows()