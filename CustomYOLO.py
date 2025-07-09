from ultralytics import YOLO
import cv2

# Load your custom YOLO model (adjust path if needed)
#model = YOLO("ZYOLO.pt")  # e.g., "best.pt" or "runs/detect/train/weights/best.pt"
model = YOLO("ZYOLO.pt")  
model.to('cuda')


cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) 

while True:
    ret, frame = cap.read() # Read a frame from the webcam
    if not ret: # If no frame is captured, break the loop
        break

    results = model(frame) 
    annotated_frame = results[0].plot()  

    # Display the results
    cv2.imshow("YOLO Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('x'):  
        break

cap.release()  # Release the webcam
cv2.destroyAllWindows()  # Close all OpenCV windows