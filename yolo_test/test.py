import cv2

from ultralytics import YOLO

# Load the YOLO model
model = YOLO("yolo11n.pt")

# Open the video file
video_path = "path/to/your/video/file.mp4"
cap = cv2.VideoCapture(0) # 웹캠 - 0번째

# Loop through the video frames
while cap.isOpened(): # 웹캠이 열리는 동안안
    # Read a frame from the video
    success, frame = cap.read()

    if success:
        # Run YOLO inference on the frame
        results = model(frame)

        for r in results:
            for box in r.boxes:
                if int(box.cls[0]) == 0: # 0 (person) 이면
                    print("hello!")

        # Visualize the results on the frame
        annotated_frame = results[0].plot()
        
        # Display the annotated frame
        cv2.imshow("YOLO Inference", annotated_frame)

        # Break the loop if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    else:
        # Break the loop if the end of the video is reached
        break

# Release the video capture object and close the display window
cap.release()

cv2.destroyAllWindows()
