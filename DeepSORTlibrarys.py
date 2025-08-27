import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import logging

model = YOLO('yolo11x.pt')

#If you dont want the processing information printed to the terminal
logging.getLogger('ultralytics').setLevel(logging.WARNING)

tracker = DeepSort(max_age=30)

#You can put any video file as long as it has people in it then it will be able to track them.
cap = cv2.VideoCapture('/path/to/videoFile')


selectedIdIndex = 0
activeIds = []

while(1):
    _, frame = cap.read()

    results = model(frame, show=False, classes=[0])

#get your people detectons
    detections = []
    if results[0].boxes is not None:
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().item()
            detections.append(([x1, y1, x2 -x1, y2-y1], conf, 'person'))
    tracks = tracker.update_tracks(detections, frame = frame)
    
#For each person add a rectangle around them
    for track in tracks:
        if not track.is_confirmed():
            continue
        track_id = track.track_id
        ltrb = track.to_ltrb()
        x1, y1, x2, y2 = map(int, ltrb)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 2)

#Give every person a tracking id 
    activeIds = [t.track_id for t in tracks if t.is_confirmed()]

#Pick a person by the order they are in the list of people and when there is no current person being tracked move onto someone else
    if activeIds:
        selectedIdIndex %= len(activeIds)
        selectedId = activeIds[selectedIdIndex]
    else:
        selectedId = None

    if selectedId is not None:
        selectedTrack = next((t for t in tracks if t.track_id == selectedId), None)

#Put a circle around your selected person
        if selectedTrack:
            x1, y1, x2, y2 = selectedTrack.to_ltrb()
            centerX = int((x2 + x1) / 2)
            centerY = int((y2 + y1) / 2)
            cv2.circle(frame, (centerX, centerY), 50, (0, 255, 255), 3)
    print("Current ID: " + str(selectedIdIndex))

    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    elif cv2.waitKey(1) & 0xFF == 32 and activeIds:
        selectedIdIndex = (selectedIdIndex + 1) % len(activeIds)
cap.release()
cv2.destroyAllWindows()
