import cv2
from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import logging

model = YOLO('yolo11x.pt')

#If you dont want the processing information printed to the terminal
logging.getLogger('ultralytics').setLevel(logging.WARNING)

tracker = DeepSort(max_age=30)

#cap = cv2.VideoCapture('/home/reed/Desktop/CodingProjects/Imaging/PeopleWalkingStock.mp4')
cap = cv2.VideoCapture('/home/reed/Desktop/CodingProjects/Imaging/StockFootageOfStreetCorner.mp4')
#cap = cv2.VideoCapture(0)

#fps = cap.get(cv2.CAP_PROP_FPS)
#delay = int((1000/fps) * (1/2))

selectedIdIndex = 0
activeIds = []

while(1):
    _, frame = cap.read()

    results = model(frame, show=False, classes=[0])

    detections = []
    if results[0].boxes is not None:
        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].cpu().item()
            detections.append(([x1, y1, x2 -x1, y2-y1], conf, 'person'))

    tracks = tracker.update_tracks(detections, frame = frame)

    for track in tracks:
        if not track.is_confirmed():
            continue
        track_id = track.track_id
        ltrb = track.to_ltrb()
        x1, y1, x2, y2 = map(int, ltrb)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID: {track_id}', (x1, y1 - 10), cv2.FONT_HERSHEY_DUPLEX, 0.6, (0, 255, 0), 2)

    activeIds = [t.track_id for t in tracks if t.is_confirmed()]

    if activeIds:
        selectedIdIndex %= len(activeIds)
        selectedId = activeIds[selectedIdIndex]
    else:
        selectedId = None

    if selectedId is not None:
        selectedTrack = next((t for t in tracks if t.track_id == selectedId), None)

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