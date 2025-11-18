import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

def classify_hand(hand_landmarks, image):
    """
    Classify the hand as 'OPEN PALM' or 'FIST'
    based on how many fingers are extended.
    """
    image_height, image_width, _ = image.shape
    landmarks = hand_landmarks.landmark

    # Helper to convert normalized coords -> pixels
    def to_pixel(lm):
        return int(lm.x * image_width), int(lm.y * image_height)

    # Finger indices (MediaPipe Hands):
    # Thumb: 4 (tip), 3 (ip), 2 (mcp)
    # Index: 8 (tip), 6 (pip)
    # Middle: 12 (tip), 10 (pip)
    # Ring: 16 (tip), 14 (pip)
    # Pinky: 20 (tip), 18 (pip)

    # For simplicity, we check if fingers (except thumb) are extended
    # by comparing tip.y and pip.y: if tip is above (smaller y) than pip, the finger is extended.
    finger_tips = [8, 12, 16, 20]
    finger_pips = [6, 10, 14, 18]

    extended_fingers = 0

    for tip_idx, pip_idx in zip(finger_tips, finger_pips):
        _, tip_y = to_pixel(landmarks[tip_idx])
        _, pip_y = to_pixel(landmarks[pip_idx])

        if tip_y < pip_y:  # tip is higher than pip → finger extended
            extended_fingers += 1

    # Optional: use thumb as extra info (simple check: tip x vs mcp x)
    # This works decently when the palm faces the camera.
    wrist_x, _ = to_pixel(landmarks[0])
    thumb_tip_x, _ = to_pixel(landmarks[4])
    thumb_mcp_x, _ = to_pixel(landmarks[2])

    thumb_extended = 0
    # Right hand vs left hand heuristic
    if thumb_tip_x < thumb_mcp_x and thumb_mcp_x > wrist_x:
        thumb_extended = 1
    elif thumb_tip_x > thumb_mcp_x and thumb_mcp_x < wrist_x:
        thumb_extended = 1

    total_extended = extended_fingers + thumb_extended

    # Very simple rules:
    # - Open palm: most fingers extended
    # - Fist: almost no fingers extended
    if total_extended >= 4:
        return "FIST"
    elif total_extended <= 1:
        return "Open Palm"
    else:
        return "UNKNOWN"

def main():
    CAMERA_URL = "http://192.168.1.2/mjpg/video.mjpg"
    cap = cv2.VideoCapture(CAMERA_URL)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    with mp_hands.Hands(
        static_image_mode=False,
        max_num_hands=1,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    ) as hands:

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            # Flip horizontally for a mirror-like view
            frame = cv2.flip(frame, 1)

            # Convert BGR (OpenCV) to RGB (MediaPipe)
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Process frame with MediaPipe
            results = hands.process(rgb_frame)

            gesture_label = ""

            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    # Draw landmarks on the frame
                    mp_drawing.draw_landmarks(
                        frame,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS
                    )

                    # Classify gesture
                    gesture_label = classify_hand(hand_landmarks, frame)

                    # Only handle first hand (we set max_num_hands=1 anyway)
                    break

            # Show gesture label on screen
            if gesture_label:
                cv2.putText(
                    frame,
                    gesture_label,
                    (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.5,
                    (0, 255, 0),
                    3,
                    cv2.LINE_AA
                )

            cv2.imshow("Hand Gesture: Fist vs Open Palm", frame)

            # Press 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
