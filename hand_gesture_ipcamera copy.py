import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GesturePublisher(Node):
    def __init__(self):
        super().__init__("gesture_publisher")
        self.pub = self.create_publisher(String, "hand_gesture", 10)

    def publish_gesture(self, gesture_label):
        msg = String()
        msg.data = gesture_label
        self.pub.publish(msg)


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

def classify_hand(hand_landmarks, image):
    """
    Orientation-invariant classification:
    - Compute how "open" the hand is based on distances
      from wrist to fingertips, normalized by palm size.
    """

    landmarks = hand_landmarks.landmark

    # Convert a landmark to a 3D vector (normalized coords)
    def v(idx):
        lm = landmarks[idx]
        return np.array([lm.x, lm.y, lm.z], dtype=float)

    wrist = v(0)

    # Fingertips: thumb, index, middle, ring, pinky
    tip_indices = [4, 8, 12, 16, 20]
    # MCP joints (base of each finger)
    mcp_indices = [2, 5, 9, 13, 17]

    tip_dists = [np.linalg.norm(v(i) - wrist) for i in tip_indices]
    mcp_dists = [np.linalg.norm(v(i) - wrist) for i in mcp_indices]

    # Palm size ~ average wrist→MCP distance
    palm_size = np.mean(mcp_dists)

    # Avoid division by zero
    if palm_size < 1e-6:
        return "UNKNOWN"

    # "Openness" = how far the tips are compared to the palm size
    openness = np.mean(tip_dists) / palm_size

    # Tune these thresholds if needed
    if openness > 1.8:
        return "OPEN PALM"
    elif openness < 1.3:
        return "FIST"
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
                    gesture_label = classify_hand(...)
                    if gesture_label is not None:
                        gesture_publisher_node.publish_gesture(gesture_label)

                    

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
