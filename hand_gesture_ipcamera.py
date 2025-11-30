import cv2
import mediapipe as mp
import numpy as np


mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils


class HandGestureDetector:
    """
    Open the camera once and reuse it in your main loop.

    Usage:

        from hand_gesture_ipcamera import HandGestureDetector

        detector = HandGestureDetector(
            camera_url="http://192.168.1.2/mjpg/video.mjpg",
            debug=False,         # True if you want a debug window
            x_range_m=3.6,       # physical width in meters
            y_range_m=2.4,       # physical height in meters
        )

        for ... in main_loop:
            gesture, loc_m, loc_px = detector.read_gesture()
            # gesture: "OPEN PALM" / "FIST" / "UNKNOWN" / None
            # loc_m:   (x_m, y_m) in meters from bottom-left, or None
            # loc_px:  (cx_px, cy_px) in pixels from top-left, or None

        detector.release()
    """

    def __init__(
        self,
        camera_url: str = "http://192.168.1.2/mjpg/video.mjpg",
        debug: bool = False,
        x_range_m: float = 2.4,
        y_range_m: float = 3.6,
    ):
        self.camera_url = camera_url
        self.debug = debug
        self.x_range_m = x_range_m
        self.y_range_m = y_range_m

        # Open camera once
        self.cap = cv2.VideoCapture(self.camera_url)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera stream at {self.camera_url}")

        # Create one persistent MediaPipe Hands instance
        self.hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.3,
            min_tracking_confidence=0.4,
        )

    # -------------------- internal helpers --------------------

    def _classify_hand(self, hand_landmarks) -> str:
        """
        Classify a single hand as OPEN PALM, FIST, or UNKNOWN
        based on how "open" the hand is.
        """
        landmarks = hand_landmarks.landmark

        wrist_idx = 0
        fingertip_ids = [4, 8, 12, 16, 20]
        mcp_ids = [2, 5, 9, 13, 17]

        wrist = np.array(
            [
                landmarks[wrist_idx].x,
                landmarks[wrist_idx].y,
                landmarks[wrist_idx].z,
            ],
            dtype=float,
        )

        fingertip_dists = []
        for idx in fingertip_ids:
            tip = np.array(
                [landmarks[idx].x, landmarks[idx].y, landmarks[idx].z],
                dtype=float,
            )
            fingertip_dists.append(np.linalg.norm(tip - wrist))

        mcp_dists = []
        for idx in mcp_ids:
            base = np.array(
                [landmarks[idx].x, landmarks[idx].y, landmarks[idx].z],
                dtype=float,
            )
            mcp_dists.append(np.linalg.norm(base - wrist))

        if not mcp_dists or not fingertip_dists:
            return "UNKNOWN"

        palm_size = float(np.mean(mcp_dists))
        mean_tip_dist = float(np.mean(fingertip_dists))

        if palm_size <= 1e-6:
            return "UNKNOWN"

        openness = mean_tip_dist / palm_size

        if openness > 1.3:
            return "OPEN PALM"
        elif openness < 1.1:
            return "FIST"
        else:
            return "UNKNOWN"

    def _hand_location_px(self, hand_landmarks, image_shape):
        """
        Approximate hand center (cx, cy) in pixel coordinates.
        Pixel space: origin at top-left, +x right, +y down.
        """
        h, w = image_shape[:2]
        print(h,w)
        xs = []
        ys = []
        for lm in hand_landmarks.landmark:
            xs.append(lm.x * w)
            ys.append(lm.y * h)

        if not xs or not ys:
            return None

        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
        return cx, cy

    def _pixel_to_meters(self, cx_px: float, cy_px: float, image_shape):
        h, w = image_shape[:2]

    # Normalize 0–1
        x_norm = cx_px / w
        y_norm = cy_px / h

        X_RANGE = self.x_range_m
        Y_RANGE = self.y_range_m

    # Convert to meters (origin bottom-left)
        x_m = x_norm * X_RANGE
        y_m = y_norm * Y_RANGE  

        return x_m, y_m


    # -------------------- public API --------------------

    def read_gesture(self):
        """
        Grab ONE frame from the already-open camera and run gesture detection.

        Returns:
            gesture_label, location_m, location_px

            gesture_label:
                "OPEN PALM", "FIST", "UNKNOWN", or None if no hand / frame error.

            location_m:
                (x_m, y_m) in meters from bottom-left corner, or None.

            location_px:
                (cx_px, cy_px) in pixels from top-left corner, or None.
        """
        if self.cap is None:
            # camera already released
            return None, None, None

        ret, frame = self.cap.read()
        if not ret:
            # Camera problem or end of stream
            return None, None, None

        # Convert BGR→RGB as required by MediaPipe
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        gesture_label = None
        location_px = None
        location_m = None

        if results.multi_hand_landmarks:
            hand_landmarks = results.multi_hand_landmarks[0]

            # Classify
            gesture_label = self._classify_hand(hand_landmarks)

            # Location in pixels
            location_px = self._hand_location_px(hand_landmarks, frame.shape)

            # Convert to meters (bottom-left origin)
            if location_px is not None:
                cy_px, cx_px = location_px
                location_m = self._pixel_to_meters(cx_px, cy_px, frame.shape)

            # Debug drawing
            if self.debug:
                mp_drawing.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                )
                if location_px is not None:
                    cy, cx = location_px
                    cv2.circle(frame, (int(cx), int(cy)), 8, (0, 255, 0), -1)

        if self.debug:
            text = gesture_label if gesture_label is not None else "No hand"
            cv2.putText(
                frame,
                text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("Hand Gesture (debug)", frame)
            cv2.waitKey(1)

        return gesture_label, location_m, location_px

    def release(self):
        """
        Release camera and close any windows.
        """
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        if self.hands is not None:
            # Some versions of MediaPipe have .close()
            try:
                self.hands.close()
            except AttributeError:
                pass
            self.hands = None

        if self.debug:
            cv2.destroyAllWindows()
