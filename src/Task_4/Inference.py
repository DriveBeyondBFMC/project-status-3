import cv2
from ultralytics import YOLO
import numpy as np
import time
import torch
import logging
import argparse
from typing import Dict, Tuple

logging.basicConfig(level = logging.INFO, format = '%(asctime)s - %(levelname)s - %(message)s')

class VideoProcessor:
    """Class to handle video processing using YOLOv8 model."""
    
    CONFIDENCE_THRESHOLDS: Dict[str, float] = {
        'crosswalk': 0.8,
        'hw-entry': 0.7,
        'hw-exit': 0.7,
        'lane': 0.5,
        'no_entry': 0.7,
        'obstacle': 0.7,
        'onewayroad': 0.7,
        'parking': 0.7,
        'priority': 0.7,
        'roundabout': 0.7,
        'stop-line': 0.7,
        'stop-sign': 0.7,
        'parking-spot': 0.7,
        "car": 0.7
    }

    def __init__(self, 
                 model_path: str, 
                 device: str = None):
        
        self.device = torch.device(device if device else ('cuda' if torch.cuda.is_available() else 'cpu'))
        self.model = YOLO(model_path).to(self.device)
        self.class_colors = self.initialize_class_colors(self.CONFIDENCE_THRESHOLDS.keys())

    @staticmethod
    def initialize_class_colors(classes: list) -> Dict[str, Tuple[int, int, int]]:
        np.random.seed(42)
        return {cls: np.random.randint(50, 255, 3).tolist() for cls in classes}

    @staticmethod
    def draw_bounding_box(frame: np.ndarray, 
                          box, class_name: str, 
                          conf: float, 
                          color: Tuple[int, int, int]) -> None:
        """Draw a bounding box around the detected object."""

        x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        
        label = f'{class_name}: {conf:.2f}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_margin = 3
        
        label_size, _ = cv2.getTextSize(label, font, font_scale, font_thickness)
        label_x = x1
        label_y = y1 - label_size[1] - text_margin if y1 > label_size[1] + text_margin else y1 + label_size[1] + text_margin
        
        cv2.rectangle(frame, 
                      (label_x, label_y - label_size[1] - text_margin // 2), 
                      (label_x + label_size[0], label_y + text_margin // 2), 
                      color, cv2.FILLED)
        
        cv2.putText(frame, label, (label_x, label_y), font, font_scale, (0, 0, 0), font_thickness, cv2.LINE_AA)

    @staticmethod
    def apply_mask(frame: np.ndarray, 
                   mask, 
                   color: Tuple[int, int, int], 
                   alpha: float = 0.3) -> np.ndarray:
        """Apply a colored mask to the frame."""
        mask_np = mask.cpu().numpy().astype(np.uint8)
        mask_np = cv2.resize(mask_np, (frame.shape[1], frame.shape[0]))
        colored_mask = np.zeros_like(frame)
        colored_mask[:, :] = color
        return np.where(mask_np[:, :, np.newaxis] == 1,
                        cv2.addWeighted(frame, 1 - alpha, colored_mask, alpha, 0),
                        frame)

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        """Process a single frame with the model and draw detections."""
        results = self.model.predict(source=frame, save=False, verbose=False, conf=0.5)
        for result in results:
            boxes = result.boxes
            masks = result.masks
            if masks is None:
                continue
            for box, mask in zip(boxes, masks.data):
                cls_id = int(box.cls[0])
                conf = box.conf[0].item()
                class_name = self.model.names[cls_id]
                if class_name in self.CONFIDENCE_THRESHOLDS and conf > self.CONFIDENCE_THRESHOLDS[class_name]:
                    color = self.class_colors.get(class_name, (0, 255, 0))
                    self.draw_bounding_box(frame, box, class_name, conf, color)
                    if mask is not None:
                        frame = self.apply_mask(frame, mask, color)
        return frame

    def process_video(self, 
                      video_path: str, 
                      output_path: str, 
                      speed_factor: int = 1) -> None:
        """Process a video file and save the output."""
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            logging.error("Can't open Video.")
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 20.0
        frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        out = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (640, 480))

        frame_count = 0
        start_time = time.time()

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            if frame_count % speed_factor != 0:
                frame_count += 1
                continue
            frame_count += 1
            frame = cv2.resize(frame, (640, 480))
            frame = self.process_frame(frame)

            elapsed_time = time.time() - start_time
            fps_display = frame_count / elapsed_time
            cv2.putText(frame, f'FPS: {fps_display:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            out.write(frame)
            cv2.imshow('YOLOv8 Segmentation', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        out.release()
        cv2.destroyAllWindows()

def parse_args():

    parser = argparse.ArgumentParser(description="Process video using YOLO model.")
    parser.add_argument('--model', type = str, required = True, help = "Path to the YOLO model (.pt file)")
    parser.add_argument('--input', type = str, required = True, help = "Path to the input video file")
    parser.add_argument('--output', type = str, required=True, help = "Path to save the processed video")
    parser.add_argument('--speed-factor', type = int, default = 1, help = "Speed factor for processing frames (default: 1)")
    parser.add_argument('--device', type = str, default = None, help = "Device to run the model on (e.g., 'cuda', 'cpu')")
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()

    processor = VideoProcessor(
        model_path = args.model, 
        device = args.device
    )

    try:
        processor.process_video(
            video_path = args.input, 
            output_path = args.output, 
            speed_factor = args.speed_factor
        )
        logging.info("Video processing completed successfully.")
    except Exception as e:
        logging.error(f"An error occurred during video processing: {e}")