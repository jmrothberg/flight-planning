"""
Vision System for Object Detection and Image Processing
Identifies people, equipment, hazards and other objects of interest
Designed for efficiency and eventual deployment on embedded systems
"""

import numpy as np
import cv2
import time
from typing import List, Dict, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum

class ObjectType(Enum):
    """Types of objects that can be detected."""
    PERSON = "person"
    EQUIPMENT = "equipment"
    HAZARD = "hazard"
    WEAPON = "weapon"  # simulated
    IED = "ied"        # simulated sensor hit
    FURNITURE = "furniture"
    UNKNOWN = "unknown"

@dataclass
class Detection:
    """Object detection result."""
    object_type: ObjectType
    confidence: float
    bbox: Tuple[int, int, int, int]  # x, y, width, height
    position: Tuple[float, float]  # world coordinates
    timestamp: float
    description: str = ""

class SimpleObjectDetector:
    """
    Simplified object detector for simulation.
    In real deployment, this would use YOLOv5/v8 or similar models.
    """
    
    def __init__(self):
        """Initialize simple object detector."""
        # Detection parameters
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        
        # Object templates (simplified for simulation)
        self.object_templates = {
            ObjectType.PERSON: {
                'size_range': (20, 200),  # pixel height range
                'aspect_ratio': (0.3, 0.8),  # width/height ratio
                'color_range': [(50, 20, 20), (180, 255, 255)]  # HSV range
            },
            ObjectType.EQUIPMENT: {
                'size_range': (30, 150),
                'aspect_ratio': (0.5, 2.0),
                'color_range': [(0, 0, 100), (180, 50, 255)]  # Metallic/gray objects
            },
            ObjectType.HAZARD: {
                'size_range': (10, 100),
                'aspect_ratio': (0.8, 1.2),
                'color_range': [(0, 100, 100), (10, 255, 255)]  # Red objects
            }
        }
    
    def detect_objects(self, image: np.ndarray) -> List[Detection]:
        """
        Detect objects in image.
        Returns list of Detection objects.
        """
        if image is None or image.size == 0:
            return []
        
        detections = []
        
        # Convert to HSV for better color detection
        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except:
            return []
        
        # Detect each object type
        for obj_type, template in self.object_templates.items():
            type_detections = self._detect_object_type(image, hsv, obj_type, template)
            detections.extend(type_detections)
        
        # Apply non-maximum suppression to remove overlapping detections
        detections = self._apply_nms(detections)
        
        return detections
    
    def _detect_object_type(self, image: np.ndarray, hsv: np.ndarray, 
                           obj_type: ObjectType, template: Dict) -> List[Detection]:
        """Detect specific object type using template matching."""
        detections = []
        
        # Create color mask
        lower_color = np.array(template['color_range'][0])
        upper_color = np.array(template['color_range'][1])
        color_mask = cv2.inRange(hsv, lower_color, upper_color)
        
        # Find contours
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            # Get bounding box
            x, y, w, h = cv2.boundingRect(contour)
            
            # Check size constraints
            if not (template['size_range'][0] <= h <= template['size_range'][1]):
                continue
            
            # Check aspect ratio
            aspect_ratio = w / h if h > 0 else 0
            if not (template['aspect_ratio'][0] <= aspect_ratio <= template['aspect_ratio'][1]):
                continue
            
            # Calculate confidence based on contour properties
            area = cv2.contourArea(contour)
            bbox_area = w * h
            fill_ratio = area / bbox_area if bbox_area > 0 else 0
            
            # Simple confidence calculation
            confidence = min(0.9, fill_ratio * 1.2)
            
            if confidence >= self.confidence_threshold:
                detection = Detection(
                    object_type=obj_type,
                    confidence=confidence,
                    bbox=(x, y, w, h),
                    position=(x + w//2, y + h//2),  # Center of bbox
                    timestamp=time.time(),
                    description=f"{obj_type.value} detected"
                )
                detections.append(detection)
        
        return detections
    
    def _apply_nms(self, detections: List[Detection]) -> List[Detection]:
        """Apply non-maximum suppression to remove overlapping detections."""
        if len(detections) <= 1:
            return detections
        
        # Convert to format for OpenCV NMS
        boxes = []
        scores = []
        
        for detection in detections:
            x, y, w, h = detection.bbox
            boxes.append([x, y, x + w, y + h])
            scores.append(detection.confidence)
        
        boxes = np.array(boxes, dtype=np.float32)
        scores = np.array(scores, dtype=np.float32)
        
        # Apply NMS
        indices = cv2.dnn.NMSBoxes(boxes, scores, self.confidence_threshold, self.nms_threshold)
        
        # Return filtered detections
        filtered_detections = []
        if len(indices) > 0:
            for i in indices.flatten():
                filtered_detections.append(detections[i])
        
        return filtered_detections

class AdvancedObjectDetector:
    """
    Advanced object detector using deep learning models.
    This would use YOLOv8 or similar in real deployment.
    """
    
    def __init__(self, model_path: Optional[str] = None):
        """Initialize advanced object detector."""
        self.model_path = model_path
        self.model = None
        self.class_names = [
            'person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train', 'truck',
            'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench',
            'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra',
            'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
            'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove',
            'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup',
            'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
            'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'sofa',
            'pottedplant', 'bed', 'diningtable', 'toilet', 'tvmonitor', 'laptop', 'mouse',
            'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink',
            'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier',
            'toothbrush'
        ]
        
        # Try to load YOLO model if available
        self._load_model()
    
    def _load_model(self):
        """Load YOLO model for object detection."""
        try:
            # Try to import ultralytics YOLO
            from ultralytics import YOLO
            self.model = YOLO('yolov8n.pt')  # Nano model for speed
            print("Loaded YOLOv8 model for object detection")
        except ImportError:
            print("YOLOv8 not available, using simple detection")
            self.model = None
        except Exception as e:
            print(f"Failed to load YOLO model: {e}")
            self.model = None
    
    def detect_objects(self, image: np.ndarray) -> List[Detection]:
        """Detect objects using YOLO model."""
        if self.model is None or image is None:
            return []
        
        try:
            # Run inference
            results = self.model(image, verbose=False)
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Extract box information
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        confidence = float(box.conf[0])
                        class_id = int(box.cls[0])
                        
                        if confidence < 0.5:  # Confidence threshold
                            continue
                        
                        # Convert to our detection format
                        class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"
                        obj_type = self._map_class_to_type(class_name)
                        
                        detection = Detection(
                            object_type=obj_type,
                            confidence=confidence,
                            bbox=(int(x1), int(y1), int(x2-x1), int(y2-y1)),
                            position=((x1+x2)/2, (y1+y2)/2),
                            timestamp=time.time(),
                            description=f"{class_name} (YOLO)"
                        )
                        detections.append(detection)
            
            return detections
            
        except Exception as e:
            print(f"YOLO detection failed: {e}")
            return []
    
    def _map_class_to_type(self, class_name: str) -> ObjectType:
        """Map YOLO class name to our object type."""
        person_classes = ['person']
        equipment_classes = ['laptop', 'cell phone', 'tv', 'microwave', 'oven', 'refrigerator']
        hazard_classes = ['fire hydrant', 'stop sign']
        furniture_classes = ['chair', 'sofa', 'bed', 'dining table']
        
        if class_name in person_classes:
            return ObjectType.PERSON
        elif class_name in equipment_classes:
            return ObjectType.EQUIPMENT
        elif class_name in hazard_classes:
            return ObjectType.HAZARD
        elif class_name in furniture_classes:
            return ObjectType.FURNITURE
        else:
            return ObjectType.UNKNOWN

class VisionSystem:
    """
    Complete vision system for drone navigation.
    Handles image processing, object detection, and data management.
    """
    
    def __init__(self, use_advanced: bool = False):
        """Initialize vision system."""
        # Initialize detectors
        self.simple_detector = SimpleObjectDetector()
        self.advanced_detector = AdvancedObjectDetector() if use_advanced else None
        self.use_advanced = use_advanced and self.advanced_detector is not None
        
        # Detection history
        self.recent_detections = []
        self.max_history = 100
        
        # Performance tracking
        self.frame_count = 0
        self.processing_times = []
        
        # Priority detection settings
        self.priority_objects = [ObjectType.PERSON, ObjectType.HAZARD, ObjectType.WEAPON, ObjectType.IED]
        self.detection_cooldown = {}  # Prevent spam of same detections
    
    def process_frame(self, camera_data: Dict) -> List[Detection]:
        """
        Process camera frame and detect objects.
        Returns list of important detections.
        """
        if not camera_data or 'objects' not in camera_data:
            return []
        
        start_time = time.time()
        detections = []
        
        # Convert simulation objects to detections
        for obj in camera_data['objects']:
            obj_type = self._string_to_object_type(obj['type'])
            
            # Check detection cooldown to prevent spam
            obj_key = (obj_type, obj['position'])
            current_time = time.time()
            
            if obj_key in self.detection_cooldown:
                if current_time - self.detection_cooldown[obj_key] < 2.0:  # 2 second cooldown
                    continue
            
            self.detection_cooldown[obj_key] = current_time
            
            detection = Detection(
                object_type=obj_type,
                confidence=obj['confidence'],
                bbox=(0, 0, 50, 50),  # Simulated bbox
                position=obj['position'],
                timestamp=current_time,
                description=f"Simulated {obj['type']}"
            )
            detections.append(detection)
        
        # Add to history
        self.recent_detections.extend(detections)
        
        # Trim history
        if len(self.recent_detections) > self.max_history:
            self.recent_detections = self.recent_detections[-self.max_history:]
        
        # Track performance
        processing_time = time.time() - start_time
        self.processing_times.append(processing_time)
        if len(self.processing_times) > 100:
            self.processing_times.pop(0)
        
        self.frame_count += 1
        
        return detections
    
    def get_priority_detections(self) -> List[Detection]:
        """Get recent high-priority detections."""
        priority_detections = []
        cutoff_time = time.time() - 10.0  # Last 10 seconds
        
        for detection in self.recent_detections:
            if (detection.timestamp >= cutoff_time and 
                detection.object_type in self.priority_objects):
                priority_detections.append(detection)
        
        return priority_detections
    
    def get_recent_detections(self, max_age: float = 30.0) -> List[Detection]:
        """Get all recent detections within max_age seconds."""
        cutoff_time = time.time() - max_age
        return [d for d in self.recent_detections if d.timestamp >= cutoff_time]
    
    def get_performance_stats(self) -> Dict:
        """Get vision system performance statistics."""
        avg_processing_time = np.mean(self.processing_times) if self.processing_times else 0
        fps = 1.0 / avg_processing_time if avg_processing_time > 0 else 0
        
        return {
            'frames_processed': self.frame_count,
            'avg_processing_time': avg_processing_time,
            'estimated_fps': fps,
            'total_detections': len(self.recent_detections),
            'priority_detections': len(self.get_priority_detections())
        }
    
    def _string_to_object_type(self, type_str: str) -> ObjectType:
        """Convert string to ObjectType enum."""
        type_map = {
            'person': ObjectType.PERSON,
            'equipment': ObjectType.EQUIPMENT,
            'hazard': ObjectType.HAZARD,
            'weapon': ObjectType.WEAPON,
            'ied': ObjectType.IED,
            'furniture': ObjectType.FURNITURE
        }
        return type_map.get(type_str.lower(), ObjectType.UNKNOWN)
    
    def reset(self):
        """Reset vision system state."""
        self.recent_detections = []
        self.frame_count = 0
        self.processing_times = []
        self.detection_cooldown = {}

class ImageProcessor:
    """
    Image processing utilities for vision enhancement.
    Includes filtering, enhancement, and preprocessing functions.
    """
    
    @staticmethod
    def enhance_image(image: np.ndarray) -> np.ndarray:
        """Enhance image quality for better detection."""
        if image is None:
            return None
        
        # Apply CLAHE for contrast enhancement
        lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        l = clahe.apply(l)
        
        enhanced = cv2.merge([l, a, b])
        enhanced = cv2.cvtColor(enhanced, cv2.COLOR_LAB2BGR)
        
        return enhanced
    
    @staticmethod
    def denoise_image(image: np.ndarray) -> np.ndarray:
        """Remove noise from image."""
        if image is None:
            return None
        
        return cv2.fastNlMeansDenoisingColored(image, None, 10, 10, 7, 21)
    
    @staticmethod
    def resize_for_processing(image: np.ndarray, max_size: int = 640) -> np.ndarray:
        """Resize image for efficient processing."""
        if image is None:
            return None
        
        h, w = image.shape[:2]
        if max(h, w) <= max_size:
            return image
        
        # Calculate new dimensions
        if h > w:
            new_h = max_size
            new_w = int(w * max_size / h)
        else:
            new_w = max_size
            new_h = int(h * max_size / w)
        
        return cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
