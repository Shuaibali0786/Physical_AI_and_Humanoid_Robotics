# Multi-modal Interaction: Speech, Gesture, and Vision

## Introduction to Multi-modal Interaction

Multi-modal interaction in humanoid robotics combines multiple sensory channels—speech, gesture, and vision—to create more natural and intuitive human-robot communication. Unlike single-modal systems, multi-modal interaction allows humans to communicate with robots using the same rich, contextual communication methods they use with other humans, significantly improving the effectiveness and naturalness of human-robot interaction.

## The Multi-modal Framework

### Integration Architecture

Multi-modal interaction systems integrate three primary modalities:

- **Speech**: Natural language commands and responses
- **Gesture**: Pointing, beckoning, and other body language
- **Vision**: Object recognition, scene understanding, and visual context

These modalities work together to provide redundancy, disambiguation, and richer communication:

```python
import cv2
import numpy as np
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

@dataclass
class SpeechInput:
    """Represents speech input with confidence and timing"""
    text: str
    confidence: float
    timestamp: float
    audio_data: Optional[np.ndarray] = None

@dataclass
class GestureInput:
    """Represents gesture input with type and location"""
    gesture_type: str  # "pointing", "beckoning", "waving", etc.
    location_2d: Tuple[int, int]  # 2D coordinates in camera frame
    location_3d: Optional[Tuple[float, float, float]]  # 3D coordinates in world frame
    confidence: float
    timestamp: float

@dataclass
class VisionInput:
    """Represents visual input with detected objects and scene info"""
    objects: List[Dict]  # List of detected objects with properties
    scene_description: str
    image: Optional[np.ndarray] = None
    timestamp: float

class MultiModalInput:
    """Container for synchronized multi-modal inputs"""
    def __init__(self, speech: Optional[SpeechInput] = None,
                 gesture: Optional[GestureInput] = None,
                 vision: Optional[VisionInput] = None):
        self.speech = speech
        self.gesture = gesture
        self.vision = vision
        self.timestamp = max(
            [ts for ts in [speech.timestamp if speech else None,
                          gesture.timestamp if gesture else None,
                          vision.timestamp if vision else None]
             if ts is not None],
            default=0.0
        )
```

### Synchronization and Fusion

```python
import time
from collections import deque

class MultiModalSynchronizer:
    def __init__(self, sync_window: float = 0.5):  # 500ms sync window
        self.sync_window = sync_window
        self.speech_buffer = deque(maxlen=10)
        self.gesture_buffer = deque(maxlen=10)
        self.vision_buffer = deque(maxlen=10)

    def add_speech_input(self, speech: SpeechInput):
        """Add speech input to synchronization buffer"""
        self.speech_buffer.append(speech)
        return self.attempt_synchronization()

    def add_gesture_input(self, gesture: GestureInput):
        """Add gesture input to synchronization buffer"""
        self.gesture_buffer.append(gesture)
        return self.attempt_synchronization()

    def add_vision_input(self, vision: VisionInput):
        """Add vision input to synchronization buffer"""
        self.vision_buffer.append(vision)
        return self.attempt_synchronization()

    def attempt_synchronization(self) -> Optional[MultiModalInput]:
        """Attempt to synchronize inputs within the time window"""
        current_time = time.time()

        # Find inputs within sync window
        speech_input = self.find_recent_input(self.speech_buffer, current_time)
        gesture_input = self.find_recent_input(self.gesture_buffer, current_time)
        vision_input = self.find_recent_input(self.vision_buffer, current_time)

        if speech_input or gesture_input or vision_input:
            return MultiModalInput(speech_input, gesture_input, vision_input)

        return None

    def find_recent_input(self, buffer, reference_time):
        """Find the most recent input within sync window"""
        if not buffer:
            return None

        recent_input = None
        min_time_diff = float('inf')

        for item in buffer:
            time_diff = abs(reference_time - item.timestamp)
            if time_diff <= self.sync_window and time_diff < min_time_diff:
                min_time_diff = time_diff
                recent_input = item

        return recent_input
```

## Speech Processing in Multi-modal Context

### Context-Aware Speech Recognition

```python
class ContextAwareSpeechProcessor:
    def __init__(self, base_asr_model):
        self.base_asr = base_asr_model
        self.vision_context = None
        self.gesture_context = None

    def process_speech_with_context(self, audio_data, vision_context: VisionInput = None,
                                  gesture_context: GestureInput = None) -> SpeechInput:
        """Process speech with multi-modal context"""
        self.vision_context = vision_context
        self.gesture_context = gesture_context

        # Get initial transcription
        text = self.base_asr.transcribe(audio_data)
        confidence = self.base_asr.get_confidence()

        # Apply context-based disambiguation
        disambiguated_text = self.apply_context_disambiguation(text)

        # Update confidence based on context consistency
        adjusted_confidence = self.adjust_confidence_with_context(
            text, disambiguated_text, confidence
        )

        return SpeechInput(
            text=disambiguated_text,
            confidence=adjusted_confidence,
            timestamp=time.time(),
            audio_data=audio_data
        )

    def apply_context_disambiguation(self, text: str) -> str:
        """Apply context to disambiguate speech"""
        if not self.vision_context:
            return text

        # Example: Resolve pronouns based on visual context
        # "Pick it up" -> "Pick up the red cup" (if red cup is visible)
        if "it" in text.lower():
            visible_objects = self.vision_context.objects
            if visible_objects:
                # For simplicity, take the most recently detected object
                most_recent_object = visible_objects[0] if visible_objects else None
                if most_recent_object:
                    resolved_text = text.lower().replace("it", most_recent_object.get("name", "object"))
                    return resolved_text

        # Example: Resolve spatial references
        # "Go there" -> "Go to the kitchen" (if gesture points to kitchen area)
        if "there" in text.lower() and self.gesture_context:
            if self.gesture_context.gesture_type == "pointing":
                # Resolve "there" to actual location based on gesture
                resolved_text = self.resolve_spatial_reference(text, self.gesture_context)
                if resolved_text != text:
                    return resolved_text

        return text

    def resolve_spatial_reference(self, text: str, gesture: GestureInput) -> str:
        """Resolve spatial references like 'there' based on gesture"""
        # This would involve mapping 2D gesture location to 3D world coordinates
        # and identifying the corresponding location or object
        if gesture.location_3d:
            # Convert 3D location to meaningful name (kitchen, living room, etc.)
            location_name = self.get_location_name_from_coordinates(gesture.location_3d)
            if location_name:
                return text.lower().replace("there", f"to {location_name}")

        return text

    def adjust_confidence_with_context(self, original_text: str,
                                     disambiguated_text: str,
                                     base_confidence: float) -> float:
        """Adjust confidence based on context consistency"""
        if original_text == disambiguated_text:
            # No disambiguation needed, keep original confidence
            return base_confidence

        # Disambiguation applied, potentially increase confidence if context is clear
        if self.vision_context and self.gesture_context:
            # Strong multi-modal context, increase confidence
            return min(1.0, base_confidence + 0.1)

        # Weak context, keep original confidence
        return base_confidence

    def get_location_name_from_coordinates(self, coordinates: Tuple[float, float, float]) -> Optional[str]:
        """Convert 3D coordinates to meaningful location name"""
        # This would typically use a map or spatial database
        # For example, check if coordinates fall within known room boundaries
        location_map = {
            "kitchen": ((0, 0, 0), (3, 3, 3)),      # (min_coords, max_coords)
            "living_room": ((3, 0, 0), (7, 4, 3)),
            "bedroom": ((0, 4, 0), (4, 7, 3))
        }

        for location_name, (min_coords, max_coords) in location_map.items():
            if (min_coords[0] <= coordinates[0] <= max_coords[0] and
                min_coords[1] <= coordinates[1] <= max_coords[1] and
                min_coords[2] <= coordinates[2] <= max_coords[2]):
                return location_name

        return None
```

## Gesture Recognition and Interpretation

### Gesture Detection Pipeline

```python
import mediapipe as mp
import math

class GestureRecognizer:
    def __init__(self):
        # Initialize MediaPipe for hand tracking
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        self.mp_drawing = mp.solutions.drawing_utils

    def detect_gestures(self, image: np.ndarray) -> List[GestureInput]:
        """Detect gestures from image frame"""
        results = self.hands.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        gestures = []

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                gesture = self.classify_gesture(hand_landmarks, image.shape)
                if gesture:
                    gestures.append(gesture)

        return gestures

    def classify_gesture(self, hand_landmarks, image_shape) -> Optional[GestureInput]:
        """Classify hand gesture based on landmarks"""
        # Get key landmarks
        landmarks = hand_landmarks.landmark

        # Calculate gesture features
        pointing_direction = self.calculate_pointing_direction(landmarks, image_shape)
        finger_positions = self.get_finger_positions(landmarks)
        palm_orientation = self.get_palm_orientation(landmarks)

        # Classify gesture type
        gesture_type = self.classify_gesture_type(finger_positions, pointing_direction)

        if gesture_type:
            # Calculate 2D position (tip of index finger)
            index_finger_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
            x_2d = int(index_finger_tip.x * image_shape[1])
            y_2d = int(index_finger_tip.y * image_shape[0])

            return GestureInput(
                gesture_type=gesture_type,
                location_2d=(x_2d, y_2d),
                location_3d=None,  # Will be calculated later with depth info
                confidence=0.8,  # Base confidence
                timestamp=time.time()
            )

        return None

    def calculate_pointing_direction(self, landmarks, image_shape) -> Tuple[float, float]:
        """Calculate the pointing direction of the hand"""
        # Use index finger tip and wrist to determine pointing direction
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        index_tip = landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]

        dx = index_tip.x - wrist.x
        dy = index_tip.y - wrist.y

        # Normalize
        length = math.sqrt(dx*dx + dy*dy)
        if length > 0:
            return (dx/length, dy/length)
        else:
            return (0, 0)

    def get_finger_positions(self, landmarks) -> Dict[str, Tuple[float, float]]:
        """Get positions of key fingers"""
        finger_tips = {
            'thumb': landmarks[self.mp_hands.HandLandmark.THUMB_TIP],
            'index': landmarks[self.mp_hands.HandLandmark.INDEX_FINGER_TIP],
            'middle': landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_TIP],
            'ring': landmarks[self.mp_hands.HandLandmark.RING_FINGER_TIP],
            'pinky': landmarks[self.mp_hands.HandLandmark.PINKY_TIP]
        }

        return {finger: (landmark.x, landmark.y) for finger, landmark in finger_tips.items()}

    def get_palm_orientation(self, landmarks) -> float:
        """Calculate palm orientation (simplified)"""
        # Use wrist and middle finger metacarpal to estimate palm orientation
        wrist = landmarks[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = landmarks[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]

        # Calculate angle
        dx = middle_mcp.x - wrist.x
        dy = middle_mcp.y - wrist.y

        return math.atan2(dy, dx)

    def classify_gesture_type(self, finger_positions: Dict, pointing_direction) -> Optional[str]:
        """Classify gesture type based on finger positions"""
        # Simple classification based on finger positions
        index_pos = finger_positions['index']
        middle_pos = finger_positions['middle']
        thumb_pos = finger_positions['thumb']
        ring_pos = finger_positions['ring']
        pinky_pos = finger_positions['pinky']

        # Pointing gesture: index finger extended, others curled
        if (self.is_extended(index_pos, thumb_pos) and
            self.is_curled(middle_pos, thumb_pos) and
            self.is_curled(ring_pos, thumb_pos) and
            self.is_curled(pinky_pos, thumb_pos)):
            return "pointing"

        # Peace sign: index and middle extended, others curled
        if (self.is_extended(index_pos, thumb_pos) and
            self.is_extended(middle_pos, thumb_pos) and
            self.is_curled(ring_pos, thumb_pos) and
            self.is_curled(pinky_pos, thumb_pos)):
            return "peace"

        # Thumbs up: thumb up, others curled
        if (self.is_extended(thumb_pos, finger_positions['index']) and
            self.is_curled(index_pos, thumb_pos) and
            self.is_curled(middle_pos, thumb_pos)):
            return "thumbs_up"

        # Wave: all fingers extended, moving
        if (self.are_all_extended([index_pos, middle_pos, ring_pos, pinky_pos], thumb_pos)):
            return "waving"

        return None

    def is_extended(self, finger_pos, reference_pos, threshold=0.1) -> bool:
        """Check if finger is extended (away from palm)"""
        distance = math.sqrt((finger_pos[0] - reference_pos[0])**2 +
                           (finger_pos[1] - reference_pos[1])**2)
        return distance > threshold

    def is_curled(self, finger_pos, reference_pos, threshold=0.1) -> bool:
        """Check if finger is curled (close to palm)"""
        return not self.is_extended(finger_pos, reference_pos, threshold)

    def are_all_extended(self, finger_positions, reference_pos, threshold=0.1) -> bool:
        """Check if all fingers are extended"""
        return all(self.is_extended(pos, reference_pos, threshold)
                  for pos in finger_positions)
```

### 3D Gesture Mapping

```python
class Gesture3DMapper:
    def __init__(self, camera_intrinsics, robot_pose):
        self.camera_intrinsics = camera_intrinsics  # Camera parameters
        self.robot_pose = robot_pose  # Robot's current pose in world

    def map_2d_to_3d(self, gesture_2d: GestureInput, depth_image: np.ndarray) -> GestureInput:
        """Map 2D gesture coordinates to 3D world coordinates"""
        x_2d, y_2d = gesture_2d.location_2d

        # Get depth at gesture location
        if depth_image is not None:
            depth_value = depth_image[y_2d, x_2d]
        else:
            # Use estimated depth or default value
            depth_value = 1.0  # meters

        # Convert 2D pixel coordinates + depth to 3D world coordinates
        x_3d, y_3d, z_3d = self.pixel_to_world_coordinates(
            x_2d, y_2d, depth_value, self.camera_intrinsics
        )

        # Transform to robot's coordinate frame
        world_coords = self.transform_to_robot_frame(
            (x_3d, y_3d, z_3d), self.robot_pose
        )

        # Update gesture with 3D coordinates
        gesture_3d = GestureInput(
            gesture_type=gesture_2d.gesture_type,
            location_2d=gesture_2d.location_2d,
            location_3d=world_coords,
            confidence=gesture_2d.confidence,
            timestamp=gesture_2d.timestamp
        )

        return gesture_3d

    def pixel_to_world_coordinates(self, x: int, y: int, depth: float,
                                 intrinsics: Dict) -> Tuple[float, float, float]:
        """Convert pixel coordinates + depth to world coordinates"""
        # Camera intrinsic parameters
        fx = intrinsics['fx']
        fy = intrinsics['fy']
        cx = intrinsics['cx']
        cy = intrinsics['cy']

        # Convert to normalized coordinates
        x_norm = (x - cx) / fx
        y_norm = (y - cy) / fy

        # Convert to world coordinates
        x_world = x_norm * depth
        y_world = y_norm * depth
        z_world = depth

        return (x_world, y_world, z_world)

    def transform_to_robot_frame(self, world_coords: Tuple[float, float, float],
                               robot_pose: Dict) -> Tuple[float, float, float]:
        """Transform world coordinates to robot's local coordinate frame"""
        # This would involve applying the inverse of the robot's pose transformation
        # For simplicity, returning the same coordinates
        # In practice, this would involve rotation and translation matrices
        return world_coords
```

## Vision Processing and Scene Understanding

### Object Detection and Recognition

```python
import torch
import torchvision.transforms as T
from PIL import Image

class VisionProcessor:
    def __init__(self):
        # Load pre-trained object detection model (e.g., YOLO, Faster R-CNN)
        self.detection_model = self.load_detection_model()
        self.classification_model = self.load_classification_model()
        self.transform = T.Compose([
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

    def load_detection_model(self):
        """Load object detection model"""
        # Using torchvision's pre-trained model as an example
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        return model

    def load_classification_model(self):
        """Load object classification model"""
        # Example using ResNet
        model = torch.hub.load('pytorch/vision:v0.10.0', 'resnet50', pretrained=True)
        model.eval()
        return model

    def process_scene(self, image: np.ndarray) -> VisionInput:
        """Process image to detect objects and understand scene"""
        # Convert numpy array to PIL Image
        pil_image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

        # Run object detection
        results = self.detection_model(pil_image)

        # Extract detected objects
        objects = self.extract_detected_objects(results, image.shape)

        # Generate scene description
        scene_description = self.generate_scene_description(objects, image)

        return VisionInput(
            objects=objects,
            scene_description=scene_description,
            image=image,
            timestamp=time.time()
        )

    def extract_detected_objects(self, detection_results, image_shape) -> List[Dict]:
        """Extract objects from detection results"""
        objects = []

        # Parse YOLO results
        detections = detection_results.pandas().xyxy[0]

        for _, detection in detections.iterrows():
            object_info = {
                'name': detection['name'],
                'confidence': detection['confidence'],
                'bbox': {
                    'x_min': int(detection['xmin']),
                    'y_min': int(detection['ymin']),
                    'x_max': int(detection['xmax']),
                    'y_max': int(detection['ymax'])
                },
                'center_2d': (
                    int((detection['xmin'] + detection['xmax']) / 2),
                    int((detection['ymin'] + detection['ymax']) / 2)
                ),
                'area': (detection['xmax'] - detection['xmin']) * (detection['ymax'] - detection['ymin'])
            }

            # Estimate 3D information if depth is available
            if hasattr(self, 'depth_estimator'):
                object_info['center_3d'] = self.estimate_3d_position(object_info, image_shape)

            objects.append(object_info)

        return objects

    def generate_scene_description(self, objects: List[Dict], image: np.ndarray) -> str:
        """Generate textual description of the scene"""
        if not objects:
            return "The scene appears to be empty or no objects were detected."

        # Count objects by type
        object_counts = {}
        for obj in objects:
            name = obj['name']
            object_counts[name] = object_counts.get(name, 0) + 1

        # Create description
        description_parts = []
        for obj_name, count in object_counts.items():
            if count == 1:
                description_parts.append(f"a {obj_name}")
            else:
                description_parts.append(f"{count} {obj_name}s")

        object_list = ", ".join(description_parts[:-1])
        if len(description_parts) > 1:
            scene_desc = f"The scene contains {object_list}, and {description_parts[-1]}."
        else:
            scene_desc = f"The scene contains {description_parts[0]}."

        return scene_desc

    def estimate_3d_position(self, object_info: Dict, image_shape: tuple) -> Optional[Tuple[float, float, float]]:
        """Estimate 3D position of object (requires depth information)"""
        # This would typically use depth estimation or stereo vision
        # For now, returning None
        return None

    def get_object_at_location(self, location_2d: Tuple[int, int], objects: List[Dict]) -> Optional[Dict]:
        """Get object at or near specified 2D location"""
        x, y = location_2d

        for obj in objects:
            bbox = obj['bbox']
            if (bbox['x_min'] <= x <= bbox['x_max'] and
                bbox['y_min'] <= y <= bbox['y_max']):
                return obj

        # If no object exactly at location, find closest one
        closest_obj = None
        min_distance = float('inf')

        for obj in objects:
            center = obj['center_2d']
            distance = math.sqrt((center[0] - x)**2 + (center[1] - y)**2)
            if distance < min_distance:
                min_distance = distance
                closest_obj = obj

        return closest_obj
```

## Multi-modal Fusion and Interpretation

### Fusion Engine

```python
class MultiModalFusionEngine:
    def __init__(self):
        self.confidence_weights = {
            'speech': 0.4,
            'gesture': 0.3,
            'vision': 0.3
        }

    def fuse_inputs(self, multi_modal_input: MultiModalInput) -> Dict:
        """Fuse multi-modal inputs into coherent interpretation"""
        interpretations = {}

        # Process each modality
        if multi_modal_input.speech:
            interpretations['speech'] = self.interpret_speech(multi_modal_input.speech)

        if multi_modal_input.gesture:
            interpretations['gesture'] = self.interpret_gesture(multi_modal_input.gesture)

        if multi_modal_input.vision:
            interpretations['vision'] = self.interpret_vision(multi_modal_input.vision)

        # Fuse interpretations based on confidence and context
        fused_interpretation = self.fuse_interpretations(interpretations, multi_modal_input)

        return fused_interpretation

    def interpret_speech(self, speech: SpeechInput) -> Dict:
        """Interpret speech input"""
        # This would use NLU system from previous sections
        interpretation = {
            'intent': self.extract_intent(speech.text),
            'entities': self.extract_entities(speech.text),
            'confidence': speech.confidence,
            'modality': 'speech'
        }
        return interpretation

    def interpret_gesture(self, gesture: GestureInput) -> Dict:
        """Interpret gesture input"""
        interpretation = {
            'gesture_type': gesture.gesture_type,
            'target_location': gesture.location_3d or gesture.location_2d,
            'confidence': gesture.confidence,
            'modality': 'gesture'
        }
        return interpretation

    def interpret_vision(self, vision: VisionInput) -> Dict:
        """Interpret vision input"""
        interpretation = {
            'detected_objects': vision.objects,
            'scene_description': vision.scene_description,
            'modality': 'vision'
        }
        return interpretation

    def extract_intent(self, text: str) -> str:
        """Extract intent from text (simplified)"""
        # This would use proper NLU system
        if any(word in text.lower() for word in ['go', 'move', 'navigate', 'walk']):
            return 'navigation'
        elif any(word in text.lower() for word in ['pick', 'grasp', 'take', 'get']):
            return 'manipulation'
        elif any(word in text.lower() for word in ['find', 'look', 'search']):
            return 'perception'
        else:
            return 'unknown'

    def extract_entities(self, text: str) -> List[str]:
        """Extract entities from text (simplified)"""
        # This would use proper NER system
        return []

    def fuse_interpretations(self, interpretations: Dict, multi_modal_input: MultiModalInput) -> Dict:
        """Fuse interpretations from different modalities"""
        # Initialize result structure
        fused_result = {
            'primary_intent': None,
            'target_object': None,
            'target_location': None,
            'action': None,
            'confidence': 0.0,
            'modality_contributions': {}
        }

        # Determine primary intent based on modality agreement and confidence
        intent_confidence = {}

        if 'speech' in interpretations:
            speech_intent = interpretations['speech']['intent']
            intent_confidence[speech_intent] = interpretations['speech']['confidence'] * self.confidence_weights['speech']

        if 'gesture' in interpretations:
            # Gestures often indicate navigation or pointing
            gesture_intent = 'navigation' if interpretations['gesture']['gesture_type'] == 'pointing' else 'attention'
            gesture_conf = interpretations['gesture']['confidence'] * self.confidence_weights['gesture']
            intent_confidence[gesture_intent] = intent_confidence.get(gesture_intent, 0) + gesture_conf

        if 'vision' in interpretations:
            # Vision provides context but less direct intent
            vision_conf = 0.5 * self.confidence_weights['vision']  # Base confidence
            intent_confidence['context'] = vision_conf

        # Select primary intent with highest confidence
        if intent_confidence:
            fused_result['primary_intent'] = max(intent_confidence, key=intent_confidence.get)
            fused_result['confidence'] = intent_confidence[fused_result['primary_intent']]

        # Resolve target object using multi-modal information
        fused_result['target_object'] = self.resolve_target_object(interpretations, multi_modal_input)

        # Resolve target location using gesture and vision
        fused_result['target_location'] = self.resolve_target_location(interpretations, multi_modal_input)

        # Generate action based on fused information
        fused_result['action'] = self.generate_action(fused_result)

        # Store modality contributions
        fused_result['modality_contributions'] = {k: v for k, v in intent_confidence.items()}

        return fused_result

    def resolve_target_object(self, interpretations: Dict, multi_modal_input: MultiModalInput) -> Optional[str]:
        """Resolve target object using multi-modal information"""
        candidates = []

        # Get object mentions from speech
        if 'speech' in interpretations:
            # This would use more sophisticated NLU to extract object mentions
            speech_objects = self.extract_objects_from_speech(interpretations['speech']['entities'])
            candidates.extend(speech_objects)

        # Get object at gesture location
        if 'gesture' in interpretations and 'vision' in interpretations:
            gesture_location = interpretations['gesture'].get('target_location')
            if gesture_location and multi_modal_input.vision:
                if isinstance(gesture_location, tuple) and len(gesture_location) == 2:
                    # 2D coordinates
                    target_obj = self.vision_processor.get_object_at_location(
                        gesture_location, multi_modal_input.vision.objects
                    )
                    if target_obj:
                        candidates.append(target_obj['name'])

        # Get most salient object from vision
        if 'vision' in interpretations:
            vision_objects = interpretations['vision']['detected_objects']
            if vision_objects:
                # Select most prominent object (largest area, closest, etc.)
                most_salient = max(vision_objects, key=lambda x: x['area'])
                candidates.append(most_salient['name'])

        # Return most confident candidate
        return candidates[0] if candidates else None

    def resolve_target_location(self, interpretations: Dict, multi_modal_input: MultiModalInput) -> Optional[Tuple]:
        """Resolve target location using multi-modal information"""
        if 'gesture' in interpretations:
            gesture = interpretations['gesture']
            if gesture.get('target_location'):
                return gesture['target_location']

        if 'vision' in interpretations:
            # Use scene description to infer location
            scene_desc = interpretations['vision']['scene_description']
            # This would use more sophisticated spatial reasoning
            return self.infer_location_from_scene(scene_desc)

        return None

    def generate_action(self, fused_result: Dict) -> Optional[str]:
        """Generate appropriate action based on fused interpretation"""
        intent = fused_result['primary_intent']
        obj = fused_result['target_object']
        location = fused_result['target_location']

        if intent == 'navigation' and location:
            return f"navigate_to({location})"
        elif intent == 'manipulation' and obj:
            return f"manipulate_object({obj})"
        elif intent == 'perception' and obj:
            return f"perceive_object({obj})"
        else:
            return "wait_for_clarification"

    def extract_objects_from_speech(self, entities: List[str]) -> List[str]:
        """Extract object names from speech entities"""
        # Simplified implementation
        return entities

    def infer_location_from_scene(self, scene_description: str) -> Optional[Tuple]:
        """Infer location from scene description"""
        # This would use more sophisticated spatial reasoning
        # For now, returning None
        return None
```

## Integration with Robot Control

### Action Execution System

```python
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import String

class MultiModalActionExecutor:
    def __init__(self):
        self.fusion_engine = MultiModalFusionEngine()
        self.vision_processor = VisionProcessor()
        self.gesture_recognizer = GestureRecognizer()
        self.speech_processor = ContextAwareSpeechProcessor(None)  # Placeholder for ASR

        # ROS 2 setup
        rclpy.init()
        self.node = rclpy.create_node('multimodal_executor')

        # Action clients
        self.move_base_client = ActionClient(self.node, 'MoveBase', 'move_base')
        self.manipulation_client = ActionClient(self.node, 'Manipulate', 'manipulation')

        # Publishers
        self.speech_publisher = self.node.create_publisher(String, 'robot_speech', 10)

    def process_multi_modal_command(self, multi_modal_input: MultiModalInput) -> bool:
        """Process multi-modal input and execute corresponding action"""
        try:
            # Fuse the multi-modal inputs
            fused_interpretation = self.fusion_engine.fuse_inputs(multi_modal_input)

            # Validate interpretation confidence
            if fused_interpretation['confidence'] < 0.5:
                self.request_clarification(multi_modal_input)
                return False

            # Execute action based on interpretation
            action_result = self.execute_action(fused_interpretation)

            return action_result

        except Exception as e:
            self.node.get_logger().error(f'Error processing multi-modal command: {e}')
            return False

    def execute_action(self, interpretation: Dict) -> bool:
        """Execute robot action based on interpretation"""
        action = interpretation['action']

        if action.startswith('navigate_to'):
            return self.execute_navigation(interpretation)
        elif action.startswith('manipulate_object'):
            return self.execute_manipulation(interpretation)
        elif action.startswith('perceive_object'):
            return self.execute_perception(interpretation)
        else:
            self.node.get_logger().warn(f'Unknown action: {action}')
            return False

    def execute_navigation(self, interpretation: Dict) -> bool:
        """Execute navigation action"""
        target_location = interpretation['target_location']

        if not target_location:
            self.node.get_logger().error('No target location specified for navigation')
            return False

        # Convert location to ROS Pose
        pose = self.convert_location_to_pose(target_location)

        # Send navigation goal
        goal_msg = self.create_navigation_goal(pose)

        self.move_base_client.wait_for_server()
        future = self.move_base_client.send_goal(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().success

    def execute_manipulation(self, interpretation: Dict) -> bool:
        """Execute manipulation action"""
        target_object = interpretation['target_object']

        if not target_object:
            self.node.get_logger().error('No target object specified for manipulation')
            return False

        # Find object in current scene
        current_objects = self.get_current_objects()
        target_obj_info = next((obj for obj in current_objects if obj['name'] == target_object), None)

        if not target_obj_info:
            self.node.get_logger().error(f'Object {target_object} not found in current scene')
            return False

        # Create manipulation goal
        goal_msg = self.create_manipulation_goal(target_obj_info)

        self.manipulation_client.wait_for_server()
        future = self.manipulation_client.send_goal(goal_msg)

        # Wait for result
        rclpy.spin_until_future_complete(self.node, future)

        return future.result().success

    def execute_perception(self, interpretation: Dict) -> bool:
        """Execute perception action"""
        target_object = interpretation['target_object']

        if target_object:
            # Focus perception on specific object
            self.focus_perception_on_object(target_object)
        else:
            # Perform general scene perception
            self.perform_general_perception()

        return True

    def request_clarification(self, multi_modal_input: MultiModalInput):
        """Request clarification when interpretation confidence is low"""
        # Generate clarification request
        request = self.generate_clarification_request(multi_modal_input)

        # Output request (speech, display, etc.)
        self.output_clarification_request(request)

    def generate_clarification_request(self, multi_modal_input: MultiModalInput) -> str:
        """Generate natural language clarification request"""
        request_parts = ["I'm not sure I understood correctly. Did you mean"]

        if multi_modal_input.speech:
            request_parts.append(f"to {multi_modal_input.speech.text}")

        if multi_modal_input.gesture:
            request_parts.append(f"to go to where you pointed")

        if multi_modal_input.vision:
            objects = [obj['name'] for obj in multi_modal_input.vision.objects[:2]]  # First 2 objects
            if objects:
                request_parts.append(f"to interact with the " + " or ".join(objects))

        return " or ".join(request_parts) + "?"

    def output_clarification_request(self, request: str):
        """Output clarification request to user"""
        # Publish to speech system
        msg = String()
        msg.data = request
        self.speech_publisher.publish(msg)

        print(f"Robot: {request}")

    def get_current_objects(self) -> List[Dict]:
        """Get currently detected objects"""
        # This would interface with current vision system
        # For now, returning empty list
        return []

    def focus_perception_on_object(self, object_name: str):
        """Focus perception system on specific object"""
        # Implementation would direct cameras, adjust focus, etc.
        pass

    def perform_general_perception(self):
        """Perform general scene perception"""
        # Implementation would scan environment, detect objects, etc.
        pass

    def convert_location_to_pose(self, location: Tuple) -> Pose:
        """Convert location coordinates to ROS Pose"""
        pose = Pose()
        if len(location) >= 3:
            pose.position = Point(x=location[0], y=location[1], z=location[2])
        else:
            pose.position = Point(x=location[0], y=location[1], z=0.0)
        return pose

    def create_navigation_goal(self, pose: Pose):
        """Create navigation goal message"""
        # Implementation depends on specific navigation system
        pass

    def create_manipulation_goal(self, object_info: Dict):
        """Create manipulation goal message"""
        # Implementation depends on specific manipulation system
        pass
```

## Real-time Processing and Optimization

### Efficient Multi-modal Pipeline

```python
import threading
import asyncio
from concurrent.futures import ThreadPoolExecutor

class EfficientMultiModalPipeline:
    def __init__(self):
        self.speech_processor = ContextAwareSpeechProcessor(None)
        self.gesture_recognizer = GestureRecognizer()
        self.vision_processor = VisionProcessor()
        self.fusion_engine = MultiModalFusionEngine()
        self.action_executor = MultiModalActionExecutor()

        # Threading for parallel processing
        self.executor = ThreadPoolExecutor(max_workers=4)
        self.speech_lock = threading.Lock()
        self.vision_lock = threading.Lock()

    def process_frame_async(self, frame: np.ndarray, audio_chunk: Optional[np.ndarray] = None):
        """Process video frame and audio chunk asynchronously"""
        # Process vision in parallel
        vision_future = self.executor.submit(self.vision_processor.process_scene, frame)

        # Process audio if available
        speech_future = None
        if audio_chunk is not None:
            speech_future = self.executor.submit(
                self.speech_processor.process_speech_with_context,
                audio_chunk
            )

        # Process gestures from frame
        gesture_future = self.executor.submit(self.gesture_recognizer.detect_gestures, frame)

        # Wait for results
        vision_result = vision_future.result()
        gesture_results = gesture_future.result()
        speech_result = speech_future.result() if speech_future else None

        # Synchronize and fuse results
        if gesture_results:
            # Use first detected gesture for now
            gesture_input = gesture_results[0]
        else:
            gesture_input = None

        multi_modal_input = MultiModalInput(
            speech=speech_result,
            gesture=gesture_input,
            vision=vision_result
        )

        # Fuse and execute
        return self.action_executor.process_multi_modal_command(multi_modal_input)

    def run_continuous_processing(self):
        """Run continuous multi-modal processing"""
        import cv2

        cap = cv2.VideoCapture(0)  # Camera input
        # audio_stream = initialize_audio_stream()  # Audio input

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    continue

                # Get audio chunk (simplified)
                audio_chunk = None  # get_audio_chunk()

                # Process frame asynchronously
                result = self.process_frame_async(frame, audio_chunk)

                # Display results
                cv2.imshow('Multi-modal Processing', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        finally:
            cap.release()
            cv2.destroyAllWindows()
```

Multi-modal interaction systems combine speech, gesture, and vision to create natural, intuitive human-robot communication that leverages the strengths of each modality while providing redundancy and disambiguation for robust interaction.