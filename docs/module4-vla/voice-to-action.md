# Voice-to-Action with OpenAI Whisper

## Introduction to Voice-to-Action Systems

Voice-to-action systems enable humanoid robots to understand spoken commands and execute corresponding physical actions. This technology represents a significant advancement in human-robot interaction, allowing users to communicate with robots using natural language rather than specialized interfaces or programming.

## OpenAI Whisper for Speech Recognition

### Overview of Whisper

OpenAI Whisper is a state-of-the-art automatic speech recognition (ASR) system trained on 680,000 hours of multilingual and multitask supervised data. It demonstrates robust performance across various domains and languages, making it ideal for humanoid robotics applications.

### Whisper Architecture

Whisper uses a multitask decoder architecture:
- **Encoder**: Processes audio input using a 30-second context window
- **Decoder**: Generates text tokens with language identification
- **Multilingual capability**: Trained on 99 languages
- **Robustness**: Performs well with background noise and accents

### Whisper Models

Different model sizes offer trade-offs between accuracy and computational requirements:
- **tiny**: 39M parameters, suitable for edge devices
- **base**: 74M parameters, good balance of speed and accuracy
- **small**: 244M parameters, higher accuracy
- **medium**: 769M parameters, very high accuracy
- **large**: 1550M parameters, highest accuracy

## Implementing Whisper for Robotics

### Basic Whisper Integration

```python
import whisper
import torch
import numpy as np
from scipy.io import wavfile

class VoiceToActionSystem:
    def __init__(self, model_size="medium"):
        # Load Whisper model
        self.model = whisper.load_model(model_size)

        # Define robot command vocabulary
        self.command_keywords = [
            "move forward", "move backward", "turn left", "turn right",
            "pick up", "put down", "stop", "go", "help", "wait"
        ]

        # Initialize ROS 2 publisher for robot commands
        self.robot_command_publisher = self.initialize_robot_publisher()

    def transcribe_audio(self, audio_path):
        # Load and transcribe audio using Whisper
        result = self.model.transcribe(audio_path)
        return result["text"]

    def process_audio_stream(self, audio_data):
        # Process streaming audio data
        # Convert to appropriate format for Whisper
        audio_np = self.preprocess_audio(audio_data)

        # Transcribe the audio
        transcription = self.model.transcribe(audio_np)
        return transcription["text"]

    def preprocess_audio(self, audio_data):
        # Convert audio to format expected by Whisper
        # Whisper expects 16kHz mono audio
        if len(audio_data.shape) > 1:
            # Convert stereo to mono
            audio_data = np.mean(audio_data, axis=1)

        # Ensure proper sample rate (Whisper expects 16kHz)
        # Resample if necessary
        return audio_data
```

### Real-time Audio Processing

```python
import pyaudio
import threading
import queue

class RealTimeVoiceProcessor:
    def __init__(self, whisper_system):
        self.whisper_system = whisper_system
        self.audio_queue = queue.Queue()

        # Audio stream parameters
        self.format = pyaudio.paInt16
        self.channels = 1
        self.rate = 16000  # Whisper expects 16kHz
        self.chunk = 1024

        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.recording = False

    def start_listening(self):
        # Start audio recording in a separate thread
        self.stream = self.audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.rate,
            input=True,
            frames_per_buffer=self.chunk
        )

        self.recording = True
        recording_thread = threading.Thread(target=self.record_audio)
        recording_thread.start()

    def record_audio(self):
        # Continuously record audio and add to queue
        while self.recording:
            data = self.stream.read(self.chunk)
            self.audio_queue.put(data)

    def process_voice_commands(self):
        # Process audio chunks and detect commands
        audio_buffer = []

        while True:
            try:
                # Get audio chunk from queue
                chunk = self.audio_queue.get(timeout=0.1)
                audio_buffer.append(chunk)

                # Process every 2 seconds of audio
                if len(audio_buffer) * self.chunk >= self.rate * 2:
                    # Convert buffer to numpy array
                    audio_np = np.frombuffer(b''.join(audio_buffer), dtype=np.int16)
                    audio_np = audio_np.astype(np.float32) / 32768.0  # Normalize

                    # Transcribe and process
                    text = self.whisper_system.process_audio_stream(audio_np)
                    self.handle_command(text)

                    # Clear buffer
                    audio_buffer = []

            except queue.Empty:
                continue

    def handle_command(self, text):
        # Process transcribed text and execute robot commands
        print(f"Recognized: {text}")

        # Extract command from text
        command = self.extract_robot_command(text)

        if command:
            print(f"Executing command: {command}")
            self.execute_robot_command(command)

    def extract_robot_command(self, text):
        # Simple keyword-based command extraction
        text_lower = text.lower()

        for keyword in self.whisper_system.command_keywords:
            if keyword in text_lower:
                return keyword

        return None
```

## Speech Recognition Optimization for Robotics

### Noise Reduction and Filtering

```python
import webrtcvad
from scipy import signal

class RobustSpeechProcessor:
    def __init__(self):
        # Voice Activity Detection
        self.vad = webrtcvad.Vad()
        self.vad.set_mode(1)  # Aggressiveness mode (0-3)

    def preprocess_for_robot_environment(self, audio_data, sample_rate=16000):
        # Apply noise reduction for robot environments
        # Remove background noise from motors, fans, etc.

        # Apply high-pass filter to remove low-frequency noise
        b, a = signal.butter(4, 100 / (sample_rate / 2), btype='high')
        filtered_audio = signal.filtfilt(b, a, audio_data)

        # Apply spectral subtraction for noise reduction
        enhanced_audio = self.spectral_subtraction(filtered_audio)

        return enhanced_audio

    def spectral_subtraction(self, audio_data):
        # Simple spectral subtraction noise reduction
        # Calculate noise profile from initial silence
        noise_profile = self.estimate_noise_profile(audio_data[:8000])  # First 0.5s

        # Apply spectral subtraction
        enhanced = audio_data - noise_profile
        return enhanced

    def estimate_noise_profile(self, initial_audio):
        # Estimate noise profile from initial audio segment
        # Assumes initial segment contains mostly noise
        return np.mean(initial_audio) * 0.1  # Simplified estimation
```

### Context-Aware Recognition

```python
class ContextAwareRecognizer:
    def __init__(self, whisper_system):
        self.whisper_system = whisper_system
        self.current_context = "navigation"  # Default context

        # Context-specific command weights
        self.context_commands = {
            "navigation": ["move", "turn", "go", "stop", "forward", "backward", "left", "right"],
            "manipulation": ["pick", "put", "grasp", "release", "lift", "place"],
            "interaction": ["hello", "help", "wait", "follow", "come", "stop"]
        }

    def recognize_with_context(self, audio_data):
        # Transcribe audio with context-aware processing
        transcription = self.whisper_system.process_audio_stream(audio_data)

        # Apply context-specific command extraction
        command = self.extract_command_with_context(transcription, self.current_context)
        return command

    def extract_command_with_context(self, text, context):
        # Prioritize commands based on current context
        context_keywords = self.context_commands.get(context, [])

        # Check context-specific keywords first
        for keyword in context_keywords:
            if keyword.lower() in text.lower():
                return keyword

        # Fall back to general command recognition
        return self.whisper_system.extract_robot_command(text)
```

## Integration with ROS 2

### ROS 2 Node for Voice Processing

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import AudioData

class VoiceToActionNode(Node):
    def __init__(self):
        super().__init__('voice_to_action_node')

        # Initialize Whisper system
        self.whisper_system = VoiceToActionSystem(model_size="base")

        # Publishers for robot commands
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speech_publisher = self.create_publisher(String, '/robot_speech', 10)

        # Subscriber for audio input
        self.audio_subscription = self.create_subscription(
            AudioData,
            '/audio_input',
            self.audio_callback,
            10
        )

        # Timer for processing
        self.process_timer = self.create_timer(0.1, self.process_commands)

        self.pending_commands = []

    def audio_callback(self, msg):
        # Process incoming audio data
        try:
            # Convert audio message to numpy array
            audio_np = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0

            # Transcribe audio
            text = self.whisper_system.process_audio_stream(audio_np)

            # Extract and queue command
            command = self.extract_robot_command(text)
            if command:
                self.pending_commands.append(command)

        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')

    def process_commands(self):
        # Process pending voice commands
        while self.pending_commands:
            command = self.pending_commands.pop(0)
            self.execute_robot_command(command)

    def extract_robot_command(self, text):
        # Extract command from transcribed text
        return self.whisper_system.extract_robot_command(text)

    def execute_robot_command(self, command):
        # Execute robot command based on voice input
        twist = Twist()

        if "forward" in command or "go" in command:
            twist.linear.x = 0.5  # Move forward at 0.5 m/s
        elif "backward" in command:
            twist.linear.x = -0.5  # Move backward
        elif "left" in command:
            twist.angular.z = 0.5  # Turn left
        elif "right" in command:
            twist.angular.z = -0.5  # Turn right
        elif "stop" in command:
            # Stop movement (twist is already zero)
            pass
        else:
            self.get_logger().info(f'Unknown command: {command}')
            return

        # Publish command
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Executed command: {command}')
```

## Performance Optimization

### Model Quantization and Optimization

```python
class OptimizedWhisperSystem:
    def __init__(self, model_size="small"):
        # Load model and apply optimizations
        self.model = whisper.load_model(model_size)

        # Apply quantization for faster inference
        self.model = self.apply_quantization(self.model)

        # Set model to evaluation mode
        self.model.eval()

    def apply_quantization(self, model):
        # Apply dynamic quantization to reduce model size
        import torch.quantization as quantization

        # Quantize the model
        quantized_model = quantization.quantize_dynamic(
            model, {torch.nn.Linear}, dtype=torch.qint8
        )

        return quantized_model

    def optimize_inference(self, audio_data):
        # Optimize inference with torch.jit
        with torch.no_grad():
            result = self.model.transcribe(audio_data)
        return result
```

## Error Handling and Robustness

### Confidence Scoring and Validation

```python
class RobustVoiceToAction:
    def __init__(self, whisper_system):
        self.whisper_system = whisper_system
        self.confidence_threshold = 0.7  # Minimum confidence for command execution

    def process_with_confidence(self, audio_data):
        # Get transcription with confidence scoring
        result = self.whisper_system.model.transcribe(
            audio_data,
            return_segments=True
        )

        # Calculate overall confidence
        confidence = self.calculate_transcription_confidence(result)

        if confidence >= self.confidence_threshold:
            command = self.extract_robot_command(result["text"])
            return command, confidence
        else:
            return None, confidence

    def calculate_transcription_confidence(self, result):
        # Calculate confidence based on transcription quality
        # This is a simplified approach - actual implementation would use
        # more sophisticated confidence measures
        text = result["text"]

        # Check for common error patterns
        error_indicators = ["you know", "um", "uh", "like"]
        error_count = sum(1 for indicator in error_indicators if indicator in text.lower())

        # Calculate confidence based on text length and error patterns
        confidence = min(1.0, len(text) / 100.0)  # Longer text = higher confidence
        confidence = max(0.0, confidence - error_count * 0.1)  # Penalize error indicators

        return confidence

    def handle_uncertain_recognition(self, audio_data):
        # Handle cases where recognition confidence is low
        command, confidence = self.process_with_confidence(audio_data)

        if confidence < self.confidence_threshold:
            # Request clarification or ignore command
            self.request_clarification()
            return None
        else:
            return command
```

## Practical Implementation Considerations

### Latency Optimization

For real-time voice-to-action systems in robotics, latency is critical:

- **Audio buffering**: Use appropriate buffer sizes to balance latency and accuracy
- **Model selection**: Choose model size based on hardware capabilities
- **Parallel processing**: Process audio and execute commands in parallel
- **Command queuing**: Queue commands for smooth execution

### Privacy and Security

When implementing voice systems in robotics:

- **Local processing**: Process audio locally when possible to protect privacy
- **Data encryption**: Encrypt audio data in transit
- **Access controls**: Implement proper authentication for voice commands
- **Data retention**: Clear audio data after processing

Voice-to-action systems with OpenAI Whisper enable natural, intuitive interaction with humanoid robots, bridging the gap between human communication and robotic action execution.