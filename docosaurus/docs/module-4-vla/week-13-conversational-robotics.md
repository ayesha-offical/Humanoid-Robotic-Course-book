---
title: Week 13 - Conversational Robotics and Vision-Language-Action Models
description: Integrate GPT models, voice commands, and Vision-Language-Action (VLA) models for embodied AI humanoids
sidebar_position: 3
hardware_tier: advanced
learning_objectives:
  - Integrate GPT-4 and voice models for conversational human-robot interaction
  - Implement Vision-Language-Action (VLA) models for embodied task execution
  - Design voice-to-action pipelines for natural language robot control
  - Deploy multimodal AI systems for real-world humanoid applications
estimated_time_minutes: 210
---

# Week 13: Conversational Robotics and Vision-Language-Action Models

## Introduction to Embodied AI

**Embodied AI** combines perception, language understanding, and physical action:
- **Vision**: Cameras for scene understanding
- **Language**: Natural language instructions and dialogue
- **Action**: Robotic manipulation and navigation

### Vision-Language-Action (VLA) Pipeline

```
User Speech → ASR → LLM (GPT-4) → Task Planner → VLA Model → Robot Actions
              ↑                                         ↓
           Camera Images ← ← ← ← ← ← ← ← ← ← ← Execution Feedback
```

## Conversational Interface with GPT-4

### Speech-to-Text with Whisper

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import numpy as np

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition')

        # Load Whisper model
        self.model = whisper.load_model("base")  # Options: tiny, base, small, medium, large

        # ROS 2 publisher for transcribed text
        self.text_pub = self.create_publisher(String, 'speech_to_text', 10)

        # Audio configuration
        self.CHUNK = 16000  # 1 second at 16kHz
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000

        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )

        # Timer to process audio
        self.timer = self.create_timer(1.0, self.process_audio)

        self.get_logger().info('Speech Recognition Node started')

    def process_audio(self):
        """Capture and transcribe audio"""
        try:
            # Read audio chunk
            audio_data = self.stream.read(self.CHUNK, exception_on_overflow=False)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0

            # Transcribe with Whisper
            result = self.model.transcribe(audio_np, language='en', fp16=False)
            text = result['text'].strip()

            if len(text) > 0:
                self.get_logger().info(f'Transcribed: "{text}"')

                # Publish to ROS 2 topic
                msg = String()
                msg.data = text
                self.text_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### GPT-4 Task Planner

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os

class GPTTaskPlanner(Node):
    def __init__(self):
        super().__init__('gpt_task_planner')

        # OpenAI API key
        openai.api_key = os.getenv('OPENAI_API_KEY')

        # Subscribe to speech-to-text
        self.text_sub = self.create_subscription(
            String,
            'speech_to_text',
            self.text_callback,
            10
        )

        # Publish task plans
        self.task_pub = self.create_publisher(String, 'robot_tasks', 10)

        # System prompt for robotics domain
        self.system_prompt = """You are an AI assistant controlling a humanoid robot with the following capabilities:
- Navigate to locations: navigate_to(location)
- Pick up objects: pick_up(object)
- Place objects: place_object(object, location)
- Open/close: manipulate_door(action)
- Speak: say(message)

Convert user requests into a sequence of these commands in JSON format.
Example:
User: "Bring me the red cup from the kitchen table"
Response: [
  {"action": "navigate_to", "args": {"location": "kitchen"}},
  {"action": "pick_up", "args": {"object": "red cup"}},
  {"action": "navigate_to", "args": {"location": "user"}},
  {"action": "place_object", "args": {"object": "red cup", "location": "hand"}},
  {"action": "say", "args": {"message": "Here is your red cup"}}
]"""

        self.conversation_history = []

        self.get_logger().info('GPT Task Planner started')

    def text_callback(self, msg):
        """Process user command with GPT-4"""
        user_text = msg.data

        # Add to conversation history
        self.conversation_history.append({"role": "user", "content": user_text})

        try:
            # Call GPT-4
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt}
                ] + self.conversation_history,
                temperature=0.3,
                max_tokens=500
            )

            assistant_reply = response.choices[0].message.content
            self.conversation_history.append({"role": "assistant", "content": assistant_reply})

            self.get_logger().info(f'GPT-4 Plan: {assistant_reply}')

            # Publish task plan
            task_msg = String()
            task_msg.data = assistant_reply
            self.task_pub.publish(task_msg)

        except Exception as e:
            self.get_logger().error(f'GPT-4 API error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GPTTaskPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Vision-Language-Action (VLA) Models

### VLA Architecture

VLA models map visual observations and language instructions directly to robot actions:

```
Visual Input (Camera) ──┐
                         ├──> VLA Model ──> Robot Actions (joint positions/velocities)
Language Input (Text) ──┘
```

**Popular VLA Models:**
- **RT-1** (Robotics Transformer 1, Google)
- **RT-2** (Robotics Transformer 2, Google)
- **PaLM-E** (Google)
- **OpenVLA** (Open-source)

### Implementing RT-1 Inference

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import torch
import numpy as np
from transformers import RT1ForConditionalGeneration, AutoProcessor

class VLAController(Node):
    def __init__(self):
        super().__init__('vla_controller')

        # Load RT-1 model (example - actual implementation may vary)
        self.model = RT1ForConditionalGeneration.from_pretrained("google/rt-1-x")
        self.processor = AutoProcessor.from_pretrained("google/rt-1-x")
        self.model.eval()

        # Move to GPU if available
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)

        # ROS 2 subscribers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )

        self.instruction_sub = self.create_subscription(
            String,
            'task_instruction',
            self.instruction_callback,
            10
        )

        # Current state
        self.bridge = CvBridge()
        self.current_image = None
        self.current_instruction = None

        # Control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('VLA Controller started')

    def image_callback(self, msg):
        """Store latest camera image"""
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

    def instruction_callback(self, msg):
        """Store latest task instruction"""
        self.current_instruction = msg.data
        self.get_logger().info(f'Received instruction: {self.current_instruction}')

    def control_loop(self):
        """Generate robot actions using VLA model"""
        if self.current_image is None or self.current_instruction is None:
            return

        try:
            # Preprocess inputs
            inputs = self.processor(
                images=self.current_image,
                text=self.current_instruction,
                return_tensors="pt"
            ).to(self.device)

            # Generate actions
            with torch.no_grad():
                outputs = self.model.generate(**inputs, max_new_tokens=100)

            # Decode actions
            actions = self.processor.decode(outputs[0], skip_special_tokens=True)

            # Parse and execute actions
            self.execute_actions(actions)

        except Exception as e:
            self.get_logger().error(f'VLA inference error: {e}')

    def execute_actions(self, actions):
        """
        Execute predicted actions on robot
        actions: String representation of robot commands
        """
        # Parse actions (format depends on VLA model output)
        # Example: "move_arm(x=0.3, y=0.2, z=0.5), open_gripper()"

        self.get_logger().info(f'Executing: {actions}')

        # Send to robot controller
        # (Implementation depends on robot interface)

def main(args=None):
    rclpy.init(args=args)
    node = VLAController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Fine-Tuning VLA for Custom Tasks

```python
import torch
from torch.utils.data import Dataset, DataLoader
from transformers import RT1ForConditionalGeneration, AutoProcessor
from tqdm import tqdm

class RobotDemonstrationDataset(Dataset):
    """Dataset of robot demonstrations (vision + language + actions)"""

    def __init__(self, data_path, processor):
        self.data = self.load_data(data_path)
        self.processor = processor

    def load_data(self, path):
        """Load demonstration data from disk"""
        # Format: List of (image, instruction, action_sequence)
        import pickle
        with open(path, 'rb') as f:
            return pickle.load(f)

    def __len__(self):
        return len(self.data)

    def __getitem__(self, idx):
        image, instruction, actions = self.data[idx]

        # Preprocess
        inputs = self.processor(
            images=image,
            text=instruction,
            return_tensors="pt"
        )

        # Tokenize action sequence
        action_tokens = self.processor.tokenizer(
            actions,
            return_tensors="pt",
            padding="max_length",
            truncation=True,
            max_length=128
        )

        return {
            'pixel_values': inputs['pixel_values'].squeeze(0),
            'input_ids': inputs['input_ids'].squeeze(0),
            'labels': action_tokens['input_ids'].squeeze(0)
        }

def fine_tune_vla(model_name, dataset_path, output_dir, epochs=10):
    """Fine-tune VLA model on custom robot data"""

    # Load model
    model = RT1ForConditionalGeneration.from_pretrained(model_name)
    processor = AutoProcessor.from_pretrained(model_name)

    # Prepare dataset
    dataset = RobotDemonstrationDataset(dataset_path, processor)
    dataloader = DataLoader(dataset, batch_size=8, shuffle=True)

    # Optimizer
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-5)

    # Training loop
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model.to(device)
    model.train()

    for epoch in range(epochs):
        total_loss = 0
        for batch in tqdm(dataloader, desc=f"Epoch {epoch+1}/{epochs}"):
            # Move to device
            pixel_values = batch['pixel_values'].to(device)
            input_ids = batch['input_ids'].to(device)
            labels = batch['labels'].to(device)

            # Forward pass
            outputs = model(
                pixel_values=pixel_values,
                input_ids=input_ids,
                labels=labels
            )

            loss = outputs.loss

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch+1} - Loss: {avg_loss:.4f}")

    # Save fine-tuned model
    model.save_pretrained(output_dir)
    processor.save_pretrained(output_dir)
    print(f"Model saved to {output_dir}")

# Example usage
# fine_tune_vla(
#     model_name="google/rt-1-x",
#     dataset_path="/data/robot_demos.pkl",
#     output_dir="/models/rt1_finetuned",
#     epochs=10
# )
```

## Text-to-Speech for Robot Responses

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyttsx3
import threading

class TextToSpeechNode(Node):
    def __init__(self):
        super().__init__('text_to_speech')

        # Initialize TTS engine
        self.engine = pyttsx3.init()
        self.engine.setProperty('rate', 150)  # Speaking rate
        self.engine.setProperty('volume', 0.9)

        # Subscribe to robot speech commands
        self.speech_sub = self.create_subscription(
            String,
            'robot_speech',
            self.speech_callback,
            10
        )

        self.get_logger().info('Text-to-Speech Node started')

    def speech_callback(self, msg):
        """Convert text to speech"""
        text = msg.data
        self.get_logger().info(f'Speaking: "{text}"')

        # Run in separate thread to avoid blocking
        thread = threading.Thread(target=self._speak, args=(text,))
        thread.start()

    def _speak(self, text):
        """Actual speech synthesis"""
        self.engine.say(text)
        self.engine.runAndWait()

def main(args=None):
    rclpy.init(args=args)
    node = TextToSpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Complete Voice-to-Action Pipeline

### Integration Launch File

```python
# launch/conversational_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Speech Recognition (Whisper)
        Node(
            package='conversational_robot',
            executable='speech_recognition_node',
            name='speech_recognition',
            output='screen'
        ),

        # GPT-4 Task Planner
        Node(
            package='conversational_robot',
            executable='gpt_task_planner',
            name='gpt_planner',
            output='screen'
        ),

        # VLA Controller
        Node(
            package='conversational_robot',
            executable='vla_controller',
            name='vla_controller',
            output='screen'
        ),

        # Text-to-Speech
        Node(
            package='conversational_robot',
            executable='text_to_speech_node',
            name='tts',
            output='screen'
        ),

        # Camera driver
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'color_width': 640,
                'color_height': 480,
                'depth_width': 640,
                'depth_height': 480,
                'fps': 30
            }]
        )
    ])
```

**Run the system:**
```bash
ros2 launch conversational_robot conversational_robot.launch.py
```

## Real-World Deployment Example

### Household Assistance Robot

```python
class HouseholdAssistantRobot:
    """Complete system for household task execution"""

    def __init__(self):
        # Initialize all components
        self.speech_recognition = SpeechRecognitionNode()
        self.task_planner = GPTTaskPlanner()
        self.vla_controller = VLAController()
        self.tts = TextToSpeechNode()

        # Object detection
        self.object_detector = ObjectDetector()  # YOLOv8 or similar

        # Navigation
        self.navigator = NavigationClient()  # Nav2

    def execute_task(self, user_command):
        """
        Execute a household task from natural language
        Example: "Please bring me a glass of water from the kitchen"
        """
        # 1. Speech to text
        transcribed_text = self.speech_recognition.transcribe(user_command)

        # 2. Task planning with GPT-4
        task_plan = self.task_planner.generate_plan(transcribed_text)

        # 3. Execute each step
        for step in task_plan:
            action = step['action']
            args = step['args']

            if action == 'navigate_to':
                self.navigator.navigate_to(args['location'])
                self.tts.speak(f"Navigating to {args['location']}")

            elif action == 'pick_up':
                # Use VLA for manipulation
                self.vla_controller.grasp_object(args['object'])
                self.tts.speak(f"Picking up {args['object']}")

            elif action == 'place_object':
                self.vla_controller.place_object(args['object'], args['location'])
                self.tts.speak(f"Placing {args['object']}")

            elif action == 'say':
                self.tts.speak(args['message'])

        self.tts.speak("Task completed!")

# Usage
# robot = HouseholdAssistantRobot()
# robot.execute_task("Bring me a glass of water from the kitchen")
```

## Key Takeaways

1. **Whisper** provides accurate speech-to-text for robot voice interfaces
2. **GPT-4** enables high-level task planning from natural language
3. **Vision-Language-Action (VLA) models** directly map observations to robot actions
4. **Fine-tuning VLA models** on custom demonstrations improves task-specific performance
5. **End-to-end pipeline** integrates speech, vision, language, and action for embodied AI

## Course Completion

Congratulations! You have completed the **Physical AI & Humanoid Robotics** course. You now have the skills to:
- Build ROS 2 systems for modular robotics
- Simulate robots in Gazebo and Isaac Sim
- Deploy VSLAM and navigation stacks
- Apply sim-to-real transfer techniques
- Control humanoid robots with locomotion and manipulation
- Integrate conversational AI for natural human-robot interaction

### Next Steps for Continued Learning

1. **Contribute to open-source**: Isaac ROS, ROS 2 packages
2. **Build a personal project**: Tabletop manipulator, mobile robot, or humanoid simulator
3. **Explore research papers**: RSS, ICRA, CoRL conferences
4. **Join robotics communities**: ROS Discourse, NVIDIA Isaac forums, Discord servers
5. **Hardware projects**: Deploy on real robots (TurtleBot, Unitree, Trossen)

---

**References:**
- [RT-1: Robotics Transformer](https://arxiv.org/abs/2212.06817)
- [RT-2: Vision-Language-Action Models](https://arxiv.org/abs/2307.15818)
- [PaLM-E: Embodied Multimodal Language Model](https://arxiv.org/abs/2303.03378)
- [Whisper: Robust Speech Recognition](https://arxiv.org/abs/2212.04356)
