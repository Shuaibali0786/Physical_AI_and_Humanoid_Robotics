# Cognitive Planning with LLMs to ROS 2 Actions

## Introduction to Cognitive Planning

Cognitive planning in robotics involves high-level reasoning that bridges natural language commands with low-level robot actions. Large Language Models (LLMs) have revolutionized this field by enabling robots to understand complex, multi-step instructions and generate appropriate sequences of ROS 2 actions to achieve user goals.

## LLM Integration for Robotics

### Overview of LLM Capabilities

Large Language Models like GPT-4, Claude, and open-source alternatives provide several key capabilities for robotic planning:

- **Natural language understanding**: Parse complex, ambiguous, or context-dependent commands
- **Reasoning and inference**: Plan multi-step sequences and handle contingencies
- **Knowledge integration**: Access world knowledge to inform planning decisions
- **Adaptability**: Handle novel situations and commands not explicitly programmed

### LLM Selection Criteria

When choosing an LLM for robotic cognitive planning:

- **Response time**: Critical for real-time interaction
- **Cost considerations**: Balance performance with operational costs
- **Reliability**: Consistent performance for safety-critical applications
- **Customization**: Ability to fine-tune for specific robotic tasks
- **Privacy**: Local models vs. cloud-based services

## Cognitive Planning Architecture

### Planning Pipeline

```python
import json
import re
from typing import List, Dict, Any
from dataclasses import dataclass

@dataclass
class RobotAction:
    """Represents a single robot action"""
    action_type: str
    parameters: Dict[str, Any]
    description: str

class CognitivePlanner:
    def __init__(self, llm_client, robot_capabilities):
        self.llm_client = llm_client
        self.robot_capabilities = robot_capabilities
        self.action_history = []

    def plan_from_command(self, user_command: str) -> List[RobotAction]:
        """Generate action sequence from natural language command"""
        # 1. Parse the command using LLM
        parsed_intent = self.parse_command_intent(user_command)

        # 2. Generate plan using LLM and robot knowledge
        action_sequence = self.generate_action_plan(parsed_intent, user_command)

        # 3. Validate plan against robot capabilities
        validated_plan = self.validate_plan(action_sequence)

        return validated_plan

    def parse_command_intent(self, command: str) -> Dict[str, Any]:
        """Parse user command to extract intent and parameters"""
        prompt = f"""
        Analyze the following robot command and extract the intent and parameters:
        Command: "{command}"

        Return a JSON object with:
        - "action": Primary action to perform
        - "target_object": Object to interact with (if any)
        - "location": Target location (if any)
        - "parameters": Additional parameters

        Example:
        {{
            "action": "navigate_to",
            "target_object": "red cup",
            "location": "kitchen table",
            "parameters": {{"speed": "medium"}}
        }}
        """

        response = self.llm_client.generate(prompt)
        return json.loads(response)

    def generate_action_plan(self, intent: Dict[str, Any], original_command: str) -> List[RobotAction]:
        """Generate detailed action sequence from parsed intent"""
        prompt = f"""
        Given the following user intent and robot capabilities, generate a detailed action sequence:

        User Intent: {json.dumps(intent)}
        Robot Capabilities: {json.dumps(self.robot_capabilities)}

        Generate a sequence of specific robot actions that will accomplish the user's goal.
        Each action should be executable by the robot's ROS 2 interface.

        Return as a list of JSON objects with:
        - "action_type": Type of action (e.g., "navigate_to", "pick_object", "place_object")
        - "parameters": Parameters for the action
        - "description": Human-readable description

        Example:
        [
            {{
                "action_type": "navigate_to",
                "parameters": {{"location": "kitchen"}},
                "description": "Move to the kitchen area"
            }},
            {{
                "action_type": "find_object",
                "parameters": {{"object": "red cup"}},
                "description": "Locate the red cup"
            }}
        ]
        """

        response = self.llm_client.generate(prompt)
        action_list = json.loads(response)

        # Convert to RobotAction objects
        actions = []
        for action_dict in action_list:
            action = RobotAction(
                action_type=action_dict["action_type"],
                parameters=action_dict["parameters"],
                description=action_dict["description"]
            )
            actions.append(action)

        return actions

    def validate_plan(self, action_sequence: List[RobotAction]) -> List[RobotAction]:
        """Validate action sequence against robot capabilities"""
        validated_actions = []

        for action in action_sequence:
            if self.is_action_valid(action):
                validated_actions.append(action)
            else:
                # Handle invalid action
                corrected_action = self.handle_invalid_action(action)
                if corrected_action:
                    validated_actions.append(corrected_action)

        return validated_actions

    def is_action_valid(self, action: RobotAction) -> bool:
        """Check if action is supported by robot"""
        return action.action_type in self.robot_capabilities

    def handle_invalid_action(self, action: RobotAction) -> RobotAction:
        """Handle invalid actions by suggesting alternatives"""
        # This could involve LLM-based alternative generation
        # or fallback to safe behaviors
        print(f"Warning: Invalid action '{action.action_type}'")
        return None
```

### LLM Client Implementation

```python
import openai
import time
from typing import Optional

class LLMClient:
    def __init__(self, api_key: str, model: str = "gpt-4"):
        self.api_key = api_key
        self.model = model
        openai.api_key = api_key

    def generate(self, prompt: str, max_retries: int = 3) -> str:
        """Generate response from LLM with error handling"""
        for attempt in range(max_retries):
            try:
                response = openai.ChatCompletion.create(
                    model=self.model,
                    messages=[{"role": "user", "content": prompt}],
                    temperature=0.1,  # Low temperature for consistency
                    max_tokens=1000
                )
                return response.choices[0].message.content.strip()

            except openai.error.RateLimitError:
                print(f"Rate limit reached, waiting... (attempt {attempt + 1})")
                time.sleep(2 ** attempt)  # Exponential backoff
            except openai.error.APIError as e:
                print(f"API Error: {e}")
                time.sleep(2 ** attempt)
            except Exception as e:
                print(f"Unexpected error: {e}")
                break

        raise Exception(f"Failed to get response after {max_retries} attempts")

    def generate_with_validation(self, prompt: str, expected_format: str) -> Optional[Dict]:
        """Generate response and validate format"""
        response = self.generate(prompt)

        # Validate JSON format
        try:
            result = json.loads(response)
            if self.validate_format(result, expected_format):
                return result
        except json.JSONDecodeError:
            pass

        # If format is invalid, request correction
        correction_prompt = f"""
        The previous response was not in the expected JSON format.
        Expected format: {expected_format}
        Response: {response}

        Please return the response in the correct JSON format.
        """
        corrected_response = self.generate(correction_prompt)
        return json.loads(corrected_response)

    def validate_format(self, result: Dict, expected_format: str) -> bool:
        """Validate response format"""
        # Implement format validation logic
        # This is a simplified example
        return isinstance(result, dict)
```

## ROS 2 Action Integration

### Action Mapping System

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import String

class ROS2ActionExecutor(Node):
    def __init__(self):
        super().__init__('cognitive_planner_executor')

        # ROS 2 action clients
        self.move_base_client = ActionClient(self, MoveBaseGoal, 'move_base')
        self.manipulation_client = ActionClient(self, 'ManipulationGoal', 'manipulation')

        # Publishers
        self.speech_publisher = self.create_publisher(String, 'robot_speech', 10)

    def execute_action(self, action: RobotAction) -> bool:
        """Execute a single robot action"""
        try:
            if action.action_type == "navigate_to":
                return self.execute_navigation_action(action)
            elif action.action_type == "pick_object":
                return self.execute_manipulation_action(action, "pick")
            elif action.action_type == "place_object":
                return self.execute_manipulation_action(action, "place")
            elif action.action_type == "find_object":
                return self.execute_perception_action(action)
            else:
                self.get_logger().warn(f"Unknown action type: {action.action_type}")
                return False

        except Exception as e:
            self.get_logger().error(f"Error executing action {action.action_type}: {e}")
            return False

    def execute_navigation_action(self, action: RobotAction) -> bool:
        """Execute navigation action"""
        location = action.parameters.get("location", "default")
        target_pose = self.get_pose_for_location(location)

        if not target_pose:
            self.get_logger().error(f"Unknown location: {location}")
            return False

        # Send navigation goal
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.header.frame_id = "map"
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target_pose.pose = target_pose

        self.move_base_client.wait_for_server()
        future = self.move_base_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def execute_manipulation_action(self, action: RobotAction, manipulation_type: str) -> bool:
        """Execute manipulation action"""
        # Implementation depends on specific manipulation capabilities
        # This is a simplified example
        object_name = action.parameters.get("object", "")

        goal_msg = {
            "type": manipulation_type,
            "object": object_name,
            "location": action.parameters.get("location", "default")
        }

        # Send manipulation goal
        self.manipulation_client.wait_for_server()
        future = self.manipulation_client.send_goal(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success

    def get_pose_for_location(self, location_name: str) -> Optional[Pose]:
        """Get predefined pose for location name"""
        # This would typically come from a map or database
        location_poses = {
            "kitchen": Pose(position=Point(x=1.0, y=2.0, z=0.0)),
            "living_room": Pose(position=Point(x=3.0, y=1.0, z=0.0)),
            "bedroom": Pose(position=Point(x=0.0, y=5.0, z=0.0))
        }

        return location_poses.get(location_name)
```

## Context-Aware Planning

### World Model Integration

```python
class WorldModel:
    def __init__(self):
        self.objects = {}
        self.locations = {}
        self.robot_state = {}
        self.update_timestamp = time.time()

    def update_object_location(self, object_id: str, location: str):
        """Update object location in world model"""
        if object_id not in self.objects:
            self.objects[object_id] = {}
        self.objects[object_id]["location"] = location
        self.objects[object_id]["last_seen"] = time.time()

    def get_object_location(self, object_id: str) -> Optional[str]:
        """Get current location of object"""
        obj = self.objects.get(object_id)
        if obj and time.time() - obj.get("last_seen", 0) < 300:  # 5 minutes
            return obj.get("location")
        return None

    def update_robot_location(self, location: str):
        """Update robot location in world model"""
        self.robot_state["location"] = location
        self.robot_state["last_updated"] = time.time()

    def get_robot_location(self) -> Optional[str]:
        """Get current robot location"""
        return self.robot_state.get("location")

class ContextAwarePlanner(CognitivePlanner):
    def __init__(self, llm_client, robot_capabilities, world_model):
        super().__init__(llm_client, robot_capabilities)
        self.world_model = world_model

    def generate_action_plan(self, intent: Dict[str, Any], original_command: str) -> List[RobotAction]:
        """Generate action plan with context awareness"""
        # Get current world state
        current_state = self.get_current_world_state()

        # Include context in planning prompt
        prompt = f"""
        Given the following user intent, robot capabilities, and current world state,
        generate a detailed action sequence:

        User Intent: {json.dumps(intent)}
        Robot Capabilities: {json.dumps(self.robot_capabilities)}
        Current World State: {json.dumps(current_state)}

        Consider the current state when planning:
        - Object locations may have changed
        - Robot may already be in a favorable position
        - Recent actions may affect the plan

        Generate a sequence of specific robot actions that will accomplish the user's goal.
        Return as a list of JSON objects with "action_type", "parameters", and "description".
        """

        response = self.llm_client.generate(prompt)
        action_list = json.loads(response)

        # Convert to RobotAction objects
        actions = []
        for action_dict in action_list:
            action = RobotAction(
                action_type=action_dict["action_type"],
                parameters=action_dict["parameters"],
                description=action_dict["description"]
            )
            actions.append(action)

        return actions

    def get_current_world_state(self) -> Dict[str, Any]:
        """Get current state of world model"""
        return {
            "objects": self.world_model.objects,
            "robot_location": self.world_model.get_robot_location(),
            "last_update": self.world_model.update_timestamp
        }
```

## Safety and Error Handling

### Plan Validation and Safety Checks

```python
class SafeCognitivePlanner(ContextAwarePlanner):
    def __init__(self, llm_client, robot_capabilities, world_model):
        super().__init__(llm_client, robot_capabilities, world_model)
        self.safety_constraints = self.define_safety_constraints()

    def define_safety_constraints(self):
        """Define safety constraints for planning"""
        return {
            "no_go_zones": ["staircase", "construction_area"],
            "speed_limits": {"navigation": 0.5, "manipulation": 0.1},
            "force_limits": {"gripping": 50.0},  # Newtons
            "timeouts": {"navigation": 60.0, "manipulation": 120.0}
        }

    def validate_plan(self, action_sequence: List[RobotAction]) -> List[RobotAction]:
        """Validate action sequence with safety checks"""
        validated_actions = []

        for action in action_sequence:
            # Check safety constraints
            if self.is_action_safe(action):
                validated_actions.append(action)
            else:
                # Handle unsafe action
                safe_alternative = self.generate_safe_alternative(action)
                if safe_alternative:
                    validated_actions.append(safe_alternative)
                else:
                    # Skip unsafe action and continue
                    continue

        return validated_actions

    def is_action_safe(self, action: RobotAction) -> bool:
        """Check if action is safe to execute"""
        if action.action_type == "navigate_to":
            location = action.parameters.get("location", "")
            if location in self.safety_constraints["no_go_zones"]:
                return False

        elif action.action_type == "manipulate_object":
            force = action.parameters.get("force", 0.0)
            max_force = self.safety_constraints["force_limits"]["gripping"]
            if force > max_force:
                return False

        return True

    def generate_safe_alternative(self, action: RobotAction) -> Optional[RobotAction]:
        """Generate safe alternative to unsafe action"""
        if action.action_type == "navigate_to":
            # Suggest safe alternative route
            original_location = action.parameters.get("location", "")
            safe_location = self.find_safe_alternative_location(original_location)
            if safe_location:
                return RobotAction(
                    action_type="navigate_to",
                    parameters={"location": safe_location},
                    description=f"Navigate to safe alternative location: {safe_location}"
                )

        elif action.action_type == "manipulate_object":
            # Reduce force to safe level
            safe_force = self.safety_constraints["force_limits"]["gripping"]
            action.parameters["force"] = safe_force
            action.description += " (force limited for safety)"

        return action

    def find_safe_alternative_location(self, original_location: str) -> Optional[str]:
        """Find safe alternative to dangerous location"""
        # Implementation would find nearby safe locations
        # This is a simplified example
        safe_alternatives = {
            "staircase": "nearby hallway",
            "construction_area": "adjacent room"
        }
        return safe_alternatives.get(original_location)
```

## Multi-Modal Integration

### Combining Vision and Language Planning

```python
class MultiModalCognitivePlanner(SafeCognitivePlanner):
    def __init__(self, llm_client, robot_capabilities, world_model, vision_system):
        super().__init__(llm_client, robot_capabilities, world_model)
        self.vision_system = vision_system

    def generate_action_plan(self, intent: Dict[str, Any], original_command: str) -> List[RobotAction]:
        """Generate action plan with multi-modal input"""
        # Get visual information
        visual_info = self.vision_system.get_current_scene_description()

        # Include visual context in planning
        prompt = f"""
        Given the following user intent, robot capabilities, current world state, and visual scene,
        generate a detailed action sequence:

        User Intent: {json.dumps(intent)}
        Robot Capabilities: {json.dumps(self.robot_capabilities)}
        Current World State: {json.dumps(self.get_current_world_state())}
        Visual Scene: {json.dumps(visual_info)}

        Use visual information to refine the plan:
        - Confirm object existence and location
        - Identify potential obstacles
        - Adapt plan based on actual scene

        Generate a sequence of specific robot actions that will accomplish the user's goal.
        Return as a list of JSON objects with "action_type", "parameters", and "description".
        """

        response = self.llm_client.generate(prompt)
        action_list = json.loads(response)

        # Convert to RobotAction objects
        actions = []
        for action_dict in action_list:
            action = RobotAction(
                action_type=action_dict["action_type"],
                parameters=action_dict["parameters"],
                description=action_dict["description"]
            )
            actions.append(action)

        return actions

    def validate_with_vision(self, action: RobotAction) -> bool:
        """Validate action using visual feedback"""
        if action.action_type == "navigate_to":
            target_location = action.parameters.get("location", "")
            # Check if path is clear using vision
            obstacles = self.vision_system.detect_obstacles_to_location(target_location)
            return len(obstacles) == 0

        elif action.action_type == "pick_object":
            target_object = action.parameters.get("object", "")
            # Check if object is visible and reachable
            object_info = self.vision_system.get_object_info(target_object)
            return object_info is not None and object_info["reachable"]

        return True
```

## Performance Optimization

### Caching and Plan Reuse

```python
import hashlib
from datetime import datetime, timedelta

class OptimizedCognitivePlanner(MultiModalCognitivePlanner):
    def __init__(self, llm_client, robot_capabilities, world_model, vision_system):
        super().__init__(llm_client, robot_capabilities, world_model, vision_system)
        self.plan_cache = {}
        self.cache_ttl = timedelta(minutes=30)

    def plan_from_command(self, user_command: str) -> List[RobotAction]:
        """Generate plan with caching"""
        # Create cache key
        cache_key = hashlib.md5(user_command.encode()).hexdigest()

        # Check cache
        cached_result = self.get_cached_plan(cache_key, user_command)
        if cached_result:
            print("Using cached plan")
            return cached_result

        # Generate new plan
        plan = super().plan_from_command(user_command)

        # Cache the plan
        self.cache_plan(cache_key, user_command, plan)

        return plan

    def get_cached_plan(self, cache_key: str, command: str) -> Optional[List[RobotAction]]:
        """Get cached plan if still valid"""
        cached = self.plan_cache.get(cache_key)
        if cached:
            plan, timestamp, original_command = cached
            if datetime.now() - timestamp < self.cache_ttl:
                # Verify plan is still valid given current state
                if self.is_plan_still_valid(plan, original_command):
                    return plan

        return None

    def cache_plan(self, cache_key: str, command: str, plan: List[RobotAction]):
        """Cache generated plan"""
        self.plan_cache[cache_key] = (plan, datetime.now(), command)

    def is_plan_still_valid(self, plan: List[RobotAction], original_command: str) -> bool:
        """Check if cached plan is still valid"""
        # Check if world state has changed significantly
        current_state = self.get_current_world_state()
        # Implementation would compare current state with state when plan was made
        return True  # Simplified for example
```

## Real-World Deployment Considerations

### Handling Uncertainty and Ambiguity

```python
class RobustCognitivePlanner(OptimizedCognitivePlanner):
    def __init__(self, llm_client, robot_capabilities, world_model, vision_system):
        super().__init__(llm_client, robot_capabilities, world_model, vision_system)
        self.confidence_threshold = 0.7

    def handle_ambiguous_command(self, command: str) -> List[RobotAction]:
        """Handle ambiguous or unclear commands"""
        # Ask clarifying questions using LLM
        clarification_prompt = f"""
        The following command is ambiguous. Generate 2-3 clarifying questions
        that would help understand what the user wants:

        Command: "{command}"

        Return as a JSON list of questions.
        """

        questions = self.llm_client.generate(clarification_prompt)
        questions_list = json.loads(questions)

        # Ask user for clarification
        for question in questions_list:
            response = self.ask_user_for_clarification(question)
            if response:
                # Regenerate plan with clarification
                clarified_command = f"{command} - {question} - {response}"
                return self.plan_from_command(clarified_command)

        # If no clarification received, use default interpretation
        return self.plan_from_command(command)

    def ask_user_for_clarification(self, question: str) -> Optional[str]:
        """Ask user for clarification (implementation depends on interaction method)"""
        # This would typically involve speech output or GUI
        print(f"Robot: {question}")
        # In practice, this would wait for user response
        return None  # Simplified for example

    def execute_with_monitoring(self, action_sequence: List[RobotAction]) -> bool:
        """Execute action sequence with monitoring and error recovery"""
        for i, action in enumerate(action_sequence):
            print(f"Executing action {i+1}/{len(action_sequence)}: {action.description}")

            # Execute action
            success = self.execute_action(action)

            if not success:
                # Handle failure
                recovery_plan = self.generate_recovery_plan(action, action_sequence[i+1:])
                if recovery_plan:
                    success = self.execute_with_monitoring(recovery_plan)
                    if success:
                        continue

            if not success:
                print(f"Action failed and no recovery possible: {action.description}")
                return False

        return True

    def generate_recovery_plan(self, failed_action: RobotAction, remaining_actions: List[RobotAction]) -> List[RobotAction]:
        """Generate recovery plan when action fails"""
        recovery_prompt = f"""
        The following action failed:
        Action: {failed_action.action_type}
        Parameters: {json.dumps(failed_action.parameters)}
        Description: {failed_action.description}

        Remaining actions: {json.dumps([a.description for a in remaining_actions])}

        Generate a recovery plan that addresses the failure and continues toward the goal.
        Return as a list of JSON objects with "action_type", "parameters", and "description".
        """

        try:
            recovery_response = self.llm_client.generate(recovery_prompt)
            recovery_actions = json.loads(recovery_response)

            # Convert to RobotAction objects
            robot_actions = []
            for action_dict in recovery_actions:
                robot_action = RobotAction(
                    action_type=action_dict["action_type"],
                    parameters=action_dict["parameters"],
                    description=action_dict["description"]
                )
                robot_actions.append(robot_action)

            return robot_actions
        except:
            return []  # No recovery plan available
```

Cognitive planning with LLMs bridges the gap between natural language commands and executable robot actions, enabling more intuitive and flexible human-robot interaction in complex tasks.