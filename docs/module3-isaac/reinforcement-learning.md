# Reinforcement Learning in Robotics with Isaac Sim

## Introduction to RL for Robotics

Reinforcement Learning (RL) has emerged as a powerful paradigm for training intelligent robotic behaviors, particularly for complex tasks that are difficult to program explicitly. Isaac Sim provides an ideal environment for developing and testing RL algorithms for humanoid robots, offering realistic physics simulation, photorealistic rendering, and seamless integration with popular RL frameworks.

## RL Fundamentals for Robotics

### Markov Decision Processes in Robotics

Robotic tasks can be formulated as Markov Decision Processes (MDPs):

- **State Space (S)**: Robot's sensor readings, joint positions, velocities, and environmental observations
- **Action Space (A)**: Joint commands, Cartesian velocities, or high-level behaviors
- **Reward Function (R)**: Scalar feedback based on task success, energy efficiency, or safety
- **Transition Dynamics (P)**: Physics simulation governing state transitions
- **Discount Factor (γ)**: Trade-off between immediate and future rewards

### RL Problem Formulation

```python
import torch
import numpy as np
from omni.isaac.gym import IsaacEnv
from omni.isaac.core.articulations import ArticulationView

class RobotRLTask(IsaacEnv):
    def __init__(self, cfg):
        super().__init__(cfg)

        # Define action and observation spaces
        self.action_space = self.define_action_space()
        self.observation_space = self.define_observation_space()

        # Initialize robot and environment
        self.robot = ArticulationView(
            prim_paths_expr="/World/Robot/.*",
            name="robot_view"
        )

        self.task_params = cfg["task_params"]

    def define_action_space(self):
        # Define continuous action space for humanoid robot
        action_dim = 12  # Number of joints to control
        action_low = np.ones(action_dim) * -1.0
        action_high = np.ones(action_dim) * 1.0
        return gym.spaces.Box(action_low, action_high)

    def define_observation_space(self):
        # Define observation space including robot state and task info
        obs_dim = 48  # Combined state dimensions
        obs_low = np.ones(obs_dim) * -np.inf
        obs_high = np.ones(obs_dim) * np.inf
        return gym.spaces.Box(obs_low, obs_high)

    def get_observations(self):
        # Get current robot state and task-relevant information
        robot_pos, robot_orn = self.robot.get_world_poses()
        robot_vel = self.robot.get_velocities()
        target_pos = self.get_target_position()

        # Combine into observation vector
        obs = np.concatenate([
            robot_pos,
            robot_orn,
            robot_vel,
            target_pos,
            self.get_task_specific_info()
        ])

        return torch.from_numpy(obs).float()

    def calculate_reward(self, actions):
        # Calculate reward based on task progress
        current_pos = self.robot.get_world_poses()[0]
        target_pos = self.get_target_position()

        # Distance-based reward
        distance_to_target = torch.norm(target_pos - current_pos)
        distance_reward = torch.exp(-distance_to_target)

        # Bonus for reaching target
        target_reached = distance_to_target < self.task_params["success_threshold"]
        success_bonus = 10.0 if target_reached else 0.0

        # Penalty for unsafe actions
        action_penalty = torch.mean(torch.square(actions))

        total_reward = distance_reward + success_bonus - 0.01 * action_penalty
        return total_reward
```

## Isaac Sim RL Integration

### Isaac Gym Environment

Isaac Sim provides a specialized environment for RL training:

```python
from omni.isaac.gym.vec_env import VecEnvGPU
from omni.isaac.core.utils.torch.maths import torch
import torch.nn as nn

class IsaacVecEnvGPU(VecEnvGPU):
    def __init__(self, task, sim_device, rl_device, graphics_device_id, headless):
        super().__init__(
            task=task,
            sim_device=sim_device,
            rl_device=rl_device,
            graphics_device_id=graphics_device_id,
            headless=headless
        )

    def reset(self):
        # Reset all environments in the vectorized environment
        super().reset()
        return self.get_observations()

    def step(self, actions):
        # Execute actions and return observations, rewards, etc.
        self.send_acctions(actions)
        self.render()
        obs = self.get_observations()
        rewards = self.get_rewards()
        dones = self.get_dones()
        info = self.get_extras()

        return obs, rewards, dones, info
```

### Physics Simulation for RL

Isaac Sim's PhysX engine provides realistic physics for RL training:

- **Accurate contact simulation**: Realistic friction, collision, and contact handling
- **Rigid body dynamics**: Proper mass, inertia, and force calculations
- **Multi-body systems**: Complex articulated robots with constraints
- **Soft body simulation**: Deformable objects and environments

## Deep RL Algorithms for Robotics

### Deep Deterministic Policy Gradient (DDPG)

DDPG is well-suited for continuous control tasks in robotics:

```python
import torch.nn as nn
import torch.optim as optim

class DDPGActor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(DDPGActor, self).__init__()

        self.l1 = nn.Linear(state_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, action_dim)

        self.max_action = max_action

    def forward(self, state):
        a = torch.relu(self.l1(state))
        a = torch.relu(self.l2(a))
        return self.max_action * torch.tanh(self.l3(a))

class DDPGCritic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(DDGCritic, self).__init__()

        self.l1 = nn.Linear(state_dim + action_dim, 256)
        self.l2 = nn.Linear(256, 256)
        self.l3 = nn.Linear(256, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], 1)
        q = torch.relu(self.l1(sa))
        q = torch.relu(self.l2(q))
        return self.l3(q)

class DDPGAgent:
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = DDPGActor(state_dim, action_dim, max_action).to(device)
        self.actor_target = DDPGActor(state_dim, action_dim, max_action).to(device)
        self.critic = DDPGCritic(state_dim, action_dim).to(device)
        self.critic_target = DDPGCritic(state_dim, action_dim).to(device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=1e-4)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=1e-3)
```

### Soft Actor-Critic (SAC)

SAC provides better sample efficiency and stability:

```python
class SACAgent:
    def __init__(self, state_dim, action_dim, max_action):
        self.actor = GaussianPolicy(state_dim, action_dim, max_action).to(device)
        self.critic_1 = QNetwork(state_dim, action_dim).to(device)
        self.critic_2 = QNetwork(state_dim, action_dim).to(device)

        self.target_entropy = -torch.prod(torch.Tensor(action_dim)).item()
        self.log_alpha = torch.zeros(1, requires_grad=True, device=device)
        self.alpha_optimizer = optim.Adam([self.log_alpha], lr=1e-4)

    def select_action(self, state, evaluate=False):
        state = torch.FloatTensor(state).to(device).unsqueeze(0)
        if evaluate is False:
            action, _, _ = self.actor.sample(state)
        else:
            _, _, action = self.actor.sample(state)
        return action.cpu().data.numpy().flatten()
```

### Proximal Policy Optimization (PPO)

PPO is particularly stable for humanoid locomotion:

```python
class PPOAgent:
    def __init__(self, state_dim, action_dim):
        self.actor = Actor(state_dim, action_dim).to(device)
        self.critic = Critic(state_dim).to(device)

        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=3e-4)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=3e-4)

        self.ppo_epochs = 10
        self.clip_epsilon = 0.2

    def update(self, states, actions, log_probs, returns, advantages):
        for _ in range(self.ppo_epochs):
            # Get current policy probabilities
            dist = self.actor(states)
            current_log_probs = dist.log_prob(actions)

            # Calculate ratio
            ratio = torch.exp(current_log_probs - log_probs)

            # Calculate surrogate objectives
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.clip_epsilon, 1 + self.clip_epsilon) * advantages

            # Actor loss
            actor_loss = -torch.min(surr1, surr2).mean()

            # Critic loss
            current_values = self.critic(states)
            critic_loss = F.mse_loss(current_values, returns)

            # Update networks
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            self.critic_optimizer.zero_grad()
            critic_loss.backward()
            self.critic_optimizer.step()
```

## Isaac Sim RL Examples

### Humanoid Locomotion

Training bipedal locomotion with RL:

```python
class HumanoidLocomotionTask(RobotRLTask):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.target_velocity = cfg["target_velocity"]
        self.max_episode_length = cfg["max_episode_length"]

    def get_observations(self):
        # Get humanoid-specific observations
        base_pos, base_orn = self.robot.get_world_poses()
        base_vel = self.robot.get_velocities()

        # Extract humanoid-specific features
        torso_pos = base_pos
        torso_orn = base_orn
        linear_vel = base_vel[:, :3]
        angular_vel = base_vel[:, 3:6]

        # Joint positions and velocities
        joint_pos = self.robot.get_joint_positions()
        joint_vel = self.robot.get_joint_velocities()

        # Combine all observations
        obs = torch.cat([
            torso_pos,
            torso_orn,
            linear_vel,
            angular_vel,
            joint_pos,
            joint_vel,
            self.target_velocity
        ], dim=-1)

        return obs

    def calculate_reward(self, actions):
        # Reward for forward locomotion
        base_pos, base_orn = self.robot.get_world_poses()
        base_vel = self.robot.get_velocities()

        # Forward velocity reward
        forward_vel = base_vel[:, 0]  # x-axis velocity
        forward_reward = torch.clamp(forward_vel, 0, None)

        # Standing upright reward
        target_orn = torch.tensor([0, 0, 0, 1]).to(self.device)  # upright quaternion
        orientation_reward = 1.0 - torch.abs(torch.sum(base_orn * target_orn, dim=-1))

        # Action smoothness penalty
        action_penalty = torch.mean(torch.square(actions))

        # Joint position limits penalty
        joint_pos = self.robot.get_joint_positions()
        joint_limit_penalty = torch.mean(torch.clamp(torch.abs(joint_pos) - 2.0, min=0.0))

        total_reward = (
            1.0 * forward_reward +
            0.5 * orientation_reward -
            0.01 * action_penalty -
            0.1 * joint_limit_penalty
        )

        return total_reward
```

### Manipulation Tasks

Training manipulation skills with RL:

```python
class ManipulationTask(RobotRLTask):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.object_pos = None
        self.target_pos = None

    def reset(self):
        # Reset object and target positions
        self.object_pos = self.generate_random_object_pos()
        self.target_pos = self.generate_random_target_pos()

        # Place object and target in environment
        self.place_object(self.object_pos)
        self.place_target(self.target_pos)

        return self.get_observations()

    def get_observations(self):
        # Get manipulation-specific observations
        ee_pos, ee_orn = self.robot.get_end_effector_poses()
        object_pos = self.get_object_position()
        target_pos = self.target_pos

        # Joint states
        joint_pos = self.robot.get_joint_positions()
        joint_vel = self.robot.get_joint_velocities()

        obs = torch.cat([
            ee_pos,           # End effector position
            ee_orn,           # End effector orientation
            object_pos,       # Object position
            target_pos,       # Target position
            joint_pos,        # Joint positions
            joint_vel,        # Joint velocities
            ee_pos - object_pos,  # Relative position to object
            object_pos - target_pos  # Relative position to target
        ], dim=-1)

        return obs

    def calculate_reward(self, actions):
        ee_pos = self.robot.get_end_effector_positions()
        object_pos = self.get_object_position()
        target_pos = self.target_pos

        # Grasping reward
        grasp_distance = torch.norm(ee_pos - object_pos, dim=-1)
        grasp_reward = torch.exp(-grasp_distance)

        # Object-to-target reward
        object_to_target = torch.norm(object_pos - target_pos, dim=-1)
        placement_reward = torch.exp(-object_to_target)

        # Bonus for successful placement
        success = object_to_target < 0.1  # 10cm threshold
        success_bonus = 10.0 * success.float()

        total_reward = grasp_reward + placement_reward + success_bonus
        return total_reward
```

## Sample Efficiency and Transfer Learning

### Domain Randomization

Domain randomization improves sim-to-real transfer:

```python
class DomainRandomization:
    def __init__(self, env):
        self.env = env
        self.param_ranges = self.define_param_ranges()

    def define_param_ranges(self):
        return {
            'robot_mass': [0.8, 1.2],  # 80% to 120% of nominal mass
            'friction': [0.5, 1.5],    # Friction coefficient range
            'restitution': [0.0, 0.2], # Bounciness range
            'actuator_strength': [0.8, 1.2], # Actuator force range
            'sensor_noise': [0.0, 0.05] # Sensor noise range
        }

    def randomize_environment(self):
        # Randomize physics parameters
        for param, (min_val, max_val) in self.param_ranges.items():
            if param == 'robot_mass':
                self.set_robot_mass(
                    self.nominal_mass * np.random.uniform(min_val, max_val)
                )
            elif param == 'friction':
                self.set_friction_coefficient(
                    np.random.uniform(min_val, max_val)
                )
            # ... continue for other parameters

    def train_with_randomization(self, agent, num_episodes):
        for episode in range(num_episodes):
            # Randomize environment at the start of each episode
            if episode % 10 == 0:  # Randomize every 10 episodes
                self.randomize_environment()

            # Train agent in randomized environment
            self.train_agent(agent)
```

### Curriculum Learning

Gradually increase task difficulty:

```python
class CurriculumLearning:
    def __init__(self, tasks):
        self.tasks = tasks
        self.current_task = 0
        self.performance_threshold = 0.8  # 80% success rate

    def evaluate_performance(self, agent):
        # Evaluate agent on current task
        successes = 0
        total_episodes = 100

        for episode in range(total_episodes):
            episode_reward = self.run_episode(agent)
            if episode_reward >= self.tasks[self.current_task].success_threshold:
                successes += 1

        return successes / total_episodes

    def advance_curriculum(self, agent):
        current_performance = self.evaluate_performance(agent)

        if current_performance >= self.performance_threshold:
            if self.current_task < len(self.tasks) - 1:
                self.current_task += 1
                print(f"Advancing to task {self.current_task + 1}")

                # Modify task parameters for increased difficulty
                self.modify_task_difficulty()

        return self.current_task

    def modify_task_difficulty(self):
        # Example: Increase target velocity for locomotion task
        if self.current_task == 1:  # Walking
            self.tasks[self.current_task].target_velocity *= 1.2
        elif self.current_task == 2:  # Running
            self.tasks[self.current_task].target_velocity *= 1.1
```

## Isaac Sim RL Training Pipeline

### Training Configuration

```yaml
# rl_training_config.yaml
rl_games_config:
  params:
    seed: 42
    algo:
      name: a2c_continuous
    model:
      name: continuous_a2c_logstd
    network:
      name: actor_critic
      separate: False
      space:
        continuous:
          mu_activation: None
          sigma_activation: None
          mu_init:
            name: default
          sigma_init:
            name: const_initializer
            val: 0
          fixed_sigma: True
    train:
      learning_rate: 3e-4
      schedule: adaptive
      kl_threshold: 0.008
      resampling_freq: 4
      max_episodes: 100000
      save_best_after: 100
      save_frequency: 500
      grad_norm: 1.0
      entropy_coef: 0.0
      truncate_grads: True
      e_clip: 0.2
      horizon_length: 32
      minibatch_size: 64
      mini_epochs: 4
      critic_coef: 2
      clip_value: True
```

### Multi-Environment Training

```python
def train_rl_agent():
    # Create multiple environments for parallel training
    num_envs = 4096  # Large batch for efficient training
    envs_per_instance = 128

    # Initialize Isaac Sim environments
    envs = []
    for i in range(0, num_envs, envs_per_instance):
        env = HumanoidLocomotionTask(
            cfg={
                'num_envs': min(envs_per_instance, num_envs - i),
                'env_spacing': 2.0,
                'episode_length': 1000,
                'asset_root': '/path/to/assets',
                'asset_file': 'humanoid.urdf'
            }
        )
        envs.append(env)

    # Initialize RL agent
    agent = PPOAgent(
        state_dim=envs[0].observation_space.shape[0],
        action_dim=envs[0].action_space.shape[0]
    )

    # Training loop
    for episode in range(100000):
        # Collect experiences from all environments
        experiences = collect_experiences(envs, agent)

        # Update agent with collected experiences
        agent.update(experiences)

        # Log training metrics
        if episode % 100 == 0:
            log_metrics(episode, experiences, agent)
```

## Safety and Robustness

### Safe RL in Isaac Sim

```python
class SafeRLAgent:
    def __init__(self, state_dim, action_dim):
        self.nominal_agent = PPOAgent(state_dim, action_dim)
        self.safety_critic = SafetyCritic(state_dim).to(device)
        self.safety_threshold = 0.1  # Maximum risk threshold

    def safe_action_selection(self, state):
        # Get nominal action from policy
        nominal_action = self.nominal_agent.select_action(state)

        # Evaluate safety of action
        safety_score = self.safety_critic.evaluate(state, nominal_action)

        if safety_score > self.safety_threshold:
            # Use safe fallback action
            safe_action = self.get_safe_fallback_action(state)
            return safe_action
        else:
            return nominal_action

    def incorporate_safety_rewards(self, rewards, states, actions):
        # Add safety penalties to rewards
        safety_costs = self.safety_critic.get_safety_costs(states, actions)
        safe_rewards = rewards - 10.0 * torch.clamp(safety_costs - self.safety_threshold, min=0.0)
        return safe_rewards
```

### Robustness Testing

```python
class RobustnessEvaluator:
    def __init__(self, trained_agent):
        self.agent = trained_agent
        self.disturbance_types = [
            'external_force',
            'sensor_noise',
            'actuator_delay',
            'model_uncertainty'
        ]

    def test_robustness(self):
        robustness_results = {}

        for disturbance_type in self.disturbance_types:
            success_rate = self.apply_disturbance_and_test(
                disturbance_type=disturbance_type,
                magnitude_range=[0.1, 1.0]
            )
            robustness_results[disturbance_type] = success_rate

        return robustness_results

    def apply_disturbance_and_test(self, disturbance_type, magnitude_range):
        # Apply disturbance and measure success rate
        successes = 0
        total_tests = 100

        for test in range(total_tests):
            magnitude = np.random.uniform(*magnitude_range)
            success = self.test_single_disturbance(
                disturbance_type, magnitude
            )
            if success:
                successes += 1

        return successes / total_tests
```

## Advanced RL Techniques

### Meta-Learning for Robotics

```python
class MetaRLAgent:
    def __init__(self, state_dim, action_dim):
        self.meta_learner = MAML(state_dim, action_dim)
        self.fast_adaptation_steps = 5

    def adapt_to_new_task(self, new_task_data):
        # Adapt quickly to new task using meta-learning
        adapted_agent = self.meta_learner.adapt(
            task_data=new_task_data,
            num_steps=self.fast_adaptation_steps
        )
        return adapted_agent

    def learn_to_adapt(self, tasks):
        # Meta-training phase: learn to adapt quickly
        for meta_batch in tasks:
            for task in meta_batch:
                # Sample trajectories
                trajectories = self.sample_trajectories(task)

                # Adapt to task
                adapted_params = self.meta_learner.adapt(
                    trajectories, task
                )

                # Evaluate on test set
                test_reward = self.evaluate_on_task(
                    adapted_params, task.test_set
                )

                # Update meta-learner
                self.meta_learner.update(test_reward)
```

### Multi-Task RL

```python
class MultiTaskRLAgent:
    def __init__(self, tasks, state_dim, action_dim):
        self.tasks = tasks
        self.shared_encoder = SharedEncoder(state_dim)
        self.task_specific_heads = nn.ModuleList([
            TaskHead(action_dim) for _ in tasks
        ])
        self.task_classifier = TaskClassifier(len(tasks))

    def forward(self, state, task_id=None):
        # Encode shared features
        shared_features = self.shared_encoder(state)

        if task_id is not None:
            # Use specific task head
            action = self.task_specific_heads[task_id](shared_features)
        else:
            # Infer task and use appropriate head
            task_probs = self.task_classifier(shared_features)
            task_id = torch.argmax(task_probs)
            action = self.task_specific_heads[task_id](shared_features)

        return action
```

Reinforcement learning with Isaac Sim provides a powerful framework for training complex humanoid robot behaviors, combining realistic physics simulation with efficient learning algorithms to develop robust and adaptive robotic systems.