"""
Minimal Gym-like Environment Wrapper for SAPIEN
================================================
This script demonstrates how to create a reusable environment wrapper around SAPIEN,
similar to OpenAI Gym/Gymnasium interfaces. Useful for reinforcement learning tasks.

Key Concepts:
- Environment initialization and cleanup
- Step function with action, observation, reward
- Reset functionality
- Observation and action space definitions
"""

import sapien
import numpy as np
from pathlib import Path


class MinimalRobotEnv:
    """
    A minimal gym-like environment wrapper for SAPIEN robot control.
    
    Observation: Joint positions and velocities (2 * n_joints,)
    Action: Target joint positions (n_joints,)
    Reward: Negative distance to goal position
    """
    
    def __init__(self, render_mode=None):
        """
        Initialize the environment.
        
        Args:
            render_mode: None for headless, 'human' for interactive viewer
        """
        self.render_mode = render_mode
        
        # Create SAPIEN engine and scene
        self.engine = sapien.Engine()
        self.scene = self.engine.create_scene()
        self.scene.set_timestep(1 / 240.0)
        
        # Add lighting
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, -1, -1], [0.5, 0.5, 0.5])
        
        # Add ground
        self.scene.add_ground(altitude=0)
        
        # Load robot
        self._load_robot()
        
        # Initialize viewer if needed
        self.viewer = None
        if self.render_mode == 'human':
            self.viewer = sapien.utils.Viewer()
            self.viewer.set_scene(self.scene)
            self.viewer.set_camera_pose(
                sapien.Pose([2, 0, 1.5], [0.9239, 0, 0, -0.3827])
            )
        
        # Define observation and action spaces
        self.active_joints = [j for j in self.robot.get_joints() if j.dof > 0]
        self.n_joints = len(self.active_joints)
        
        # Observation: [positions, velocities]
        self.observation_dim = self.n_joints * 2
        
        # Action: target joint positions
        self.action_dim = self.n_joints
        
        # Goal configuration (for reward computation)
        self.goal_qpos = np.array([0, 0.5, 0, -1.5, 0, 2.0, 0.785, 0.04, 0.04])[:self.n_joints]
        
        # Episode tracking
        self.current_step = 0
        self.max_steps = 500
        
    def _load_robot(self):
        """Load the Panda robot into the scene."""
        loader = self.scene.create_urdf_loader()
        loader.fix_root_link = True
        
        robot_path = Path(__file__).parent.parent.parent / "robot_descriptions/Panda/panda.urdf"
        self.robot = loader.load(str(robot_path))
        self.robot.set_pose(sapien.Pose([0, 0, 0]))
        
        # Configure joint drives for position control
        for joint in self.robot.get_joints():
            if joint.dof > 0:
                joint.set_drive_properties(stiffness=1000, damping=200)
    
    def reset(self, seed=None):
        """
        Reset the environment to initial state.
        
        Args:
            seed: Random seed for reproducibility
            
        Returns:
            observation: Initial observation
            info: Additional information dict
        """
        if seed is not None:
            np.random.seed(seed)
        
        # Reset to initial configuration with small random noise
        init_qpos = np.array([0, 0, 0, -1.0, 0, 1.57, 0.785, 0.04, 0.04])[:self.n_joints]
        init_qpos += np.random.uniform(-0.1, 0.1, self.n_joints)
        
        # Set joint positions and velocities
        for i, joint in enumerate(self.active_joints):
            joint.set_drive_target(init_qpos[i])
        self.robot.set_qpos(init_qpos)
        self.robot.set_qvel(np.zeros(self.n_joints))
        
        # Step physics to stabilize
        for _ in range(10):
            self.scene.step()
        
        self.current_step = 0
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def step(self, action):
        """
        Execute one environment step.
        
        Args:
            action: Target joint positions (n_joints,)
            
        Returns:
            observation: Current state observation
            reward: Scalar reward value
            terminated: Whether episode is done (goal reached)
            truncated: Whether episode hit time limit
            info: Additional information dict
        """
        # Clip action to reasonable joint limits
        action = np.clip(action, -2.8, 2.8)
        
        # Apply action (set joint targets)
        for i, joint in enumerate(self.active_joints):
            joint.set_drive_target(action[i])
        
        # Step physics simulation
        self.scene.step()
        
        # Update viewer
        if self.viewer is not None:
            self.viewer.render()
        
        # Get new observation
        observation = self._get_observation()
        
        # Compute reward (negative distance to goal)
        qpos = self.robot.get_qpos()
        distance_to_goal = np.linalg.norm(qpos - self.goal_qpos)
        reward = -distance_to_goal
        
        # Check termination conditions
        terminated = distance_to_goal < 0.05  # Goal reached
        self.current_step += 1
        truncated = self.current_step >= self.max_steps  # Time limit
        
        info = self._get_info()
        info['distance_to_goal'] = distance_to_goal
        
        return observation, reward, terminated, truncated, info
    
    def _get_observation(self):
        """Get current observation (joint positions and velocities)."""
        qpos = self.robot.get_qpos()
        qvel = self.robot.get_qvel()
        observation = np.concatenate([qpos, qvel])
        return observation
    
    def _get_info(self):
        """Get additional information about current state."""
        return {
            'step': self.current_step,
            'qpos': self.robot.get_qpos().copy(),
            'qvel': self.robot.get_qvel().copy(),
        }
    
    def close(self):
        """Clean up resources."""
        if self.viewer is not None:
            self.viewer.close()
        self.scene = None
        self.engine = None


def demo_random_policy():
    """Demonstrate the environment with a random policy."""
    print("=" * 60)
    print("Demo 1: Random Policy")
    print("=" * 60)
    print("Running episodes with random actions...")
    
    env = MinimalRobotEnv(render_mode='human')
    
    n_episodes = 3
    for episode in range(n_episodes):
        obs, info = env.reset(seed=42 + episode)
        print(f"\nEpisode {episode + 1}")
        print(f"  Initial observation shape: {obs.shape}")
        print(f"  Initial qpos: {info['qpos'][:3]}... (showing first 3)")
        
        total_reward = 0
        for step in range(100):
            # Random action
            action = np.random.uniform(-0.5, 0.5, env.action_dim)
            
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            
            if step % 20 == 0:
                print(f"  Step {step}: reward={reward:.3f}, distance={info['distance_to_goal']:.3f}")
            
            if terminated or truncated:
                break
        
        print(f"  Episode finished after {info['step']} steps")
        print(f"  Total reward: {total_reward:.2f}")
        print(f"  Final distance to goal: {info['distance_to_goal']:.3f}")
    
    env.close()
    print("\n✓ Random policy demo completed")


def demo_simple_controller():
    """Demonstrate the environment with a simple P-controller."""
    print("\n" + "=" * 60)
    print("Demo 2: Simple P-Controller")
    print("=" * 60)
    print("Using proportional controller to reach goal...")
    
    env = MinimalRobotEnv(render_mode='human')
    
    obs, info = env.reset(seed=123)
    print(f"Initial distance to goal: {np.linalg.norm(info['qpos'] - env.goal_qpos):.3f}")
    
    # Simple proportional controller
    Kp = 0.1
    
    for step in range(300):
        current_qpos = info['qpos']
        
        # Compute action: move towards goal
        action = current_qpos + Kp * (env.goal_qpos - current_qpos)
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        if step % 30 == 0:
            print(f"Step {step}: distance={info['distance_to_goal']:.4f}, reward={reward:.3f}")
        
        if terminated:
            print(f"\n✓ Goal reached at step {step}!")
            print(f"  Final distance: {info['distance_to_goal']:.4f}")
            break
        
        if truncated:
            print(f"\nTime limit reached. Final distance: {info['distance_to_goal']:.4f}")
            break
    
    env.close()
    print("✓ P-controller demo completed")


def demo_observation_action_spaces():
    """Demonstrate observation and action space information."""
    print("\n" + "=" * 60)
    print("Demo 3: Observation and Action Spaces")
    print("=" * 60)
    
    env = MinimalRobotEnv(render_mode=None)  # Headless
    
    print(f"Action dimension: {env.action_dim}")
    print(f"Observation dimension: {env.observation_dim}")
    print(f"Number of controllable joints: {env.n_joints}")
    print(f"Max episode steps: {env.max_steps}")
    print(f"\nGoal configuration: {env.goal_qpos}")
    
    obs, info = env.reset()
    print(f"\nObservation breakdown:")
    print(f"  Joint positions (first half): {obs[:env.n_joints]}")
    print(f"  Joint velocities (second half): {obs[env.n_joints:]}")
    
    # Test a few steps
    print(f"\nTesting environment dynamics...")
    for i in range(5):
        action = np.zeros(env.action_dim)  # Hold position
        obs, reward, terminated, truncated, info = env.step(action)
    
    print(f"After 5 steps with zero action:")
    print(f"  Observation: {obs[:3]}... (showing first 3 values)")
    print(f"  Reward: {reward:.3f}")
    
    env.close()
    print("\n✓ Space information demo completed")


def main():
    """Run all demonstrations."""
    print("\n" + "=" * 60)
    print("SAPIEN Minimal Environment Wrapper Demo")
    print("=" * 60)
    print("\nThis demonstrates a gym-like interface for SAPIEN:")
    print("  - reset(): Initialize episode")
    print("  - step(action): Execute action and get observation, reward")
    print("  - Observation: Joint positions and velocities")
    print("  - Action: Target joint positions")
    print("  - Reward: Negative distance to goal configuration")
    
    # Run demonstrations
    demo_observation_action_spaces()
    demo_random_policy()
    demo_simple_controller()
    
    print("\n" + "=" * 60)
    print("All demos completed successfully!")
    print("=" * 60)
    print("\nKey Takeaways:")
    print("  ✓ Environment wrapper encapsulates SAPIEN setup")
    print("  ✓ Standard reset()/step() interface for RL")
    print("  ✓ Observations, rewards, and termination conditions")
    print("  ✓ Can be extended for more complex tasks")


if __name__ == "__main__":
    main()
