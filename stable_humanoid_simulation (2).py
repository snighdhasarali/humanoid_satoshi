import pybullet as p
import pybullet_data
import numpy as np
import time
import math

class StableHumanoidSimulation:
    def __init__(self):
        # Initialize PyBullet
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setTimeStep(1.0/240.0)  # Fixed timestep
        
        # Load plane and humanoid
        self.planeId = p.loadURDF("plane.urdf")
        
        # Start the humanoid in a better position
        startPos = [0, 0, 1.0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.humanoidId = p.loadURDF("humanoid/humanoid.urdf", 
                                    startPos, 
                                    startOrientation,
                                    useFixedBase=False)  # Allow it to fall but we'll control it
        
        # Enable joint torque sensors for better control
        for i in range(p.getNumJoints(self.humanoidId)):
            p.enableJointForceTorqueSensor(self.humanoidId, i, enableSensor=1)
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.humanoidId)
        self.joint_indices = list(range(self.num_joints))
        
        # Initialize control parameters with better values
        self.setup_control_parameters()
        
        # Disable default motors to use our own control
        self.disable_default_motors()
        
        print(f"Humanoid loaded with {self.num_joints} joints")
        
    def disable_default_motors(self):
        """Disable default joint motors to use our own control"""
        for i in range(self.num_joints):
            p.setJointMotorControl2(
                bodyUniqueId=self.humanoidId,
                jointIndex=i,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=0,
                force=0
            )
    
    def setup_control_parameters(self):
        # Joint information
        self.joint_names = []
        self.joint_limits = {}
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.humanoidId, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_names.append(joint_name)
            
            # Store joint limits
            self.joint_limits[i] = {
                'lower': joint_info[8],
                'upper': joint_info[9]
            }
        
        # Better control parameters for stability
        self.kp = 100.0  # Higher proportional gain for stability
        self.kd = 10.0   # Higher derivative gain for damping
        
        # Standing pose (neutral position)
        self.standing_pose = self.get_standing_pose()
        
    def get_standing_pose(self):
        """Define a stable standing pose"""
        standing_pose = np.zeros(self.num_joints)
        
        # Set specific joint angles for stable standing
        for i, name in enumerate(self.joint_names):
            if "hip" in name.lower() and "right" in name.lower():
                standing_pose[i] = 0.1  # Slight bend
            elif "hip" in name.lower() and "left" in name.lower():
                standing_pose[i] = -0.1  # Slight bend
            elif "knee" in name.lower():
                standing_pose[i] = -0.2  # Bent knees for stability
            elif "ankle" in name.lower():
                standing_pose[i] = 0.1   # Ankle adjustment
            elif "shoulder" in name.lower():
                standing_pose[i] = 0.0   # Neutral shoulders
            elif "elbow" in name.lower():
                standing_pose[i] = 0.0   # Straight elbows
        
        return standing_pose
    
    def get_joint_states(self):
        """Get current joint positions and velocities"""
        joint_states = p.getJointStates(self.humanoidId, self.joint_indices)
        positions = [state[0] for state in joint_states]
        velocities = [state[1] for state in joint_states]
        return np.array(positions), np.array(velocities)
    
    def get_base_position(self):
        """Get the base position and orientation"""
        base_state = p.getBasePositionAndOrientation(self.humanoidId)
        return base_state[0], base_state[1]
    
    def balance_control(self):
        """Simple balance control to keep the humanoid upright"""
        base_pos, base_orn = self.get_base_position()
        euler_angles = p.getEulerFromQuaternion(base_orn)
        
        # Get current joint states
        current_pos, current_vel = self.get_joint_states()
        
        # Target is standing pose with balance adjustments
        target_positions = self.standing_pose.copy()
        
        # Adjust based on tilt (simple balance)
        roll, pitch, yaw = euler_angles
        
        # Balance adjustment - counteract tilting
        balance_adjustment = np.zeros(self.num_joints)
        
        for i, name in enumerate(self.joint_names):
            if "ankle" in name.lower() and "right" in name.lower():
                balance_adjustment[i] = -pitch * 0.5 - roll * 0.3
            elif "ankle" in name.lower() and "left" in name.lower():
                balance_adjustment[i] = -pitch * 0.5 + roll * 0.3
            elif "hip" in name.lower() and "right" in name.lower():
                balance_adjustment[i] = pitch * 0.3
            elif "hip" in name.lower() and "left" in name.lower():
                balance_adjustment[i] = pitch * 0.3
        
        target_positions += balance_adjustment
        
        return target_positions
    
    def impedance_control(self, target_positions, target_velocities=None):
        """
        Stable Impedance Control with torque limits
        """
        if target_velocities is None:
            target_velocities = np.zeros(len(target_positions))
            
        current_pos, current_vel = self.get_joint_states()
        
        # Calculate impedance control forces
        position_error = target_positions - current_pos
        velocity_error = target_velocities - current_vel
        
        # Impedance control law
        forces = self.kp * position_error + self.kd * velocity_error
        
        # Apply torque limits for stability
        max_torque = 50.0
        forces = np.clip(forces, -max_torque, max_torque)
        
        # Apply forces
        for i, force in enumerate(forces):
            p.setJointMotorControl2(
                bodyUniqueId=self.humanoidId,
                jointIndex=i,
                controlMode=p.TORQUE_CONTROL,
                force=force
            )
    
    def admittance_control(self, external_force, dt=1.0/240.0):
        """
        Admittance Control: position response to external forces
        """
        M = 2.0  # Virtual mass
        D = 5.0  # Virtual damping  
        K = 50.0 # Virtual stiffness
        
        current_pos, current_vel = self.get_joint_states()
        
        # Admittance control law
        acceleration = (external_force - D * current_vel - K * current_pos) / M
        
        # Integration to get velocity and position
        target_velocity = current_vel + acceleration * dt
        target_position = current_pos + target_velocity * dt
        
        return target_position, target_velocity
    
    def simple_inverse_kinematics(self, target_position, link_name="right_wrist"):
        """
        Simple Inverse Kinematics for end effector positioning
        """
        # Find link index
        link_index = -1
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.humanoidId, i)
            if joint_info[12].decode('utf-8') == link_name:
                link_index = i
                break
        
        if link_index == -1:
            print(f"Link {link_name} not found, using current positions")
            current_pos, _ = self.get_joint_states()
            return current_pos
        
        try:
            # Use current positions as initial guess
            current_pos, _ = self.get_joint_states()
            
            # Calculate IK
            joint_poses = p.calculateInverseKinematics(
                self.humanoidId,
                link_index,
                targetPosition=target_position,
                targetOrientation=[0, 0, 0, 1],
                lowerLimits=[self.joint_limits[i]['lower'] for i in range(self.num_joints)],
                upperLimits=[self.joint_limits[i]['upper'] for i in range(self.num_joints)],
                jointRanges=[self.joint_limits[i]['upper'] - self.joint_limits[i]['lower'] for i in range(self.num_joints)],
                restPoses=current_pos.tolist(),
                maxNumIterations=50
            )
            
            return np.array(joint_poses[:self.num_joints])
            
        except Exception as e:
            print(f"IK failed: {e}, using current positions")
            current_pos, _ = self.get_joint_states()
            return current_pos
    
    def hjb_value_iteration(self, state_grid, goal_state, max_iterations=50):
        """
        Simplified HJB Value Iteration for demonstration
        """
        V = np.zeros(len(state_grid))
        
        for iteration in range(max_iterations):
            V_new = np.copy(V)
            
            for i in range(len(state_grid)):
                state_cost = abs(state_grid[i] - goal_state[0])
                
                # Consider possible next states
                possible_costs = [state_cost + V[i]]  # Stay
                
                if i > 0:
                    possible_costs.append(state_cost + V[i-1])  # Move left
                if i < len(state_grid) - 1:
                    possible_costs.append(state_cost + V[i+1])  # Move right
                
                V_new[i] = min(possible_costs)
            
            # Check for convergence
            if np.max(np.abs(V_new - V)) < 0.001:
                break
                
            V = V_new
        
        print(f"HJB converged in {iteration+1} iterations")
        return V
    
    def run_stable_simulation(self, duration=10):
        """Main stable simulation loop"""
        print("Starting stable simulation...")
        
        time_step = 0
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Balance control to maintain upright position
            balanced_target = self.balance_control()
            
            # Apply impedance control with balanced targets
            self.impedance_control(balanced_target)
            
            # Occasionally apply small external forces to demonstrate admittance
            if time_step % 200 == 0:
                external_force = np.random.normal(0, 2.0, self.num_joints)
                admittance_target, admittance_velocity = self.admittance_control(external_force)
                self.impedance_control(admittance_target, admittance_velocity)
            
            # Step simulation
            p.stepSimulation()
            time.sleep(1.0/240.0)
            time_step += 1
    
    def demo_arm_movement(self):
        """Demonstrate arm movement using IK"""
        print("Demonstrating arm movement with IK...")
        
        # Get current right wrist position
        right_wrist_state = p.getLinkState(self.humanoidId, 5)  # right_wrist
        current_pos = right_wrist_state[0]
        
        # Create a simple circular trajectory for the right arm
        for angle in range(0, 360, 10):
            theta = math.radians(angle)
            target_x = current_pos[0] + 0.2 * math.cos(theta)
            target_y = current_pos[1] + 0.2 * math.sin(theta) 
            target_z = current_pos[2] + 0.1
            
            target_position = [target_x, target_y, target_z]
            
            # Calculate IK for right wrist
            joint_angles = self.simple_inverse_kinematics(target_position, "right_wrist")
            
            # Blend with balanced pose for stability
            balanced_target = self.balance_control()
            blended_target = 0.7 * balanced_target + 0.3 * joint_angles
            
            # Apply control
            self.impedance_control(blended_target)
            
            # Step simulation
            for _ in range(5):  # Multiple steps per target
                p.stepSimulation()
                time.sleep(1.0/240.0)
    
    def demo_control_strategies(self):
        """Demonstrate different control strategies"""
        print("\n=== DEMO: Control Strategies ===")
        
        # 1. Basic standing with balance
        print("1. Standing with balance control...")
        self.run_stable_simulation(duration=3)
        
        # 2. Arm movement with IK
        print("2. Arm movement with Inverse Kinematics...")
        self.demo_arm_movement()
        
        # 3. HJB demonstration
        print("3. HJB Value Iteration...")
        state_grid = np.linspace(-2, 2, 20)
        goal_state = np.array([0.5])
        value_function = self.hjb_value_iteration(state_grid, goal_state)
        print(f"HJB Value function range: {np.min(value_function):.3f} to {np.max(value_function):.3f}")
        
        # 4. Combined demonstration
        print("4. Combined control demonstration...")
        self.run_stable_simulation(duration=3)
    
    def close(self):
        """Clean shutdown"""
        try:
            p.disconnect()
            print("Simulation closed successfully")
        except:
            print("Disconnect completed")

def main():
    # Create simulation
    sim = StableHumanoidSimulation()
    
    try:
        # Let the humanoid stabilize initially
        print("Initial stabilization...")
        for _ in range(100):
            balanced_target = sim.balance_control()
            sim.impedance_control(balanced_target)
            p.stepSimulation()
            time.sleep(1.0/240.0)
        
        # Run demonstrations
        sim.demo_control_strategies()
        
        print("\nSimulation completed successfully!")
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"Simulation error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        sim.close()

if __name__ == "__main__":
    main()
