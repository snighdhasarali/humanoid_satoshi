import pybullet as p
import pybullet_data
import numpy as np
import time
import math

class RobustHumanoid:
    def __init__(self):
        # Initialize PyBullet with minimal settings
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.8)
        p.setPhysicsEngineParameter(fixedTimeStep=1.0/240.0)
        
        # Load simple environment
        self.planeId = p.loadURDF("plane.urdf")
        
        # Load humanoid with fixed base initially for stability
        startPos = [0, 0, 1.0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.humanoidId = p.loadURDF("humanoid/humanoid.urdf", 
                                    startPos, 
                                    startOrientation,
                                    useFixedBase=True)  # Fixed base for stability
        
        # Get joint information
        self.num_joints = p.getNumJoints(self.humanoidId)
        self.joint_indices = list(range(self.num_joints))
        
        # Setup control parameters
        self.setup_control()
        
        print(f"Humanoid loaded with {self.num_joints} joints")
        
    def setup_control(self):
        self.joint_names = []
        self.joint_limits = {}
        
        for i in range(self.num_joints):
            joint_info = p.getJointInfo(self.humanoidId, i)
            joint_name = joint_info[1].decode('utf-8')
            self.joint_names.append(joint_name)
            self.joint_limits[i] = {
                'lower': joint_info[8],
                'upper': joint_info[9]
            }
        
        # Conservative control gains
        self.kp = 50.0
        self.kd = 5.0
        
        # Define a stable pose
        self.stable_pose = self.get_stable_pose()
        
    def get_stable_pose(self):
        """Define a stable standing pose"""
        pose = np.zeros(self.num_joints)
        
        # Simple stable pose - mostly straight with slight knee bend
        for i, name in enumerate(self.joint_names):
            if "knee" in name.lower():
                pose[i] = -0.3  # Bent knees
            elif "ankle" in name.lower():
                pose[i] = 0.1   # Ankle adjustment
            elif "hip" in name.lower() and "right" in name.lower():
                pose[i] = 0.05
            elif "hip" in name.lower() and "left" in name.lower():
                pose[i] = -0.05
                
        return pose
    
    def get_joint_states(self):
        """Safely get joint states"""
        try:
            joint_states = p.getJointStates(self.humanoidId, self.joint_indices)
            positions = [state[0] for state in joint_states]
            velocities = [state[1] for state in joint_states]
            return np.array(positions), np.array(velocities)
        except:
            return np.zeros(self.num_joints), np.zeros(self.num_joints)
    
    def simple_impedance_control(self, target_positions):
        """Simple impedance control without complex math"""
        current_pos, current_vel = self.get_joint_states()
        
        # Simple PD control
        for i in range(self.num_joints):
            error = target_positions[i] - current_pos[i]
            force = self.kp * error - self.kd * current_vel[i]
            
            # Limit force
            force = max(min(force, 30.0), -30.0)
            
            p.setJointMotorControl2(
                bodyUniqueId=self.humanoidId,
                jointIndex=i,
                controlMode=p.TORQUE_CONTROL,
                force=force
            )
    
    def demo_impedance_control(self, duration=3):
        """Demo impedance control with simple movements"""
        print("Demo: Impedance Control")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Create simple oscillating target
            t = time.time() - start_time
            target_pose = self.stable_pose.copy()
            
            # Add simple arm movements
            for i, name in enumerate(self.joint_names):
                if "shoulder" in name.lower():
                    target_pose[i] = 0.2 * math.sin(t * 2)
                elif "elbow" in name.lower():
                    target_pose[i] = 0.1 * math.sin(t * 3)
            
            self.simple_impedance_control(target_pose)
            p.stepSimulation()
            time.sleep(1.0/240.0)
    
    def demo_admittance_control(self, duration=3):
        """Demo admittance control - responding to virtual forces"""
        print("Demo: Admittance Control")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            t = time.time() - start_time
            
            # Start from stable pose
            target_pose = self.stable_pose.copy()
            
            # Simulate external forces as sine waves
            virtual_force = np.zeros(self.num_joints)
            for i, name in enumerate(self.joint_names):
                if "shoulder" in name.lower():
                    virtual_force[i] = 5.0 * math.sin(t * 3)
            
            # Admittance: position changes in response to force
            # Simple implementation: integrate force to get position
            admittance_effect = virtual_force * 0.01  # Scale factor
            target_pose += admittance_effect
            
            self.simple_impedance_control(target_pose)
            p.stepSimulation()
            time.sleep(1.0/240.0)
    
    def simple_inverse_kinematics(self):
        """Simple IK demo using built-in functions carefully"""
        print("Demo: Simple Inverse Kinematics")
        
        # Use a simple approach - direct position control for arms
        target_angles = self.stable_pose.copy()
        
        # Move right arm to a specific position
        for i, name in enumerate(self.joint_names):
            if "right_shoulder" in name.lower():
                target_angles[i] = 0.5
            elif "right_elbow" in name.lower():
                target_angles[i] = -0.3
        
        # Apply gradually
        steps = 50
        current_pose, _ = self.get_joint_states()
        
        for step in range(steps):
            # Interpolate between current and target
            alpha = step / steps
            interpolated_pose = current_pose * (1 - alpha) + target_angles * alpha
            
            self.simple_impedance_control(interpolated_pose)
            p.stepSimulation()
            time.sleep(1.0/240.0)
        
        # Hold for a moment
        for _ in range(50):
            self.simple_impedance_control(target_angles)
            p.stepSimulation()
            time.sleep(1.0/240.0)
        
        # Return to stable pose
        for step in range(steps):
            alpha = step / steps
            interpolated_pose = target_angles * (1 - alpha) + self.stable_pose * alpha
            self.simple_impedance_control(interpolated_pose)
            p.stepSimulation()
            time.sleep(1.0/240.0)
    
    def demo_hjb_simple(self):
        """Simple HJB demonstration without complex math"""
        print("Demo: HJB Concept")
        
        # Simple value iteration for a 1D problem
        states = np.linspace(-1, 1, 10)
        goal_state = 0.0
        
        # Initialize value function
        V = np.abs(states - goal_state)
        
        print("Initial costs:", V)
        
        # Simple value iteration (greatly simplified)
        for iteration in range(5):
            V_new = np.copy(V)
            for i in range(len(states)):
                # Consider moving left, right, or staying
                costs = []
                if i > 0:
                    costs.append(V[i-1] + 0.1)  # cost to move left
                if i < len(states) - 1:
                    costs.append(V[i+1] + 0.1)  # cost to move right
                costs.append(V[i])  # cost to stay
                
                V_new[i] = min(costs)
            
            V = V_new
            print(f"Iteration {iteration+1}: {V}")
        
        print("HJB demonstration completed")
        return V
    
    def release_fixed_base(self):
        """Release the fixed base to allow falling (for advanced demos)"""
        p.removeConstraint(self.humanoidId)
        
    def run_all_demos(self):
        """Run all demonstrations safely"""
        print("=== Starting Robust Humanoid Demos ===")
        
        try:
            # Demo 1: Impedance Control
            self.demo_impedance_control(duration=3)
            time.sleep(1)
            
            # Demo 2: Admittance Control  
            self.demo_admittance_control(duration=3)
            time.sleep(1)
            
            # Demo 3: Simple IK
            self.simple_inverse_kinematics()
            time.sleep(1)
            
            # Demo 4: HJB
            self.demo_hjb_simple()
            time.sleep(1)
            
            # Final combined demo
            print("Final Combined Demo")
            self.demo_impedance_control(duration=3)
            
        except Exception as e:
            print(f"Demo error: {e}")
    
    def close(self):
        """Safe cleanup"""
        try:
            p.disconnect()
            print("Clean shutdown completed")
        except:
            print("Shutdown completed")

def main():
    print("Initializing Robust Humanoid Simulation...")
    sim = RobustHumanoid()
    
    try:
        # Let it stabilize
        print("Stabilizing...")
        for _ in range(100):
            sim.simple_impedance_control(sim.stable_pose)
            p.stepSimulation()
            time.sleep(1.0/240.0)
        
        # Run demos
        sim.run_all_demos()
        
        print("All demonstrations completed successfully!")
        
    except KeyboardInterrupt:
        print("Simulation interrupted by user")
    except Exception as e:
        print(f"Simulation error: {e}")
    finally:
        sim.close()

if __name__ == "__main__":
    main()
