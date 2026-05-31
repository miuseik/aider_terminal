"""
PyBullet Simulation with World Coordinate System Display
"""

import pybullet as p
import pybullet_data
import time


def create_pybullet_simulation():
    """Create and run PyBullet simulation with world coordinate system"""
    
    # Connect to PyBullet with GUI
    client_id = p.connect(p.GUI)
    
    # Set additional search path for data files
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    print("=" * 60)
    print("PyBullet Simulation - World Coordinate System")
    print("=" * 60)
    print("\nCoordinate System:")
    print("  X-axis (Red):   Right direction")
    print("  Y-axis (Green): Forward direction")
    print("  Z-axis (Blue):  Up direction")
    print("\nControls:")
    print("  - Left mouse drag: Rotate camera")
    print("  - Right mouse drag: Pan camera")
    print("  - Scroll: Zoom in/out")
    print("  - Press 'q' or close window to exit")
    print("=" * 60)
    
    # Set gravity
    p.setGravity(0, 0, -9.81)
    
    # Set camera position
    p.resetDebugVisualizerCamera(
        cameraDistance=3.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.5]
    )
    
    # Create ground plane
    ground_collision = p.createCollisionShape(p.GEOM_PLANE)
    ground_visual = p.createVisualShape(
        p.GEOM_PLANE,
        rgbaColor=[0.9, 0.9, 0.9, 1]
    )
    ground = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=ground_collision,
        baseVisualShapeIndex=ground_visual
    )
    
    # Create coordinate system axes using cylinders and cones
    axis_radius = 0.02
    axis_length = 1.0
    arrow_size = 0.05
    
    # X-axis (Red)
    x_axis_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=axis_radius, height=axis_length)
    x_axis_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=axis_radius, length=axis_length, rgbaColor=[1, 0, 0, 1])
    x_arrow_collision = p.createCollisionShape(p.GEOM_CONE, radius=arrow_size, height=0.2)
    x_arrow_visual = p.createVisualShape(p.GEOM_CONE, radius=arrow_size, length=0.2, rgbaColor=[1, 0, 0, 1])
    
    x_axis = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=x_axis_collision,
        baseVisualShapeIndex=x_axis_visual,
        basePosition=[0.5, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([0, 1.5708, 0])  # Rotate 90 degrees around Y
    )
    
    x_arrow = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=x_arrow_collision,
        baseVisualShapeIndex=x_arrow_visual,
        basePosition=[1.1, 0, 0],
        baseOrientation=p.getQuaternionFromEuler([0, 1.5708, 0])
    )
    
    # Y-axis (Green)
    y_axis_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=axis_radius, height=axis_length)
    y_axis_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=axis_radius, length=axis_length, rgbaColor=[0, 1, 0, 1])
    y_arrow_collision = p.createCollisionShape(p.GEOM_CONE, radius=arrow_size, height=0.2)
    y_arrow_visual = p.createVisualShape(p.GEOM_CONE, radius=arrow_size, length=0.2, rgbaColor=[0, 1, 0, 1])
    
    y_axis = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=y_axis_collision,
        baseVisualShapeIndex=y_axis_visual,
        basePosition=[0, 0.5, 0],
        baseOrientation=p.getQuaternionFromEuler([-1.5708, 0, 0])  # Rotate 90 degrees around X
    )
    
    y_arrow = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=y_arrow_collision,
        baseVisualShapeIndex=y_arrow_visual,
        basePosition=[0, 1.1, 0],
        baseOrientation=p.getQuaternionFromEuler([-1.5708, 0, 0])
    )
    
    # Z-axis (Blue)
    z_axis_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=axis_radius, height=axis_length)
    z_axis_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=axis_radius, length=axis_length, rgbaColor=[0, 0, 1, 1])
    z_arrow_collision = p.createCollisionShape(p.GEOM_CONE, radius=arrow_size, height=0.2)
    z_arrow_visual = p.createVisualShape(p.GEOM_CONE, radius=arrow_size, length=0.2, rgbaColor=[0, 0, 1, 1])
    
    z_axis = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=z_axis_collision,
        baseVisualShapeIndex=z_axis_visual,
        basePosition=[0, 0, 0.5]
    )
    
    z_arrow = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=z_arrow_collision,
        baseVisualShapeIndex=z_arrow_visual,
        basePosition=[0, 0, 1.1]
    )
    
    # Origin sphere
    origin_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.05)
    origin_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[1, 1, 1, 1])
    origin = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=origin_collision,
        baseVisualShapeIndex=origin_visual,
        basePosition=[0, 0, 0]
    )
    
    # Create a box to demonstrate the coordinate system
    box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1])
    box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.1, 0.1, 0.1], rgbaColor=[0.7, 0.7, 0.7, 1])
    box = p.createMultiBody(
        baseMass=1.0,
        baseCollisionShapeIndex=box_collision,
        baseVisualShapeIndex=box_visual,
        basePosition=[0.5, 0.5, 0.5]
    )
    
    # Run simulation
    try:
        while p.isConnected():
            # Step simulation
            p.stepSimulation()
            
            # Get box position
            box_pos, box_orn = p.getBasePositionAndOrientation(box)
            print(f"\rBox position: x={box_pos[0]:.2f}, y={box_pos[1]:.2f}, z={box_pos[2]:.2f}", end="")
            
            # Small delay for visualization
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    
    # Disconnect
    p.disconnect()
    print("\n\nSimulation ended.")


if __name__ == "__main__":
    create_pybullet_simulation()
