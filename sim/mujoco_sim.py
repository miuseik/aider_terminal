"""
MuJoCo Simulation with World Coordinate System Display
"""

import mujoco
import mujoco.viewer
import numpy as np


def create_mujoco_simulation():
    """Create and run MuJoCo simulation with world coordinate system"""
    
    # Load the XML model
    model = mujoco.MjModel.from_xml_path("mujoco_world.xml")
    data = mujoco.MjData(model)
    
    print("=" * 60)
    print("MuJoCo Simulation - World Coordinate System")
    print("=" * 60)
    print("\nCoordinate System:")
    print("  X-axis (Red):   Right direction")
    print("  Y-axis (Green): Forward direction")
    print("  Z-axis (Blue):  Up direction")
    print("\nControls:")
    print("  - Left mouse drag: Rotate camera")
    print("  - Right mouse drag: Pan camera")
    print("  - Scroll: Zoom in/out")
    print("  - Press ESC or close window to exit")
    print("=" * 60)
    
    # Launch the viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set initial camera position
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -30
        
        # Run simulation
        while viewer.is_running():
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Optional: Print box position
            box_pos = data.geom_xpos[3]  # Box geom index
            if np.any(box_pos != 0):
                print(f"\rBox position: x={box_pos[0]:.2f}, y={box_pos[1]:.2f}, z={box_pos[2]:.2f}", end="")
    
    print("\n\nSimulation ended.")


if __name__ == "__main__":
    create_mujoco_simulation()
