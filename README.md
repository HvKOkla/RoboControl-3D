
This project was built to demonstrate advanced web-based 3D rendering, mathematical kinematics, and efficient state management in the browser.

## Key Features

*   **Real-time Inverse Kinematics (IK):** A custom Jacobian-based solver allows dragging the end-effector in 3D space, with the arm adapting its joints in real-time.
*   **Forward Kinematics (FK):** Implements Denavit-Hartenberg (DH) parameters to compute the robot's spatial transformations.
*   **PBR Rendering:** High-quality metallic materials, procedural HDR environment mapping, soft shadows (PCF), and ACES Filmic tone mapping.
*   **Trajectory Editor:** Record, visualize, and export waypoints (JSON, G-Code, Python) for external robot controllers.
*   **Safety Validation:** A dedicated validator ensures joints remain within limits, prevents floor collisions, and detects workspace boundary violations.
*   **Interactive Workspace Cloud:** Generates a point cloud representing the reachable workspace using random joint configurations.

## 🛠️ Technical Stack

*   **Rendering:** Three.js (r130)
*   **Architecture:** Vanilla JavaScript (ES6 Classes)
*   **Math:** Custom Matrix & Vector operations for kinematics.
*   **UI/UX:** HTML5/CSS3 (Glassmorphism design, responsive HUD).

## 🧠 Technical Highlights (Why this matters)

This project goes beyond a simple Three.js scene. It demonstrates core competencies required for complex 3D web applications:

1.  **State Management & Interpolation:** Smooth animation between waypoints is handled via custom easing functions (`smoothstep`), ensuring no sudden jumps in the robot's movement.
2.  **Object-Oriented Design:** Logic is decoupled from rendering (`UltraSafeValidator`, `KinematicChain`, `Robot3DView`).
3.  **Performance:** The scene updates transformation matrices directly (`matrixAutoUpdate = false`) to minimize the render loop overhead, a technique essential for handling high object counts.
4.  **Math & Space:** Deep understanding of Euler angles, quaternions, and transformation matrices to avoid Gimbal Lock.

## 🚀 Getting Started

The project has **zero build dependencies** and runs entirely in the browser.

1.  Clone the repository:
    ```bash
    git clone https://github.com/your-username/robot-simulator.git
    ```
2.  Open `index.html` in your favorite web browser (or use a local server like `Live Server` in VS Code).

> *Note: Some browsers restrict loading GLTF models via `file://`, so a local server is recommended if you plan to use 3D model imports.*

## 🎮 How to Use

*   **Select DOF:** Switch between a simplified 3-DOF arm and a full 6-DOF industrial arm.
*   **Drag the Target:** Click and drag the green wireframe sphere in the 3D view to trigger the IK solver.
*   **Joints & Sliders:** Manually control each joint using the left panel sliders.
*   **Workspace:** Click "Nuage Espace" to visualize the reachable area of the robot.
*   **Record & Export:** Add waypoints, run the trajectory, and export it to JSON, G-Code, or Python.

## 🔮 Future Improvements (v2.0)

*   [ ] **InstancedMesh optimization:** For rendering thousands of trajectory points without performance drops.
*   [ ] **WebWorker integration:** Move the IK solver to a background thread to keep the main thread at 60 FPS.
*   [ ] **GLTF Loader:** Fully implement custom 3D models for the robot links.

## 📄 License

MIT License. Feel free to use, modify, and distribute.
```
