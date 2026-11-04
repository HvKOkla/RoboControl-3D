
```markdown
# RoboControl 3D

**Web-based 6-DOF robotic arm simulator featuring Jacobian-based Inverse Kinematics, DH parameters, and trajectory generation.**

## Overview
This project demonstrates advanced web-based 3D rendering, mathematical kinematics, and efficient state management in the browser. It bridges the gap between interactive web development and robotics engineering.

## Mathematical Validation (Python, C++ & Octave)

To prove the mathematical accuracy of the WebGL simulator, the Forward Kinematics (DH parameters) logic has been ported and validated in three additional languages. All scripts output the exact same end-effector coordinates (X, Y, Z), ensuring mathematical consistency across all platforms.

### 1. Python (`python_examples/`)
A script using `numpy` that replicates the exact joint transformations. Perfect for academic research and ROS prototyping.
*   **Run it:** `python python_examples/ik_solver.py`

**Python 3D Trajectory Visualization:**
By running the Python script, it calculates the Forward Kinematics and plots the trajectory using Matplotlib (Dark Mode).
<img width="479" height="419" alt="trajectory_plot" src="https://github.com/user-attachments/assets/23eca3fc-c157-4b3d-a9ae-001d1134d815" />

### 2. C++ (`cpp_examples/`)
A high-performance version using raw matrix multiplication and CMake, demonstrating how the algorithm would run on an industrial robot controller.
*   **Build it:** Use `cmake` or standard `g++ forward_kinematics.cpp -o kinematics_test`

### 3. Octave/MATLAB (`octave_examples/`)
A `.m` script for academic validation and rapid mathematical prototyping.
*   **Run it:** `octave octave_examples/kinematics.m`

---

## 🛠️ Tech Stack
*   **Rendering:** Three.js (WebGL), PBR Materials, Custom HDR Environment
*   **Math:** Denavit-Hartenberg (DH), Jacobian IK, Quaternions
*   **Architecture:** Vanilla JavaScript (ES6), HTML5/CSS3

## ✨ Core Features
*   **Real-time IK:** Drag the target to solve joint angles dynamically.
*   **Trajectory Export:** Generate JSON, G-Code, and Python scripts.
*   **Collision Safety:** Math-based joint limits and ground avoidance validation.
*   **Workspace Cloud:** 3D point cloud generation of reachable areas.

## 🚀 Quick Start
No build tools required for the web app. Just clone and open:

```bash
git clone https://github.com/HvKOkla/RoboControl-3D.git
cd RoboControl-3D
# Open index.html in your browser
```

## 🎮 Controls
*   **Mouse Drag:** Rotate camera / Move IK target.
*   **Sliders:** Manual joint control.
*   **Space / Esc:** Trigger E-STOP (Press `R` to reset).

## 🤝 Contributing
We welcome contributions! Check out the [open issues](https://github.com/HvKOkla/RoboControl-3D/issues) to see what needs to be done. 
Look for the `good first issue` label if you're new to the project.

## 📄 License
MIT License.
```
