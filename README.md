
```markdown
# RoboControl 3D

**Web-based 6-DOF robotic arm simulator featuring Jacobian-based Inverse Kinematics, DH parameters, and trajectory generation.**

## Mathematical Validation (Python & C++)

To prove the mathematical accuracy of the WebGL simulator, the Forward Kinematics (DH parameters) logic has been ported and validated in two additional languages:

*   **Python (`python_examples/`)** : A script using `numpy` that replicates the exact joint transformations. Perfect for academic research and ROS prototyping.
    *   Run it with: `python python_examples/ik_solver.py`
*   **C++ (`cpp_examples/`)** : A high-performance version using raw matrix multiplication and CMake, demonstrating how the algorithm would run on an industrial robot controller.
    *   Build it with: `cmake` and `make` (or standard `g++`).
*   **Octave/MATLAB (`octave_examples/`)** : A `.m` script for academic validation. 
    *   Run it with: `octave octave_examples/kinematics.m`
	

Both scripts output the exact same end-effector coordinates (X, Y, Z) as the web interface, ensuring mathematical consistency across all platforms.

## Tech Stack
* **Rendering:** Three.js (WebGL), PBR Materials, Custom HDR Environment
* **Math:** Denavit-Hartenberg (DH), Jacobian IK, Quaternions
* **Architecture:** Vanilla JavaScript (ES6), HTML5/CSS3

## Core Features
* **Real-time IK:** Drag the target to solve joint angles dynamically.
* **Trajectory Export:** Generate JSON, G-Code, and Python scripts.
* **Collision Safety:** Math-based joint limits and ground avoidance validation.
* **Workspace Cloud:** 3D point cloud generation of reachable areas.

## Quick Start
No build tools required. Just clone and open:

```bash
git clone https://github.com/HvKOkla/RoboControl-3D.git
cd RoboControl-3D
# Open index.html in your browser
```

## Controls
* **Mouse Drag:** Rotate camera / Move IK target.
* **Sliders:** Manual joint control.
* **Space / Esc:** Trigger E-STOP (Press `R` to reset).

## 🤝 Contributing
We welcome contributions! Check out the [open issues](https://github.com/HvKOkla/RoboControl-3D/issues) to see what needs to be done. 
Look for the `good first issue` label if you're new to the project.

Please review our [Contributing Guidelines](CONTRIBUTING.md) and [Code of Conduct](CODE_OF_CONDUCT.md) before making your first contribution.

## 📄 License
MIT License.
```
