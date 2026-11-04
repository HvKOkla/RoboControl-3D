# RoboControl 3D

**An interactive WebGL-based 6-DOF robotic arm simulator featuring real-time Inverse Kinematics, DH parameters, and trajectory generation.**


<img width="600" alt="Trajectory Plot" src="https://github.com/user-attachments/assets/23eca3fc-c157-4b3d-a9ae-001d1134d815" />

## Core Features
*   **Real-time IK:** Drag the target in 3D space to dynamically solve joint angles (Jacobian-based).
*   **Trajectory Editor:** Record waypoints and export them to **JSON, G-Code, or Python**.
*   **Safety Validation:** Built-in collision checks (joint limits, ground avoidance).
*   **Math Validation:** The kinematics logic is ported and validated in **Python, C++, and Octave** (see folders).

## 🚀 Quick Start
No build tools required. Just clone and open:

```bash
git clone https://github.com/HvKOkla/RoboControl-3D.git
cd RoboControl-3D
# Open index.html in your browser

🧮 Validation Scripts
Python: python python_examples/ik_solver.py (Requires numpy & matplotlib)

C++: Build with g++ forward_kinematics.cpp -o test in cpp_examples/

Octave: Run octave octave_examples/kinematics.m

🎮 Controls
Mouse: Rotate / Zoom / Move IK target

Space / Esc: Trigger E-STOP (Press R to reset)

Sliders: Manual joint control

🤝 Contributing
Contributions are welcome! Check the open issues or look for good first issue labels.

Please review our [Contributing Guidelines](CONTRIBUTING.md) and [Code of Conduct](CODE_OF_CONDUCT.md) before making your first contribution.

## 📄 License
MIT License.
```
