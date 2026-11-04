
```markdown
# RoboControl 3D

**Web-based 6-DOF robotic arm simulator featuring Jacobian-based Inverse Kinematics, DH parameters, and trajectory generation.**

🔗 **[Live Demo](https://hvkokla.github.io/RoboControl-3D/)**

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

## 📄 License
MIT License.
```
