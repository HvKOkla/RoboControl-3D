
```markdown
# RoboControl 3D

**A browser-based 6-DOF robotic arm simulator featuring Jacobian-based Inverse Kinematics, Denavit-Hartenberg (DH) parameters, and real-time trajectory generation.**


<img width="600" alt="RoboControl 3D Trajectory" src="https://github.com/user-attachments/assets/23eca3fc-c157-4b3d-a9ae-001d1134d815" />

---

## What is this project?
RoboControl 3D is an interactive sandbox designed for robotics students, engineers, and developers. It allows you to visualize and experiment with the kinematics of a robotic arm directly in your web browser, without installing any heavy software (like MATLAB or ROS). 

Whether you want to understand how DH parameters work, test an Inverse Kinematics (IK) algorithm, or generate G-Code trajectories, this simulator provides a lightweight, zero-dependency environment.

## ✨ Core Features & How to Use Them

*   **Interactive 3D Workspace:** Rotate the camera (left click), zoom (scroll), and pan (right click) around a fully rendered 6-DOF industrial arm.
*   **Real-time Inverse Kinematics (IK):** Drag the green wireframe target in the 3D space. The arm will dynamically calculate the joint angles required to reach that position using a Jacobian-based solver.
*   **Manual Joint Control:** Use the sliders on the left panel to manually adjust each joint (J1 to J6). The 3D model and the telemetry panel will update instantly.
*   **Trajectory & Waypoint Editor:** 
    *   Position the robot and click **"+ Ajouter Point"** to save a waypoint.
    *   Click **"Exécuter Séquence"** to watch the robot smoothly interpolate between your saved points.
    *   Export your trajectory in **JSON, G-Code, or Python** for use with external controllers.
*   **Safety & Collision Validation:** A custom `UltraSafeValidator` engine prevents the robot from hitting the floor (Z < 0) or exceeding joint limits. 
*   **Emergency Stop (E-STOP):** Press `Space` or `Escape` to instantly halt all movements. Press `R` to reset the system.
*   **Workspace Cloud:** Click **"Nuage Espace"** to generate a 3D point cloud of all the positions the robot can reach.

---

## 🧮 Mathematical Validation (Python, C++ & Octave)

To prove the mathematical accuracy of the WebGL simulator, the Forward Kinematics logic has been ported and validated in three additional languages. **All scripts output the exact same end-effector coordinates (X, Y, Z)**, ensuring mathematical consistency across all platforms.

### 1. Python (`python_examples/`)
A script using `numpy` that replicates the exact joint transformations. It also includes a 3D visualization using `matplotlib` (shown at the top of this page).
*   **Run it:** `python python_examples/ik_solver.py`
*   **Requirements:** `pip install numpy matplotlib`

### 2. C++ (`cpp_examples/`)
A high-performance version using raw matrix multiplication and CMake, demonstrating how the algorithm would run on a real-time industrial robot controller.
*   **Build it:** Use CMake or standard `g++ forward_kinematics.cpp -o kinematics_test`

### 3. Octave/MATLAB (`octave_examples/`)
A `.m` script for academic research and rapid mathematical prototyping. 
*   **Run it:** `octave octave_examples/kinematics.m`

---

## 🛠️ Tech Stack
*   **Rendering:** Three.js (WebGL), PBR Materials, Custom HDR Environment mapping, ACES Filmic Tone Mapping.
*   **Mathematics:** Denavit-Hartenberg (DH) matrices, Jacobian IK, Euler angles & Quaternions (to avoid Gimbal Lock).
*   **Architecture:** Vanilla JavaScript (ES6 Classes), HTML5, CSS3 (Glassmorphism UI).

## 🚀 Getting Started
No build tools or `npm install` required for the web app. Just clone and open:

```bash
git clone https://github.com/HvKOkla/RoboControl-3D.git
cd RoboControl-3D
# Double-click index.html to run it in your browser
```
*(Note: Some browsers restrict loading local 3D models via `file://`. If you plan to use custom GLTF models, run a local server like VS Code's Live Server).*

## 🤝 Contributing
We welcome contributions from the open-source community! Whether you want to fix a bug, improve the UI, or implement advanced math algorithms (like analytical IK or RRT path planning), your help is appreciated.

*   Check out the [open issues](https://github.com/HvKOkla/RoboControl-3D/issues) to see what needs to be done.
*   Look for the `good first issue` label if you are new to open-source.

## 📄 License
This project is licensed under the **MIT License**. You are free to use, modify, and distribute it.
```
4. **Les prérequis Python** : On précise qu'il faut `pip install numpy matplotlib` pour que le script fonctionne.
5. **"Contributing"** : On précise qu'on cherche des gens pour faire des maths avancées (IK analytique, RRT), ce qui va attirer les ingénieurs.
