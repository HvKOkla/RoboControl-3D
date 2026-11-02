
class UltraSafeValidator {
  constructor(config) {
    this.updateConfig(config);
    this.isEStopped = false;
  }

  updateConfig(config) {
    this.jointLimits = config.jointLimits;
    this.maxReach = config.maxReach;
    this.minGroundZ = config.minGroundZ;
  }

  triggerEStop() { this.isEStopped = true; }
  resetEStop() { this.isEStopped = false; }

  validateState(jointAnglesDeg, endEffectorPos, obstacles = []) {
    if (this.isEStopped) {
      return { safe: false, status: 'danger', message: '🚨 ARRÊT D\'URGENCE ACTIF !' };
    }

    for (let a of jointAnglesDeg) {
      if (!Number.isFinite(a)) {
        return { safe: false, status: 'danger', message: 'Erreur : Données angulaires NaN/Infini.' };
      }
    }

    for (let i = 0; i < jointAnglesDeg.length; i++) {
      const limit = this.jointLimits[i];
      if (limit && (jointAnglesDeg[i] < limit.min || jointAnglesDeg[i] > limit.max)) {
        return { 
          safe: false, 
          status: 'warning', 
          message: `Butée Joint ${i + 1} (${jointAnglesDeg[i].toFixed(1)}° hors [${limit.min}°, ${limit.max}°])` 
        };
      }
    }

    if (endEffectorPos.z < this.minGroundZ) {
      return { 
        safe: false, 
        status: 'danger', 
        message: `Risque collision sol (Z: ${endEffectorPos.z.toFixed(2)} < ${this.minGroundZ})` 
      };
    }

    // Détection de collision basique avec les obstacles enregistrés
    for (const obs of obstacles) {
      if (obs.box && obs.box.containsPoint(endEffectorPos)) {
        return {
          safe: false,
          status: 'danger',
          message: 'Collision détectée avec un obstacle !'
        };
      }
    }

    const dist = Math.sqrt(endEffectorPos.x ** 2 + endEffectorPos.y ** 2 + endEffectorPos.z ** 2);
    if (dist > this.maxReach) {
      return { 
        safe: false, 
        status: 'warning', 
        message: `Hors de portée (${dist.toFixed(1)} cm > max ${this.maxReach} cm)` 
      };
    }

    return { safe: true, status: 'success', message: 'Système nominal : Aucune anomalie' };
  }

  clampAngle(angleDeg, jointIndex) {
    const limit = this.jointLimits[jointIndex];
    if (!limit) return angleDeg;
    return Math.max(limit.min, Math.min(limit.max, angleDeg));
  }
}


class DHLink {
  constructor(a, alpha, d, thetaOffset = 0) {
    this.a = a;
    this.alpha = alpha;
    this.d = d;
    this.thetaOffset = thetaOffset;
  }

  getMatrix(thetaRad) {
    const totalTheta = thetaRad + this.thetaOffset;
    const ct = Math.cos(totalTheta), st = Math.sin(totalTheta);
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);

    const m = new THREE.Matrix4();
    m.set(
      ct, -st * ca,  st * sa, this.a * ct,
      st,  ct * ca, -ct * sa, this.a * st,
       0,       sa,       ca,        this.d,
       0,        0,        0,             1
    );
    return m;
  }
}

class KinematicChain {
  constructor(dhParams) {
    this.setParams(dhParams);
  }

  setParams(dhParams) {
    this.links = dhParams.map(p => new DHLink(p.a, p.alpha, p.d, p.thetaOffset));
  }

  forwardKinematics(jointAnglesDeg) {
    let currentMatrix = new THREE.Matrix4().identity();
    const transforms = [currentMatrix.clone()];

    for (let i = 0; i < this.links.length; i++) {
      const rad = (jointAnglesDeg[i] * Math.PI) / 180;
      const linkMatrix = this.links[i].getMatrix(rad);
      currentMatrix.multiply(linkMatrix);
      transforms.push(currentMatrix.clone());
    }

    const pos = new THREE.Vector3();
    const quat = new THREE.Quaternion();
    const scale = new THREE.Vector3();
    currentMatrix.decompose(pos, quat, scale);

    return { endEffectorPos: pos, transforms };
  }

  solveIK(targetPos, currentAnglesDeg, maxIter = 25, damping = 0.5) {
    let angles = [...currentAnglesDeg];
    const deltaDeg = 0.1;

    for (let iter = 0; iter < maxIter; iter++) {
      const currentFK = this.forwardKinematics(angles);
      const error = new THREE.Vector3().subVectors(targetPos, currentFK.endEffectorPos);

      if (error.length() < 0.1) break;

      const J = [];
      for (let j = 0; j < angles.length; j++) {
        angles[j] += deltaDeg;
        const fkPlus = this.forwardKinematics(angles);
        angles[j] -= deltaDeg;

        const diff = new THREE.Vector3().subVectors(fkPlus.endEffectorPos, currentFK.endEffectorPos).divideScalar(deltaDeg);
        J.push(diff);
      }

      for (let j = 0; j < angles.length; j++) {
        const dot = J[j].dot(error);
        angles[j] += dot * damping;
      }
    }
    return angles;
  }
}


class Robot3DView {
  constructor(containerEl, onTargetMove) {
    this.container = containerEl;
    this.onTargetMove = onTargetMove;
    this.workspaceMesh = null;
    this.fingerLeft = null;
    this.fingerRight = null;
    this.attachedObject = null;
    this.obstacles = [];
    this.importedModels = [];
    this.initScene();
  }

  createHDRProceduralEnvironment() {
    const canvas = document.createElement('canvas');
    canvas.width = 256;
    canvas.height = 128;
    const ctx = canvas.getContext('2d');
    const grad = ctx.createLinearGradient(0, 0, 0, 128);
    grad.addColorStop(0, '#38bdf8');
    grad.addColorStop(0.5, '#1e293b');
    grad.addColorStop(1, '#0f172a');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, 256, 128);

    const texture = new THREE.CanvasTexture(canvas);
    texture.mapping = THREE.EquirectangularReflectionMapping;
    return texture;
  }

  initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f172a);
    
    const envMap = this.createHDRProceduralEnvironment();
    this.scene.environment = envMap;

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
    this.camera.position.set(80, 70, 110);

    this.renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: "high-performance" });
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;
    this.container.appendChild(this.renderer.domElement);

    this.orbit = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.set(0, 20, 0);
    this.orbit.update();

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
    dirLight.position.set(60, 120, 60);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.bias = -0.0001;
    this.scene.add(dirLight);

    const spotLight = new THREE.SpotLight(0x38bdf8, 1.5, 150, Math.PI / 4, 0.5);
    spotLight.position.set(-50, 80, -50);
    this.scene.add(spotLight);

    const grid = new THREE.GridHelper(200, 40, 0x38bdf8, 0x334155);
    grid.position.y = 0.01;
    this.scene.add(grid);

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0b1329,
      roughness: 0.4,
      metalness: 0.6
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    this.scene.add(floor);

    this.initEnvironmentObjects();

    this.targetMesh = new THREE.Mesh(
      new THREE.SphereGeometry(2, 16, 16),
      new THREE.MeshBasicMaterial({ color: 0x10b981, wireframe: true })
    );
    this.scene.add(this.targetMesh);

    this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
    this.transformControls.attach(this.targetMesh);
    this.transformControls.setSize(0.75);
    this.scene.add(this.transformControls);

    this.transformControls.addEventListener('dragging-changed', (event) => {
      this.orbit.enabled = !event.value;
    });

    this.transformControls.addEventListener('change', () => {
      if (this.transformControls.dragging) {
        this.onTargetMove(this.targetMesh.position);
      }
    });

    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);
  }

  initEnvironmentObjects() {
    const tableGeo = new THREE.BoxGeometry(40, 12, 30);
    const tableMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3, metalness: 0.7 });
    this.tableMesh = new THREE.Mesh(tableGeo, tableMat);
    this.tableMesh.position.set(25, 6, 15);
    this.tableMesh.castShadow = true;
    this.tableMesh.receiveShadow = true;
    this.scene.add(this.tableMesh);

    // Bounding box pour la gestion de collision
    const tableBox = new THREE.Box3().setFromObject(this.tableMesh);
    this.obstacles.push({ mesh: this.tableMesh, box: tableBox });

    const zoneGeo = new THREE.CylinderGeometry(8, 8, 0.2, 32);
    const zoneMat = new THREE.MeshBasicMaterial({ color: 0x10b981, transparent: true, opacity: 0.4, wireframe: true });
    const dropZone = new THREE.Mesh(zoneGeo, zoneMat);
    dropZone.position.set(-25, 0.1, 20);
    this.scene.add(dropZone);

    const cubeGeo = new THREE.BoxGeometry(4, 4, 4);
    const cubeMat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, roughness: 0.2, metalness: 0.3 });
    this.targetCube = new THREE.Mesh(cubeGeo, cubeMat);
    this.targetCube.position.set(20, 14, 15);
    this.targetCube.castShadow = true;
    this.targetCube.receiveShadow = true;
    this.scene.add(this.targetCube);

    const cylGeo = new THREE.CylinderGeometry(2, 2, 5, 24);
    const cylMat = new THREE.MeshStandardMaterial({ color: 0xef4444, roughness: 0.3, metalness: 0.8 });
    this.targetCyl = new THREE.Mesh(cylGeo, cylMat);
    this.targetCyl.position.set(30, 14.5, 10);
    this.targetCyl.castShadow = true;
    this.targetCyl.receiveShadow = true;
    this.scene.add(this.targetCyl);
  }

  loadGLTFModel(url, position = new THREE.Vector3(0, 0, 0), scale = new THREE.Vector3(1, 1, 1)) {
    if (typeof THREE.GLTFLoader === 'undefined') {
      console.warn('GLTFLoader non chargé.');
      return;
    }
    const loader = new THREE.GLTFLoader();
    loader.load(url, (gltf) => {
      const model = gltf.scene;
      model.position.copy(position);
      model.scale.copy(scale);
      this.scene.add(model);
      this.importedModels.push(model);

      const box = new THREE.Box3().setFromObject(model);
      this.obstacles.push({ mesh: model, box: box });
    });
  }

  resetObjectPositions() {
    if (this.attachedObject) {
      this.scene.attach(this.attachedObject);
      this.attachedObject = null;
    }
    this.targetCube.position.set(20, 14, 15);
    this.targetCube.rotation.set(0, 0, 0);
    this.targetCyl.position.set(30, 14.5, 10);
    this.targetCyl.rotation.set(0, 0, 0);
  }

  disposeGroup(group) {
    while (group.children.length > 0) {
      const obj = group.children[0];
      group.remove(obj);
      if (obj.geometry) obj.geometry.dispose();
      if (obj.material) {
        if (Array.isArray(obj.material)) obj.material.forEach(m => m.dispose());
        else obj.material.dispose();
      }
    }
  }

  buildRobot(dhParams) {
    this.disposeGroup(this.robotGroup);

    const pbrMetalOrange = new THREE.MeshStandardMaterial({ color: 0xf97316, roughness: 0.2, metalness: 0.8 });
    const pbrDarkMetal = new THREE.MeshStandardMaterial({ color: 0x334155, roughness: 0.3, metalness: 0.9 });
    const pbrChrome = new THREE.MeshStandardMaterial({ color: 0xe2e8f0, roughness: 0.1, metalness: 0.95 });

    this.visualLinks = [];

    const baseMesh = new THREE.Mesh(new THREE.CylinderGeometry(12, 14, 4, 32), pbrDarkMetal);
    baseMesh.position.y = 2;
    baseMesh.castShadow = true;
    baseMesh.receiveShadow = true;
    this.robotGroup.add(baseMesh);

    for (let i = 0; i < dhParams.length; i++) {
      const group = new THREE.Group();
      
      const jMesh = new THREE.Mesh(new THREE.SphereGeometry(3.2, 24, 24), pbrDarkMetal);
      jMesh.castShadow = true;
      group.add(jMesh);

      const len = dhParams[i].a || dhParams[i].d || 8;
      const lGeo = new THREE.CylinderGeometry(1.8, 2.2, len, 24);
      lGeo.translate(0, len / 2, 0);
      lGeo.rotateZ(-Math.PI / 2);

      const lMesh = new THREE.Mesh(lGeo, i % 2 === 0 ? pbrMetalOrange : pbrChrome);
      lMesh.castShadow = true;
      lMesh.receiveShadow = true;
      group.add(lMesh);

      if (i === dhParams.length - 1) {
        const gripperGroup = new THREE.Group();
        gripperGroup.position.set(len, 0, 0);

        const basePince = new THREE.Mesh(new THREE.BoxGeometry(2, 6, 2), pbrDarkMetal);
        gripperGroup.add(basePince);

        const fingerGeo = new THREE.BoxGeometry(4, 1, 1);
        fingerGeo.translate(2, 0, 0);

        this.fingerLeft = new THREE.Mesh(fingerGeo, pbrChrome);
        this.fingerLeft.position.set(0, 2, 0);
        this.fingerLeft.castShadow = true;

        this.fingerRight = new THREE.Mesh(fingerGeo, pbrChrome);
        this.fingerRight.position.set(0, -2, 0);
        this.fingerRight.castShadow = true;

        gripperGroup.add(this.fingerLeft);
        gripperGroup.add(this.fingerRight);

        this.gripperBaseMesh = basePince;
        group.add(gripperGroup);
      }

      this.visualLinks.push(group);
      this.robotGroup.add(group);
    }
  }

  setGripperOpenness(percent) {
    if (!this.fingerLeft || !this.fingerRight) return;
    const offset = (percent / 100) * 1.8 + 0.4;
    this.fingerLeft.position.y = offset;
    this.fingerRight.position.y = -offset;
  }

  attachObjectToGripper(obj) {
    if (this.fingerLeft && obj) {
      this.fingerLeft.parent.attach(obj);
      this.attachedObject = obj;
    }
  }

  detachObject(scene) {
    if (this.attachedObject) {
      scene.attach(this.attachedObject);
      this.attachedObject = null;
    }
  }

  toggleWorkspaceCloud(dhParams, kinematics) {
    if (this.workspaceMesh) {
      this.scene.remove(this.workspaceMesh);
      this.workspaceMesh.geometry.dispose();
      this.workspaceMesh.material.dispose();
      this.workspaceMesh = null;
      return false;
    }

    const points = [];
    const samples = 3500;

    for (let i = 0; i < samples; i++) {
      const randAngles = dhParams.map(() => (Math.random() - 0.5) * 240);
      const fk = kinematics.forwardKinematics(randAngles);
      if (fk.endEffectorPos.z >= 0) {
        points.push(fk.endEffectorPos.x, fk.endEffectorPos.y, fk.endEffectorPos.z);
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
    const mat = new THREE.PointsMaterial({ color: 0x38bdf8, size: 0.8, transparent: true, opacity: 0.35 });

    this.workspaceMesh = new THREE.Points(geo, mat);
    this.scene.add(this.workspaceMesh);
    return true;
  }

  updateTransforms(transforms, updateGizmo = true) {
    for (let i = 0; i < this.visualLinks.length; i++) {
      if (transforms[i]) {
        this.visualLinks[i].matrixAutoUpdate = false;
        this.visualLinks[i].matrix.copy(transforms[i]);
      }
    }
    if (updateGizmo && transforms.length > 0) {
      const lastMat = transforms[transforms.length - 1];
      const pos = new THREE.Vector3();
      lastMat.decompose(pos, new THREE.Quaternion(), new THREE.Vector3());
      this.targetMesh.position.copy(pos);
    }
  }

  render() {
    this.renderer.render(this.scene, this.camera);
  }
}


class WaypointManager {
  constructor(containerEl, onUpdate) {
    this.container = containerEl;
    this.onUpdate = onUpdate;
    this.waypoints = [];
  }

  addWaypoint(angles, pos) {
    this.waypoints.push({
      id: Date.now(),
      angles: [...angles],
      pos: { x: pos.x, y: pos.y, z: pos.z }
    });
    this.render();
    if (this.onUpdate) this.onUpdate();
  }

  clear() {
    this.waypoints = [];
    this.render();
    if (this.onUpdate) this.onUpdate();
  }

  render() {
    if (this.waypoints.length === 0) {
      this.container.innerHTML = `<div class="empty-state">Aucun point d'étape enregistré</div>`;
      return;
    }

    this.container.innerHTML = this.waypoints.map((wp, i) => `
      <div class="waypoint-item">
        <span>#${i + 1} [X:${wp.pos.x.toFixed(0)}, Y:${wp.pos.y.toFixed(0)}, Z:${wp.pos.z.toFixed(0)}]</span>
        <span style="color: var(--accent);">${wp.angles.map(a => Math.round(a) + '°').join(', ')}</span>
      </div>
    `).join('');
  }

  exportJSON() {
    const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(this.waypoints, null, 2));
    this.downloadFile(dataStr, "trajectory_waypoints.json");
  }

  exportGCode() {
    let gcode = "; G-Code Généré par Simulateur Robotique\nG21 ; Unités en mm\nG90 ; Positionnement absolu\nG28 ; Homing\n\n";
    this.waypoints.forEach((wp, i) => {
      gcode += `; Waypoint ${i + 1}\nG01 X${(wp.pos.x * 10).toFixed(2)} Y${(wp.pos.y * 10).toFixed(2)} Z${(wp.pos.z * 10).toFixed(2)} F1500\n`;
    });
    const dataStr = "data:text/plain;charset=utf-8," + encodeURIComponent(gcode);
    this.downloadFile(dataStr, "robot_trajectory.gcode");
  }

  exportPython() {
    let py = "# Script Python de Trajectoire Robotique (ROS / PyBullet)\nimport time\n\nwaypoints = " + JSON.stringify(this.waypoints.map(w => w.angles), null, 2) + "\n\n";
    py += "def execute_trajectory(robot):\n    for angles in waypoints:\n        robot.set_joint_angles(angles)\n        time.sleep(0.5)\n";
    const dataStr = "data:text/plain;charset=utf-8," + encodeURIComponent(py);
    this.downloadFile(dataStr, "robot_trajectory.py");
  }

  downloadFile(dataStr, filename) {
    const downloadAnchor = document.createElement('a');
    downloadAnchor.setAttribute("href", dataStr);
    downloadAnchor.setAttribute("download", filename);
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    downloadAnchor.remove();
  }
}


class RobotApp {
  constructor() {
    this.speedFactor = 1.0; // Vitesse de déplacement réglable

    this.dofConfigs = {
      '3': {
        dhParams: [
          { a: 0,  alpha: Math.PI / 2, d: 15, thetaOffset: 0 },
          { a: 30, alpha: 0,           d: 0,  thetaOffset: 0 },
          { a: 25, alpha: 0,           d: 0,  thetaOffset: 0 }
        ],
        limits: [
          { min: -150, max: 150 },
          { min: -60,  max: 90 },
          { min: -120, max: 60 }
        ],
        names: ["Base (J1)", "Épaule (J2)", "Coude (J3)"],
        maxReach: 55
      },
      '6': {
        dhParams: [
          { a: 0,  alpha: Math.PI / 2,  d: 15, thetaOffset: 0 },
          { a: 25, alpha: 0,            d: 0,  thetaOffset: 0 },
          { a: 0,  alpha: Math.PI / 2,  d: 0,  thetaOffset: 0 },
          { a: 0,  alpha: -Math.PI / 2, d: 20, thetaOffset: 0 },
          { a: 0,  alpha: Math.PI / 2,  d: 0,  thetaOffset: 0 },
          { a: 10, alpha: 0,            d: 0,  thetaOffset: 0 }
        ],
        limits: [
          { min: -170, max: 170 },
          { min: -90,  max: 90 },
          { min: -135, max: 135 },
          { min: -180, max: 180 },
          { min: -120, max: 120 },
          { min: -360, max: 360 }
        ],
        names: ["Base (J1)", "Épaule (J2)", "Coude (J3)", "Poignet R (J4)", "Poignet P (J5)", "Pince (J6)"],
        maxReach: 60
      }
    };

    this.currentDOF = '3';
    this.activeConfig = this.dofConfigs[this.currentDOF];

    this.validator = new UltraSafeValidator({
      jointLimits: this.activeConfig.limits,
      maxReach: this.activeConfig.maxReach,
      minGroundZ: 0.5
    });

    this.kinematics = new KinematicChain(this.activeConfig.dhParams);

    this.view = new Robot3DView(
      document.getElementById('canvas-container'),
      (targetPos) => this.onIKTargetDragged(targetPos)
    );

    this.waypointMgr = new WaypointManager(document.getElementById('waypoint-list'));

    this.initUI();
    this.switchDOF('3');
    this.animate();
  }

  initUI() {
    document.getElementById('dof-select').addEventListener('change', (e) => {
      this.switchDOF(e.target.value);
    });

    document.getElementById('btn-estop').addEventListener('click', () => {
      this.validator.triggerEStop();
      this.updateRobot();
    });

    document.querySelectorAll('.btn-speed').forEach(btn => {
      btn.addEventListener('click', (e) => {
        document.querySelectorAll('.btn-speed').forEach(b => b.classList.remove('active'));
        e.target.classList.add('active');
        const speed = parseFloat(e.target.getAttribute('data-speed'));
        this.setSpeed(speed);
      });
    });

    document.getElementById('btn-workspace').addEventListener('click', () => {
      const active = this.view.toggleWorkspaceCloud(this.activeConfig.dhParams, this.kinematics);
      document.getElementById('btn-workspace').style.background = active ? '#10b981' : '';
    });

    document.getElementById('btn-pickplace').addEventListener('click', () => {
      this.runPickAndPlaceSequence();
    });

    const gripperSlider = document.getElementById('gripper-slider');
    gripperSlider.addEventListener('input', () => {
      const val = gripperSlider.value;
      document.getElementById('gripper-val').textContent = `${val}%`;
      this.view.setGripperOpenness(val);
    });

    document.getElementById('btn-add-wp').addEventListener('click', () => {
      const angles = this.getCurrentAngles();
      const fk = this.kinematics.forwardKinematics(angles);
      this.waypointMgr.addWaypoint(angles, fk.endEffectorPos);
    });

    document.getElementById('btn-clear-wp').addEventListener('click', () => {
      this.waypointMgr.clear();
    });

    document.getElementById('btn-run-wp').addEventListener('click', () => {
      this.runWaypointTrajectory();
    });

    document.getElementById('btn-export-json').addEventListener('click', () => this.waypointMgr.exportJSON());
    document.getElementById('btn-export-gcode').addEventListener('click', () => this.waypointMgr.exportGCode());
    document.getElementById('btn-export-python').addEventListener('click', () => this.waypointMgr.exportPython());

    window.addEventListener('resize', () => {
      this.view.camera.aspect = window.innerWidth / window.innerHeight;
      this.view.camera.updateProjectionMatrix();
      this.view.renderer.setSize(window.innerWidth, window.innerHeight);
    });
  }

  setSpeed(factor) {
    this.speedFactor = Math.max(0.1, Math.min(3.0, factor));
  }

  switchDOF(dofKey) {
    this.currentDOF = dofKey;
    this.activeConfig = this.dofConfigs[dofKey];

    this.validator.updateConfig({
      jointLimits: this.activeConfig.limits,
      maxReach: this.activeConfig.maxReach,
      minGroundZ: 0.5
    });

    this.kinematics.setParams(this.activeConfig.dhParams);
    this.view.buildRobot(this.activeConfig.dhParams);

    const container = document.getElementById('sliders-container');
    container.innerHTML = '';

    this.sliders = [];
    this.valBadges = [];

    this.activeConfig.names.forEach((name, i) => {
      const limit = this.activeConfig.limits[i];
      const div = document.createElement('div');
      div.className = 'control-group slider-card';
      div.innerHTML = `
        <div class="slider-header">
          <label for="j${i}-slider">${name}</label>
          <span id="j${i}-val" class="val-badge">0°</span>
        </div>
        <div class="slider-container">
          <input type="range" id="j${i}-slider" min="${limit.min}" max="${limit.max}" value="0" step="1">
        </div>
      `;
      container.appendChild(div);

      const slider = div.querySelector('input');
      const badge = div.querySelector('.val-badge');

      slider.addEventListener('input', () => {
        badge.textContent = `${slider.value}°`;
        this.updateRobot(true);
      });

      this.sliders.push(slider);
      this.valBadges.push(badge);
    });

    this.updateRobot();
  }

  getCurrentAngles() {
    return this.sliders.map((s, idx) => {
      const val = parseFloat(s.value);
      return this.validator.clampAngle(val, idx);
    });
  }

  setAngles(angles, updateGizmo = true) {
    angles.forEach((a, i) => {
      if (this.sliders[i]) {
        const clamped = this.validator.clampAngle(a, i);
        this.sliders[i].value = clamped;
        this.valBadges[i].textContent = `${clamped.toFixed(0)}°`;
      }
    });
    this.updateRobot(updateGizmo);
  }

  updateRobot(updateGizmo = true) {
    const angles = this.getCurrentAngles();

    const fk = this.kinematics.forwardKinematics(angles);
    const validation = this.validator.validateState(angles, fk.endEffectorPos, this.view.obstacles);

    this.updateUIStatus(validation, fk.endEffectorPos);

    if (validation.safe) {
      this.view.updateTransforms(fk.transforms, updateGizmo);
    }
  }

  onIKTargetDragged(targetPos) {
    if (this.validator.isEStopped) return;

    const currentAngles = this.getCurrentAngles();
    const solvedAngles = this.kinematics.solveIK(targetPos, currentAngles);

    this.setAngles(solvedAngles, false);
  }

  updateUIStatus(validation, pos) {
    const alertBox = document.getElementById('safety-alert');
    const alertText = document.getElementById('safety-alert-text');
    
    alertBox.className = `alert alert-${validation.status}`;
    if (alertText) alertText.textContent = validation.message;

    document.getElementById('pos-x').innerHTML = `${pos.x.toFixed(1)} <small>cm</small>`;
    document.getElementById('pos-y').innerHTML = `${pos.y.toFixed(1)} <small>cm</small>`;
    document.getElementById('pos-z').innerHTML = `${pos.z.toFixed(1)} <small>cm</small>`;

    const dist = Math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2);
    const reachPct = Math.min(100, (dist / this.activeConfig.maxReach) * 100);
    document.getElementById('reach-val').innerHTML = `${reachPct.toFixed(0)} <small>%</small>`;
  }

  setGripperUI(percent) {
    document.getElementById('gripper-slider').value = percent;
    document.getElementById('gripper-val').textContent = `${percent}%`;
    this.view.setGripperOpenness(percent);
  }

  runPickAndPlaceSequence() {
    if (this.validator.isEStopped) this.validator.resetEStop();
    this.view.resetObjectPositions();

    const cubePos = this.view.targetCube.position.clone();
    const dropPos = new THREE.Vector3(-25, 3, 20);

    const startAngles = this.getCurrentAngles();
    const hoverCubeAngles = this.kinematics.solveIK(new THREE.Vector3(cubePos.x, cubePos.y + 10, cubePos.z), startAngles);
    const graspAngles = this.kinematics.solveIK(cubePos, hoverCubeAngles);
    const hoverDropAngles = this.kinematics.solveIK(new THREE.Vector3(dropPos.x, dropPos.y + 12, dropPos.z), graspAngles);
    const releaseAngles = this.kinematics.solveIK(dropPos, hoverDropAngles);

    this.setGripperUI(100);
    this.animateToAngles(hoverCubeAngles, 800, () => {
      this.animateToAngles(graspAngles, 600, () => {
        this.animateGripper(0, 400, () => {
          this.view.attachObjectToGripper(this.view.targetCube);
          this.animateToAngles(hoverDropAngles, 1000, () => {
            this.animateToAngles(releaseAngles, 600, () => {
              this.animateGripper(100, 400, () => {
                this.view.detachObject(this.view.scene);
                this.animateToAngles(startAngles, 800);
              });
            });
          });
        });
      });
    });
  }

  animateGripper(targetVal, baseDuration, onComplete) {
    const duration = baseDuration / this.speedFactor;
    const startVal = parseFloat(document.getElementById('gripper-slider').value);
    const startTime = performance.now();

    const step = (now) => {
      if (this.validator.isEStopped) return;
      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1.0);
      const current = startVal + (targetVal - startVal) * progress;
      this.setGripperUI(current);

      if (progress < 1.0) {
        requestAnimationFrame(step);
      } else if (onComplete) {
        onComplete();
      }
    };
    requestAnimationFrame(step);
  }

  runWaypointTrajectory() {
    if (this.waypointMgr.waypoints.length === 0 || this.validator.isEStopped) return;

    let wpIdx = 0;
    const executeWp = () => {
      if (wpIdx >= this.waypointMgr.waypoints.length || this.validator.isEStopped) return;
      const target = this.waypointMgr.waypoints[wpIdx].angles;
      this.animateToAngles(target, 1000, () => {
        wpIdx++;
        setTimeout(executeWp, 200 / this.speedFactor);
      });
    };
    executeWp();
  }

  animateToAngles(targetAngles, baseDuration, onComplete) {
    const duration = baseDuration / this.speedFactor;
    const startAngles = this.getCurrentAngles();
    const startTime = performance.now();

    const animStep = (now) => {
      if (this.validator.isEStopped) return;

      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1.0);
      const ease = progress * progress * progress * (progress * (progress * 6 - 15) + 10);

      const interpolated = startAngles.map((start, i) => {
        const target = targetAngles[i] !== undefined ? targetAngles[i] : start;
        return start + (target - start) * ease;
      });

      this.setAngles(interpolated, true);

      if (progress < 1.0) {
        requestAnimationFrame(animStep);
      } else if (onComplete) {
        onComplete();
      }
    };

    requestAnimationFrame(animStep);
  }

  animate() {
    requestAnimationFrame(() => this.animate());
    this.view.render();
  }
}

// Démarrage de l'application
window.addEventListener('DOMContentLoaded', () => {
  new RobotApp();
});