import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';

const SAFETY = Object.freeze({
  MIN_GROUND_Z: 0.5,
  MAX_JOINT_VELOCITY_DEG_PER_SEC: 120,
  MAX_REACH_MARGIN: 1.05,
  IK_MAX_ITER: 30,
  IK_TOLERANCE: 0.1,
  IK_DAMPING: 0.5,
  WORKSPACE_SAMPLES: 3500,
  MAX_WAYPOINTS: 100,
  MAX_IMPORTED_MODELS: 10,
  MAX_SPEED_FACTOR: 3.0,
  MIN_SPEED_FACTOR: 0.1,
  ANIMATION_TIMEOUT_MS: 10000,
  SCENE_UNIT_TO_MM: 10,
  LOCALSTORAGE_KEY: 'robocontrol_waypoints_v1',
});


const SafeLogger = {
  log(level, message, data) {
    if (typeof console === 'undefined') return;
    const ts = new Date().toISOString();
    const prefix = `[${ts}] [${level}]`;
    if (data !== undefined) console[level === 'error' ? 'error' : level === 'warn' ? 'warn' : 'log'](prefix, message, data);
    else console[level === 'error' ? 'error' : level === 'warn' ? 'warn' : 'log'](prefix, message);
  },
  info(m, d) { this.log('info', m, d); },
  warn(m, d) { this.log('warn', m, d); },
  error(m, d) { this.log('error', m, d); },
};



class UltraSafeValidator {
  constructor(config) {
    this.updateConfig(config);
    this.isEStopped = false;
    this.lastValidationTime = 0;
    this._lastRejectedMessage = '';
    this._lastRejectedTime = 0;
  }

  updateConfig(config) {
    if (!config || !Array.isArray(config.jointLimits)) {
      throw new Error('Configuration invalide : jointLimits manquant');
    }
    this.jointLimits = config.jointLimits.map(l => ({
      min: Number(l.min),
      max: Number(l.max),
    }));
    this.maxReach = Math.max(0, Number(config.maxReach) || 0);
    this.minGroundZ = Number(config.minGroundZ) || 0;
  }

  triggerEStop() {
    this.isEStopped = true;
    SafeLogger.warn('E-STOP déclenché');
  }

  resetEStop() {
    this.isEStopped = false;
    SafeLogger.info('E-STOP réinitialisé');
  }

  _isFiniteVec3(v) {
    return v && Number.isFinite(v.x) && Number.isFinite(v.y) && Number.isFinite(v.z);
  }

  _reject(status, message) {
    const now = performance.now();
    if (message === this._lastRejectedMessage && now - this._lastRejectedTime < 250) {
      return { safe: false, status, message };
    }
    this._lastRejectedMessage = message;
    this._lastRejectedTime = now;
    if (status === 'danger') SafeLogger.warn(`[Validator] ${message}`);
    return { safe: false, status, message };
  }

  validateState(jointAnglesDeg, endEffectorPos, obstacles = []) {
    if (this.isEStopped) {
      return this._reject('danger', "🚨 ARRÊT D'URGENCE ACTIF !");
    }

    if (!Array.isArray(jointAnglesDeg) || jointAnglesDeg.length === 0) {
      return this._reject('danger', 'Erreur : tableau d\'angles invalide.');
    }
    for (let i = 0; i < jointAnglesDeg.length; i++) {
      if (!Number.isFinite(jointAnglesDeg[i])) {
        return this._reject('danger', 'Erreur : Données angulaires NaN/Infini.');
      }
    }

    if (!this._isFiniteVec3(endEffectorPos)) {
      return this._reject('danger', 'Erreur : position effecteur invalide.');
    }

    const TOLERANCE_DEG = 0.5;
    for (let i = 0; i < jointAnglesDeg.length; i++) {
      const limit = this.jointLimits[i];
      if (!limit) continue;
      if (jointAnglesDeg[i] < limit.min - TOLERANCE_DEG || jointAnglesDeg[i] > limit.max + TOLERANCE_DEG) {
        return this._reject(
          'warning',
          `Butée Joint ${i + 1} (${jointAnglesDeg[i].toFixed(1)}° hors [${limit.min}°, ${limit.max}°])`
        );
      }
    }

    if (endEffectorPos.z < this.minGroundZ) {
      return this._reject(
        'danger',
        `Risque collision sol (Z: ${endEffectorPos.z.toFixed(2)} < ${this.minGroundZ})`
      );
    }

    const SAFETY_MARGIN = 0.5;
    for (const obs of obstacles) {
      if (obs && obs.box && obs.box.containsPoint(endEffectorPos)) {
        return this._reject('danger', 'Collision détectée avec un obstacle !');
      }
      if (obs && obs.box) {
        const closest = obs.box.clampPoint(endEffectorPos, new THREE.Vector3());
        if (closest.distanceTo(endEffectorPos) < SAFETY_MARGIN) {
          return this._reject('warning', 'Proximité obstacle (marge < 0.5 cm)');
        }
      }
    }

    const dist = Math.sqrt(
      endEffectorPos.x ** 2 + endEffectorPos.y ** 2 + endEffectorPos.z ** 2
    );
    if (dist > this.maxReach) {
      return this._reject(
        'warning',
        `Hors de portée (${dist.toFixed(1)} cm > max ${this.maxReach} cm)`
      );
    }

    return { safe: true, status: 'success', message: 'Système nominal : Aucune anomalie' };
  }

  clampAngle(angleDeg, jointIndex) {
    if (!Number.isFinite(angleDeg)) return 0;
    const limit = this.jointLimits[jointIndex];
    if (!limit) return angleDeg;
    return Math.max(limit.min, Math.min(limit.max, angleDeg));
  }

  isVelocitySafe(fromAngles, toAngles, durationMs) {
    if (!durationMs || durationMs <= 0) return false;
    if (fromAngles.length !== toAngles.length) return false;
    const dt = durationMs / 1000;
    for (let i = 0; i < fromAngles.length; i++) {
      const delta = Math.abs(toAngles[i] - fromAngles[i]);
      const vel = delta / dt;
      if (vel > SAFETY.MAX_JOINT_VELOCITY_DEG_PER_SEC) {
        SafeLogger.warn(`Vitesse joint ${i + 1} trop élevée : ${vel.toFixed(1)}°/s`);
        return false;
      }
    }
    return true;
  }
}



class DHLink {
  constructor(a, alpha, d, thetaOffset = 0) {
    this.a = a;
    this.alpha = alpha;
    this.d = d;
    this.thetaOffset = thetaOffset;
  }

  getMatrix(thetaRad, target) {
    const totalTheta = thetaRad + this.thetaOffset;
    const ct = Math.cos(totalTheta), st = Math.sin(totalTheta);
    const ca = Math.cos(this.alpha), sa = Math.sin(this.alpha);

    const m = target || new THREE.Matrix4();
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
    // ── FIX #15 : pool de matrices réutilisables (perf) ──
    this._tmpMatrix = new THREE.Matrix4();
    this._tmpVec = new THREE.Vector3();
    this._tmpQuat = new THREE.Quaternion();
    this._tmpScale = new THREE.Vector3();
    this.setParams(dhParams);
  }

  setParams(dhParams) {
    if (!Array.isArray(dhParams) || dhParams.length === 0) {
      throw new Error('DH params invalides');
    }
    this.links = dhParams.map(p => new DHLink(p.a, p.alpha, p.d, p.thetaOffset));
  }

  forwardKinematics(jointAnglesDeg) {
    let currentMatrix = this._tmpMatrix.identity();
    const transforms = [currentMatrix.clone()];

    for (let i = 0; i < this.links.length; i++) {
      const deg = Number(jointAnglesDeg[i]);
      if (!Number.isFinite(deg)) {
        SafeLogger.error(`Angle joint ${i + 1} invalide : ${jointAnglesDeg[i]}`);
        return { endEffectorPos: new THREE.Vector3(0, 0, 0), transforms, invalid: true };
      }
      const rad = (deg * Math.PI) / 180;
      const linkMatrix = this.links[i].getMatrix(rad, new THREE.Matrix4());
      currentMatrix.multiply(linkMatrix);
      transforms.push(currentMatrix.clone());
    }

    const pos = new THREE.Vector3();
    const quat = this._tmpQuat;
    const scale = this._tmpScale;
    currentMatrix.decompose(pos, quat, scale);

    return { endEffectorPos: pos, transforms };
  }

  // ── FIX #4 : IK bornée + clamp articulaire + validation NaN ──
  solveIK(targetPos, currentAnglesDeg, limits, maxIter = SAFETY.IK_MAX_ITER, damping = SAFETY.IK_DAMPING) {
    if (!targetPos || !Number.isFinite(targetPos.x)) {
      SafeLogger.warn('IK : cible invalide');
      return [...currentAnglesDeg];
    }

    const clamp = (v, i) => {
      if (!limits || !limits[i]) return v;
      return Math.max(limits[i].min, Math.min(limits[i].max, v));
    };

    let angles = currentAnglesDeg.map((a, i) => clamp(a, i));
    const deltaDeg = 0.1;

    for (let iter = 0; iter < maxIter; iter++) {
      const currentFK = this.forwardKinematics(angles);
      if (currentFK.invalid) return [...currentAnglesDeg];

      const error = new THREE.Vector3().subVectors(targetPos, currentFK.endEffectorPos);
      if (!Number.isFinite(error.length())) return [...currentAnglesDeg];
      if (error.length() < SAFETY.IK_TOLERANCE) break;

      const J = [];
      for (let j = 0; j < angles.length; j++) {
        angles[j] += deltaDeg;
        const fkPlus = this.forwardKinematics(angles);
        angles[j] -= deltaDeg;
        const diff = new THREE.Vector3()
          .subVectors(fkPlus.endEffectorPos, currentFK.endEffectorPos)
          .divideScalar(deltaDeg);
        J.push(diff);
      }

      for (let j = 0; j < angles.length; j++) {
        const dot = J[j].dot(error);
        if (Number.isFinite(dot)) {
          angles[j] = clamp(angles[j] + dot * damping, j);
        }
      }

      if (angles.some(a => !Number.isFinite(a))) {
        SafeLogger.warn('IK divergente — retour aux angles initiaux');
        return [...currentAnglesDeg];
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
    this._resizeHandler = null;
    this._resizeObserver = null;
    this._rafId = null;
    this._paused = false;
    // ── FIX #3 : flag anti-boucle TransformControls ──
    this._updatingGizmoProgrammatically = false;
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
    texture.colorSpace = THREE.SRGBColorSpace;
    return texture;
  }

  initScene() {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0f172a);
    this.scene.environment = this.createHDRProceduralEnvironment();

    this.camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
    this.camera.position.set(80, 70, 110);

    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      powerPreference: 'high-performance',
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(window.innerWidth, window.innerHeight);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.1;
    this.container.appendChild(this.renderer.domElement);

    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.set(0, 20, 0);
    this.orbit.enableDamping = true;
    this.orbit.dampingFactor = 0.08;
    this.orbit.update();

    const ambientLight = new THREE.AmbientLight(0xffffff, 0.7);
    this.scene.add(ambientLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 2.4);
    dirLight.position.set(60, 120, 60);
    dirLight.castShadow = true;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    dirLight.shadow.bias = -0.0001;
    this.scene.add(dirLight);

    const spotLight = new THREE.SpotLight(0x38bdf8, 3.0, 150, Math.PI / 4, 0.5);
    spotLight.position.set(-50, 80, -50);
    this.scene.add(spotLight);

    const grid = new THREE.GridHelper(200, 40, 0x38bdf8, 0x334155);
    grid.position.y = 0.01;
    this.scene.add(grid);

    const floorGeo = new THREE.PlaneGeometry(200, 200);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0b1329,
      roughness: 0.4,
      metalness: 0.6,
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

    this.transformControls = new TransformControls(this.camera, this.renderer.domElement);
    this.transformControls.attach(this.targetMesh);
    this.transformControls.setSize(0.75);
    this.scene.add(this.transformControls.getHelper());

    this.transformControls.addEventListener('dragging-changed', (event) => {
      this.orbit.enabled = !event.value;
    });

    // ── FIX #3 : ignore les updates programmatiques ──
    this.transformControls.addEventListener('change', () => {
      if (!this.transformControls.dragging) return;
      if (this._updatingGizmoProgrammatically) return;
      this.onTargetMove(this.targetMesh.position);
    });

    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    // ── FIX #18 : ResizeObserver + fallback window ──
    this._resizeHandler = () => {
      if (!this.camera || !this.renderer) return;
      const w = this.container.clientWidth || window.innerWidth;
      const h = this.container.clientHeight || window.innerHeight;
      this.camera.aspect = w / h;
      this.camera.updateProjectionMatrix();
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.setSize(w, h);
    };
    window.addEventListener('resize', this._resizeHandler);

    if (typeof ResizeObserver !== 'undefined') {
      this._resizeObserver = new ResizeObserver(this._resizeHandler);
      this._resizeObserver.observe(this.container);
    }
  }

  initEnvironmentObjects() {
    const tableGeo = new THREE.BoxGeometry(40, 12, 30);
    const tableMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3, metalness: 0.7 });
    this.tableMesh = new THREE.Mesh(tableGeo, tableMat);
    this.tableMesh.position.set(25, 6, 15);
    this.tableMesh.castShadow = true;
    this.tableMesh.receiveShadow = true;
    this.scene.add(this.tableMesh);

    this.obstacles.push({
      mesh: this.tableMesh,
      box: new THREE.Box3().setFromObject(this.tableMesh),
    });

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

  // ── FIX #11 : validation URL stricte ──
  async loadGLTFModel(url, position = new THREE.Vector3(0, 0, 0), scale = new THREE.Vector3(1, 1, 1)) {
    if (typeof url !== 'string' || url.length === 0) {
      SafeLogger.warn('loadGLTFModel : URL invalide');
      return;
    }
    try {
      const parsed = new URL(url, window.location.href);
      if (!['http:', 'https:'].includes(parsed.protocol)) {
        SafeLogger.warn('loadGLTFModel : protocole refusé', parsed.protocol);
        return;
      }
    } catch {
      SafeLogger.warn('loadGLTFModel : URL malformée', url);
      return;
    }
    if (this.importedModels.length >= SAFETY.MAX_IMPORTED_MODELS) {
      SafeLogger.warn(`Limite de ${SAFETY.MAX_IMPORTED_MODELS} modèles importés atteinte`);
      return;
    }
    try {
      const { GLTFLoader } = await import('three/addons/loaders/GLTFLoader.js');
      const loader = new GLTFLoader();
      loader.load(
        url,
        (gltf) => {
          const model = gltf.scene;
          model.position.copy(position);
          model.scale.copy(scale);
          this.scene.add(model);
          this.importedModels.push(model);
          const box = new THREE.Box3().setFromObject(model);
          this.obstacles.push({ mesh: model, box });
        },
        undefined,
        (err) => {
          SafeLogger.error('Échec chargement GLTF', err);
          const alertText = document.getElementById('safety-alert-text');
          if (alertText) alertText.textContent = 'Erreur lors du chargement du modèle 3D';
        }
      );
    } catch (e) {
      SafeLogger.error('GLTFLoader indisponible', e);
    }
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
    const clamped = Math.max(0, Math.min(100, Number(percent) || 0));
    const offset = (clamped / 100) * 1.8 + 0.4;
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
    const samples = Math.min(SAFETY.WORKSPACE_SAMPLES, 10000);

    for (let i = 0; i < samples; i++) {
      const randAngles = dhParams.map(() => (Math.random() - 0.5) * 240);
      const fk = kinematics.forwardKinematics(randAngles);
      if (fk.invalid) continue;
      const p = fk.endEffectorPos;
      if (Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.z) && p.z >= 0) {
        points.push(p.x, p.y, p.z);
      }
    }

    const geo = new THREE.BufferGeometry();
    geo.setAttribute('position', new THREE.Float32BufferAttribute(points, 3));
    const mat = new THREE.PointsMaterial({
      color: 0x38bdf8,
      size: 0.8,
      transparent: true,
      opacity: 0.35,
    });

    this.workspaceMesh = new THREE.Points(geo, mat);
    this.scene.add(this.workspaceMesh);
    return true;
  }

  updateTransforms(transforms, updateGizmo = true) {
    if (!transforms || transforms.length === 0) return;
    for (let i = 0; i < this.visualLinks.length; i++) {
      if (transforms[i + 1]) {
        this.visualLinks[i].matrixAutoUpdate = false;
        this.visualLinks[i].matrix.copy(transforms[i + 1]);
      }
    }
    if (updateGizmo && transforms.length > 0) {
      const lastMat = transforms[transforms.length - 1];
      const pos = new THREE.Vector3();
      lastMat.decompose(pos, new THREE.Quaternion(), new THREE.Vector3());
      if (Number.isFinite(pos.x)) {
        // ── FIX #3 : protège contre la boucle IK ↔ gizmo ──
        this._updatingGizmoProgrammatically = true;
        this.targetMesh.position.copy(pos);
        this._updatingGizmoProgrammatically = false;
      }
    }
  }

  render() {
    this.orbit.update();
    this.renderer.render(this.scene, this.camera);
  }

  dispose() {
    if (this._resizeHandler) window.removeEventListener('resize', this._resizeHandler);
    if (this._resizeObserver) this._resizeObserver.disconnect();
    if (this.renderer) this.renderer.dispose();
  }
}



class WaypointManager {
  constructor(containerEl, onUpdate) {
    this.container = containerEl;
    this.onUpdate = onUpdate;
    this.waypoints = [];
    this.loadFromLocalStorage();
  }

  addWaypoint(angles, pos) {
    if (this.waypoints.length >= SAFETY.MAX_WAYPOINTS) {
      SafeLogger.warn(`Limite de ${SAFETY.MAX_WAYPOINTS} waypoints atteinte`);
      return false;
    }
    if (!Array.isArray(angles) || angles.some(a => !Number.isFinite(a))) {
      SafeLogger.warn('addWaypoint : angles invalides');
      return false;
    }
    if (!pos || !Number.isFinite(pos.x) || !Number.isFinite(pos.y) || !Number.isFinite(pos.z)) {
      SafeLogger.warn('addWaypoint : position invalide');
      return false;
    }
    this.waypoints.push({
      id: Date.now() + Math.random(),
      angles: [...angles],
      pos: { x: pos.x, y: pos.y, z: pos.z },
    });
    this.saveToLocalStorage();
    this.render();
    if (this.onUpdate) this.onUpdate();
    return true;
  }

  // ── FIX #9 : suppression individuelle ──
  removeWaypoint(id) {
    this.waypoints = this.waypoints.filter(w => w.id !== id);
    this.saveToLocalStorage();
    this.render();
    if (this.onUpdate) this.onUpdate();
  }

  clear() {
    this.waypoints = [];
    this.saveToLocalStorage();
    this.render();
    if (this.onUpdate) this.onUpdate();
  }

  // ── FIX #10 : persistance localStorage ──
  saveToLocalStorage() {
    try {
      localStorage.setItem(SAFETY.LOCALSTORAGE_KEY, JSON.stringify(this.waypoints));
    } catch (e) {
      SafeLogger.warn('Échec sauvegarde localStorage', e);
    }
  }

  loadFromLocalStorage() {
    try {
      const raw = localStorage.getItem(SAFETY.LOCALSTORAGE_KEY);
      if (!raw) return;
      const parsed = JSON.parse(raw);
      if (!Array.isArray(parsed)) return;
      this.waypoints = parsed.filter(w =>
        w && Array.isArray(w.angles) &&
        w.angles.every(Number.isFinite) &&
        w.pos && Number.isFinite(w.pos.x) && Number.isFinite(w.pos.y) && Number.isFinite(w.pos.z)
      );
    } catch (e) {
      SafeLogger.warn('Échec chargement localStorage', e);
      this.waypoints = [];
    }
  }

  _esc(s) {
    return String(s).replace(/[&<>"']/g, c => ({
      '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;'
    }[c]));
  }

  render() {
    if (this.waypoints.length === 0) {
      this.container.innerHTML = `<div class="empty-state">Aucun point d'étape enregistré</div>`;
      return;
    }
    this.container.innerHTML = this.waypoints.map((wp, i) => `
      <div class="waypoint-item" data-id="${this._esc(wp.id)}">
        <span>#${i + 1} [X:${this._esc(wp.pos.x.toFixed(0))}, Y:${this._esc(wp.pos.y.toFixed(0))}, Z:${this._esc(wp.pos.z.toFixed(0))}]</span>
        <span style="color: var(--accent);">${this._esc(wp.angles.map(a => Math.round(a) + '°').join(', '))}</span>
        <button type="button" class="btn-wp-remove" aria-label="Supprimer le waypoint ${i + 1}">✕</button>
      </div>
    `).join('');

    // ── FIX #9 : bind suppression ──
    this.container.querySelectorAll('.btn-wp-remove').forEach(btn => {
      btn.addEventListener('click', (e) => {
        const id = parseFloat(e.currentTarget.parentElement.dataset.id);
        this.removeWaypoint(id);
      });
    });
  }

  exportJSON() {
    const dataStr = 'data:text/json;charset=utf-8,' + encodeURIComponent(JSON.stringify(this.waypoints, null, 2));
    this.downloadFile(dataStr, 'trajectory_waypoints.json');
  }

  exportGCode() {
    let gcode = '; G-Code Généré par Simulateur Robotique\n';
    gcode += `; Unités scène: cm → mm (×${SAFETY.SCENE_UNIT_TO_MM})\n`;
    gcode += 'G21 ; Unités en mm\nG90 ; Positionnement absolu\nG28 ; Homing\n\n';
    this.waypoints.forEach((wp, i) => {
      const x = wp.pos.x * SAFETY.SCENE_UNIT_TO_MM;
      const y = wp.pos.y * SAFETY.SCENE_UNIT_TO_MM;
      const z = wp.pos.z * SAFETY.SCENE_UNIT_TO_MM;
      gcode += `; Waypoint ${i + 1}\nG01 X${x.toFixed(2)} Y${y.toFixed(2)} Z${z.toFixed(2)} F1500\n`;
    });
    const dataStr = 'data:text/plain;charset=utf-8,' + encodeURIComponent(gcode);
    this.downloadFile(dataStr, 'robot_trajectory.gcode');
  }

  // ── FIX #8 : export Python avec limites articulaires ──
  exportPython(jointLimits) {
    const limits = jointLimits || [];
    let py = '# Script Python de Trajectoire Robotique (ROS / PyBullet)\n';
    py += '# Généré par RoboControl 3D\n';
    py += 'import time\n\n';
    py += 'JOINT_LIMITS = ' + JSON.stringify(limits.map(l => [l.min, l.max]), null, 2) + '\n\n';
    py += 'waypoints = ' + JSON.stringify(this.waypoints.map(w => w.angles), null, 2) + '\n\n';
    py += `def safe_set_joint_angles(robot, angles):
    """Applique les angles avec vérification des butées."""
    if len(angles) != len(JOINT_LIMITS):
        raise ValueError(f"Attendu {len(JOINT_LIMITS)} joints, reçu {len(angles)}")
    for i, (a, (lo, hi)) in enumerate(zip(angles, JOINT_LIMITS)):
        if not (lo <= a <= hi):
            raise ValueError(f"Joint {i+1} hors limites: {a}° hors [{lo}°, {hi}°]")
    robot.set_joint_angles(angles)


def execute_trajectory(robot):
    for angles in waypoints:
        safe_set_joint_angles(robot, angles)
        time.sleep(0.5)


if __name__ == "__main__":
    # robot = MyRobot()  # remplacer par votre API
    # execute_trajectory(robot)
    pass
`;
    const dataStr = 'data:text/plain;charset=utf-8,' + encodeURIComponent(py);
    this.downloadFile(dataStr, 'robot_trajectory.py');
  }

  exportURDF(dhParams, jointLimits) {
    const links = dhParams.map((_, i) => `<link name="link_${i + 1}"/>`).join('\n  ');
    const joints = dhParams.map((dh, i) => {
      const limit = jointLimits[i];
      return `  <joint name="joint_${i + 1}" type="revolute">\n` +
        `    <parent link="${i === 0 ? 'base_link' : `link_${i}_dh`}"/>\n` +
        `    <child link="link_${i + 1}"/>\n` +
        `    <origin xyz="0 0 ${(dh.d / 100).toFixed(6)}" rpy="0 0 ${dh.thetaOffset.toFixed(6)}"/>\n` +
        `    <axis xyz="0 0 1"/>\n` +
        `    <limit lower="${(limit.min * Math.PI / 180).toFixed(6)}" upper="${(limit.max * Math.PI / 180).toFixed(6)}" effort="1" velocity="2"/>\n` +
        `  </joint>\n` +
        `  <joint name="link_transform_${i + 1}" type="fixed">\n` +
        `    <parent link="link_${i + 1}"/><child link="link_${i + 1}_dh"/>\n` +
        `    <origin xyz="${(dh.a / 100).toFixed(6)} 0 0" rpy="${dh.alpha.toFixed(6)} 0 0"/>\n` +
        `  </joint>`;
    }).join('\n');
    const dhLinks = dhParams.map((_, i) => `<link name="link_${i + 1}_dh"/>`).join('\n  ');
    const urdf = `<?xml version="1.0"?>\n<robot name="robocontrol_arm">\n` +
      `  <link name="base_link"/>\n  ${links}\n  ${dhLinks}\n${joints}\n</robot>\n`;
    this.downloadFile('data:application/xml;charset=utf-8,' + encodeURIComponent(urdf), 'robot.urdf');
  }

  downloadFile(dataStr, filename) {
    const a = document.createElement('a');
    a.setAttribute('href', dataStr);
    a.setAttribute('download', filename);
    document.body.appendChild(a);
    a.click();
    a.remove();
  }
}



class RobotApp {
  constructor() {
    this.speedFactor = 1.0;
    this._animationToken = 0;
    // ── FIX #1 : initialisation défensive ──
    this.sliders = [];
    this.valBadges = [];
    this._paused = false;

    this.dofConfigs = {
      '3': {
        dhParams: [
          { a: 0,  alpha: Math.PI / 2, d: 15, thetaOffset: 0 },
          { a: 30, alpha: 0,           d: 0,  thetaOffset: 0 },
          { a: 25, alpha: 0,           d: 0,  thetaOffset: 0 },
        ],
        limits: [
          { min: -150, max: 150 },
          { min: -60,  max: 90 },
          { min: -120, max: 60 },
        ],
        names: ['Base (J1)', 'Épaule (J2)', 'Coude (J3)'],
        maxReach: 55,
      },
      '6': {
        dhParams: [
          { a: 0,  alpha: Math.PI / 2,  d: 15, thetaOffset: 0 },
          { a: 25, alpha: 0,            d: 0,  thetaOffset: 0 },
          { a: 0,  alpha: Math.PI / 2,  d: 0,  thetaOffset: 0 },
          { a: 0,  alpha: -Math.PI / 2, d: 20, thetaOffset: 0 },
          { a: 0,  alpha: Math.PI / 2,  d: 0,  thetaOffset: 0 },
          { a: 10, alpha: 0,            d: 0,  thetaOffset: 0 },
        ],
        limits: [
          { min: -170, max: 170 },
          { min: -90,  max: 90 },
          { min: -135, max: 135 },
          { min: -180, max: 180 },
          { min: -120, max: 120 },
          { min: -360, max: 360 },
        ],
        names: ['Base (J1)', 'Épaule (J2)', 'Coude (J3)', 'Poignet R (J4)', 'Poignet P (J5)', 'Pince (J6)'],
        maxReach: 60,
      },
    };

    this.currentDOF = '3';
    this.activeConfig = this.dofConfigs[this.currentDOF];

    this.validator = new UltraSafeValidator({
      jointLimits: this.activeConfig.limits,
      maxReach: this.activeConfig.maxReach,
      minGroundZ: SAFETY.MIN_GROUND_Z,
    });

    this.kinematics = new KinematicChain(this.activeConfig.dhParams);

    const container = document.getElementById('canvas-container');
    if (!container) throw new Error('#canvas-container introuvable');

    this.view = new Robot3DView(container, (targetPos) => this.onIKTargetDragged(targetPos));
    this.waypointMgr = new WaypointManager(document.getElementById('waypoint-list'));
    this.waypointMgr.render();

    this.initUI();
    this.switchDOF('3');
    this.startLoop();

    // ── FIX #19 : pause quand onglet caché ──
    document.addEventListener('visibilitychange', () => {
      this._paused = document.hidden;
    });

    document.getElementById('loader')?.classList.add('hidden');
  }

  initUI() {
    document.getElementById('dof-select')?.addEventListener('change', (e) => {
      this.switchDOF(e.target.value);
    });

    // ── FIX #2 : E-STOP toggle (trigger ↔ reset) ──
    document.getElementById('btn-estop')?.addEventListener('click', () => {
      if (this.validator.isEStopped) {
        this.validator.resetEStop();
      } else {
        this.validator.triggerEStop();
      }
      this._applyEStopToSliders();
      this.updateRobot();
      this._updateEStopUI();
    });

    // ── FIX #22 : ne pas voler Space/Escape aux inputs ──
    window.addEventListener('keydown', (e) => {
      const tag = document.activeElement?.tagName;
      const isFormControl = tag === 'INPUT' || tag === 'SELECT' || tag === 'TEXTAREA';

      if (e.key === 'Escape') {
        e.preventDefault();
        if (!this.validator.isEStopped) {
          this.validator.triggerEStop();
          this._applyEStopToSliders();
          this.updateRobot();
          this._updateEStopUI();
        }
        return;
      }

      if (e.code === 'Space' && !isFormControl) {
        e.preventDefault();
        if (!this.validator.isEStopped) {
          this.validator.triggerEStop();
          this._applyEStopToSliders();
          this.updateRobot();
          this._updateEStopUI();
        }
        return;
      }

      if ((e.key === 'r' || e.key === 'R') && !isFormControl) {
        this.validator.resetEStop();
        this._applyEStopToSliders();
        this._updateEStopUI();
      }
    });

    document.querySelectorAll('.btn-speed').forEach(btn => {
      btn.addEventListener('click', (e) => {
        document.querySelectorAll('.btn-speed').forEach(b => {
          b.classList.remove('active');
          b.setAttribute('aria-pressed', 'false');
        });
        e.currentTarget.classList.add('active');
        e.currentTarget.setAttribute('aria-pressed', 'true');
        const speed = parseFloat(e.currentTarget.getAttribute('data-speed'));
        this.setSpeed(speed);
      });
    });

    document.getElementById('btn-workspace')?.addEventListener('click', (e) => {
      const active = this.view.toggleWorkspaceCloud(this.activeConfig.dhParams, this.kinematics);
      e.currentTarget.style.background = active ? '#10b981' : '';
    });

    document.getElementById('btn-pickplace')?.addEventListener('click', () => {
      this.runPickAndPlaceSequence();
    });

    const gripperSlider = document.getElementById('gripper-slider');
    gripperSlider?.addEventListener('input', () => {
      const val = parseInt(gripperSlider.value, 10) || 0;
      document.getElementById('gripper-val').textContent = `${val}%`;
      this.view.setGripperOpenness(val);
    });

    document.getElementById('btn-add-wp')?.addEventListener('click', () => {
      const angles = this.getCurrentAngles();
      const fk = this.kinematics.forwardKinematics(angles);
      if (fk.invalid) return;
      this.waypointMgr.addWaypoint(angles, fk.endEffectorPos);
    });

    document.getElementById('btn-clear-wp')?.addEventListener('click', () => {
      this.waypointMgr.clear();
    });

    document.getElementById('btn-run-wp')?.addEventListener('click', () => {
      this.runWaypointTrajectory();
    });

    document.getElementById('btn-export-json')?.addEventListener('click', () => this.waypointMgr.exportJSON());
    document.getElementById('btn-export-gcode')?.addEventListener('click', () => this.waypointMgr.exportGCode());
    document.getElementById('btn-export-python')?.addEventListener('click', () => {
      this.waypointMgr.exportPython(this.activeConfig.limits);
    });
    document.getElementById('btn-export-urdf')?.addEventListener('click', () => {
      this.waypointMgr.exportURDF(this.activeConfig.dhParams, this.activeConfig.limits);
    });
  }

  _applyEStopToSliders() {
    const stopped = this.validator.isEStopped;
    this.sliders.forEach(s => { s.disabled = stopped; });
  }

  _updateEStopUI() {
    const btn = document.getElementById('btn-estop');
    if (!btn) return;
    if (this.validator.isEStopped) {
      btn.textContent = '🔄 RÉARMER (R)';
      btn.classList.remove('btn-danger');
      btn.classList.add('btn-warning');
      btn.setAttribute('aria-label', "Arrêt d'urgence actif — cliquer pour réarmer");
    } else {
      btn.textContent = '⚡ E-STOP';
      btn.classList.remove('btn-warning');
      btn.classList.add('btn-danger');
      btn.setAttribute('aria-label', "Arrêt d'urgence");
    }
  }

  setSpeed(factor) {
    this.speedFactor = Math.max(SAFETY.MIN_SPEED_FACTOR, Math.min(SAFETY.MAX_SPEED_FACTOR, factor));
  }

  switchDOF(dofKey) {
    if (!this.dofConfigs[dofKey]) {
      SafeLogger.warn(`DOF inconnu : ${dofKey}`);
      return;
    }
    this.currentDOF = dofKey;
    this.activeConfig = this.dofConfigs[dofKey];

    this.validator.updateConfig({
      jointLimits: this.activeConfig.limits,
      maxReach: this.activeConfig.maxReach,
      minGroundZ: SAFETY.MIN_GROUND_Z,
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
      // ── FIX #20 : aria-valuetext pour lecteurs d'écran ──
      slider.setAttribute('aria-valuetext', `0 degrés`);
      slider.setAttribute('aria-label', name);

      slider.addEventListener('input', () => {
        if (this.validator.isEStopped) {
          slider.value = this.validator.clampAngle(parseFloat(slider.value), i);
          return;
        }
        badge.textContent = `${slider.value}°`;
        slider.setAttribute('aria-valuetext', `${slider.value} degrés`);
        this.updateRobot(true);
      });

      // ── FIX #6 : sliders désactivés en E-STOP ──
      if (this.validator.isEStopped) slider.disabled = true;

      this.sliders.push(slider);
      this.valBadges.push(badge);
    });

    this.updateRobot();
  }

  getCurrentAngles() {
    return this.sliders.map((s, idx) => this.validator.clampAngle(parseFloat(s.value), idx));
  }

  setAngles(angles, updateGizmo = true) {
    angles.forEach((a, i) => {
      if (this.sliders[i]) {
        const clamped = this.validator.clampAngle(a, i);
        this.sliders[i].value = clamped;
        this.sliders[i].setAttribute('aria-valuetext', `${clamped.toFixed(0)} degrés`);
        if (this.valBadges[i]) this.valBadges[i].textContent = `${clamped.toFixed(0)}°`;
      }
    });
    this.updateRobot(updateGizmo);
  }

  updateRobot(updateGizmo = true) {
    const angles = this.getCurrentAngles();
    const fk = this.kinematics.forwardKinematics(angles);
    if (fk.invalid) return;
    const validation = this.validator.validateState(angles, fk.endEffectorPos, this.view.obstacles);
    this.updateUIStatus(validation, fk.endEffectorPos);
    if (validation.safe) {
      this.view.updateTransforms(fk.transforms, updateGizmo);
    }
  }

  onIKTargetDragged(targetPos) {
    if (this.validator.isEStopped) return;
    const currentAngles = this.getCurrentAngles();
    // ── FIX #4 : passe les limites à solveIK ──
    const solvedAngles = this.kinematics.solveIK(targetPos, currentAngles, this.activeConfig.limits);
    const fk = this.kinematics.forwardKinematics(solvedAngles);
    if (fk.invalid) return;
    const v = this.validator.validateState(solvedAngles, fk.endEffectorPos, this.view.obstacles);
    if (!v.safe && v.status === 'danger') return;
    this.setAngles(solvedAngles, false);
  }

  updateUIStatus(validation, pos) {
    const alertBox = document.getElementById('safety-alert');
    const alertText = document.getElementById('safety-alert-text');
    if (alertBox) alertBox.className = `alert alert-${validation.status}`;
    if (alertText) alertText.textContent = validation.message;

    const px = document.getElementById('pos-x');
    const py = document.getElementById('pos-y');
    const pz = document.getElementById('pos-z');
    if (px) px.innerHTML = `${pos.x.toFixed(1)} <small>cm</small>`;
    if (py) py.innerHTML = `${pos.y.toFixed(1)} <small>cm</small>`;
    if (pz) pz.innerHTML = `${pos.z.toFixed(1)} <small>cm</small>`;

    const dist = Math.sqrt(pos.x ** 2 + pos.y ** 2 + pos.z ** 2);
    const reachPct = Math.min(100, (dist / this.activeConfig.maxReach) * 100);
    const reachEl = document.getElementById('reach-val');
    if (reachEl) reachEl.innerHTML = `${reachPct.toFixed(0)} <small>%</small>`;
  }

  setGripperUI(percent) {
    const clamped = Math.max(0, Math.min(100, percent));
    const slider = document.getElementById('gripper-slider');
    const label = document.getElementById('gripper-val');
    if (slider) slider.value = clamped;
    if (label) label.textContent = `${Math.round(clamped)}%`;
    this.view.setGripperOpenness(clamped);
  }

  runPickAndPlaceSequence() {
    if (this.validator.isEStopped) {
      SafeLogger.warn('Pick & Place bloqué : E-STOP actif');
      return;
    }
    this._animationToken++;
    const token = this._animationToken;
    this.view.resetObjectPositions();

    const limits = this.activeConfig.limits;
    const cubePos = this.view.targetCube.position.clone();
    const dropPos = new THREE.Vector3(-25, 3, 20);
    const startAngles = this.getCurrentAngles();

    const hoverCubeAngles = this.kinematics.solveIK(new THREE.Vector3(cubePos.x, cubePos.y + 10, cubePos.z), startAngles, limits);
    const graspAngles = this.kinematics.solveIK(cubePos, hoverCubeAngles, limits);
    const hoverDropAngles = this.kinematics.solveIK(new THREE.Vector3(dropPos.x, dropPos.y + 12, dropPos.z), graspAngles, limits);
    const releaseAngles = this.kinematics.solveIK(dropPos, hoverDropAngles, limits);

    this.setGripperUI(100);
    this.animateToAngles(hoverCubeAngles, 800, () => {
      if (token !== this._animationToken) return;
      this.animateToAngles(graspAngles, 600, () => {
        if (token !== this._animationToken) return;
        this.animateGripper(0, 400, () => {
          if (token !== this._animationToken) return;
          this.view.attachObjectToGripper(this.view.targetCube);
          this.animateToAngles(hoverDropAngles, 1000, () => {
            if (token !== this._animationToken) return;
            this.animateToAngles(releaseAngles, 600, () => {
              if (token !== this._animationToken) return;
              this.animateGripper(100, 400, () => {
                if (token !== this._animationToken) return;
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
    // ── FIX #17 : durée plancher ──
    const duration = Math.max(16, baseDuration / this.speedFactor);
    const startVal = parseFloat(document.getElementById('gripper-slider')?.value || '100');
    const startTime = performance.now();

    const step = (now) => {
      if (this.validator.isEStopped) return;
      if (now - startTime > SAFETY.ANIMATION_TIMEOUT_MS) {
        SafeLogger.warn('Animation gripper timeout');
        return;
      }
      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1.0);
      this.setGripperUI(startVal + (targetVal - startVal) * progress);
      if (progress < 1.0) requestAnimationFrame(step);
      else if (onComplete) onComplete();
    };
    requestAnimationFrame(step);
  }

  runWaypointTrajectory() {
    if (this.waypointMgr.waypoints.length === 0 || this.validator.isEStopped) return;
    this._animationToken++;
    const token = this._animationToken;

    let wpIdx = 0;
    const executeWp = () => {
      if (token !== this._animationToken) return;
      if (wpIdx >= this.waypointMgr.waypoints.length || this.validator.isEStopped) return;
      const target = this.waypointMgr.waypoints[wpIdx].angles;
      this.animateToAngles(target, 1000, () => {
        if (token !== this._animationToken) return;
        wpIdx++;
        setTimeout(executeWp, 200 / this.speedFactor);
      });
    };
    executeWp();
  }

  
  animateToAngles(targetAngles, baseDuration, onComplete) {
    const myToken = this._animationToken;
    const startAngles = this.getCurrentAngles();
    const duration = Math.max(50, baseDuration / this.speedFactor);

    if (!this.validator.isVelocitySafe(startAngles, targetAngles, duration)) {
      SafeLogger.warn('Animation bloquée : vitesse articulaire excessive');
      return;
    }

    const startTime = performance.now();

    const animStep = (now) => {
      if (myToken !== this._animationToken) return; // annulé
      if (this.validator.isEStopped) return;
      if (now - startTime > SAFETY.ANIMATION_TIMEOUT_MS) {
        SafeLogger.warn('Animation timeout');
        return;
      }

      const elapsed = now - startTime;
      const progress = Math.min(elapsed / duration, 1.0);
      const ease = progress * progress * progress * (progress * (progress * 6 - 15) + 10);

      const interpolated = startAngles.map((s, i) => {
        const t = targetAngles[i] !== undefined ? targetAngles[i] : s;
        return s + (t - s) * ease;
      });

      this.setAngles(interpolated, true);

      if (progress < 1.0) requestAnimationFrame(animStep);
      else if (onComplete) onComplete();
    };

    requestAnimationFrame(animStep);
  }

  startLoop() {
    const loop = () => {
      if (!this._paused) this.view.render();
      requestAnimationFrame(loop);
    };
    requestAnimationFrame(loop);
  }
}


window.addEventListener('DOMContentLoaded', () => {
  try {
    const test = document.createElement('canvas');
    const gl = test.getContext('webgl2') || test.getContext('webgl');
    if (!gl) {
      document.getElementById('loader')?.classList.add('hidden');
      const err = document.getElementById('webgl-error');
      if (err) err.hidden = false;
      return;
    }

    new RobotApp();
    SafeLogger.info('Application initialisée');
  } catch (e) {
    SafeLogger.error('Échec initialisation', e);
    document.getElementById('loader')?.classList.add('hidden');
  }
});
