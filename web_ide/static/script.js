// Three.js Variables
let scene, camera, renderer, qbotGroup, officeGroup, zonesGroup;
let fpvCamera, fpvRenderer;
let isAnimating = false;
let qbotState = { action: 'stand', rotationAngle: 0, wasColliding: false };
let wheelMeshes = [];
let currentZone = null;
let wallMeshes = [];

// Persistent Point Cloud Mapping
let mapScene, mapCamera, mapRenderer, mapControls, qbotMarker;
let mapPointCloud;
const maxMapPoints = 100000;
const mapGeometry = new THREE.BufferGeometry();
const mapPositions = new Float32Array(maxMapPoints * 3);
let mapPointIndex = 0;
mapGeometry.setAttribute('position', new THREE.BufferAttribute(mapPositions, 3));

// Voxel Environment Reconstruction
let voxelMesh;
const maxVoxels = 50000;
const voxelSize = 0.5;
const occupiedVoxels = new Set();
let voxelCount = 0;

// A* Pathfinding Variables
let currentPath = [];
let targetWaypoint = null;
let patrolWaypoints = [
    {x: 18, z: 1},  // Server Room
    {x: 18, z: 17}, // Main Lobby
    {x: 1, z: 17},  // High Voltage Lab
    {x: 1, z: 1}    // Back to Start
];
let currentWaypointIndex = 0;
let pathLineMesh = null; // Visual line for A* path

// 3D Point Cloud LiDAR Variables
let lidarPointCloud;
const maxPoints = 144; // 2.5 degree resolution
const lidarGeometry = new THREE.BufferGeometry();
const lidarPositions = new Float32Array(maxPoints * 3);
lidarGeometry.setAttribute('position', new THREE.BufferAttribute(lidarPositions, 3));

// Grid Map for Maze Generation and A* Routing (20x20)
const mazeGrid = [
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1],
    [1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1],
    [1,0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,0,1,0,1],
    [1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0,1],
    [1,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,0,1,1,1,1,0,1,0,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,0,1,1,1,1,1,1,1,0,1,1,1,1,0,1],
    [1,0,1,0,0,0,1,0,0,0,0,0,1,0,0,0,0,1,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
];

function initThreeJS() {
    const container = document.getElementById('sim-canvas');
    scene = new THREE.Scene();
    scene.name = "Gazebo Environment";
    scene.background = new THREE.Color(0x22242a);
    scene.fog = new THREE.FogExp2(0x22242a, 0.03);
    
    const hemiLight = new THREE.HemisphereLight(0xffffff, 0x444444, 0.6);
    hemiLight.position.set(0, 200, 0);
    scene.add(hemiLight);

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(10, 20, 10);
    dirLight.castShadow = true;
    dirLight.shadow.camera.top = 25;
    dirLight.shadow.camera.bottom = -25;
    dirLight.shadow.camera.left = -25;
    dirLight.shadow.camera.right = 25;
    dirLight.shadow.mapSize.width = 2048;
    dirLight.shadow.mapSize.height = 2048;
    scene.add(dirLight);

    camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 150);
    camera.position.set(-15, 25, 15);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 0.5, 0);
    controls.update();

    const fpvContainer = document.getElementById('fpv-container');
    fpvRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    fpvRenderer.setSize(fpvContainer.clientWidth, fpvContainer.clientHeight);
    fpvContainer.appendChild(fpvRenderer.domElement);
    fpvCamera = new THREE.PerspectiveCamera(60, fpvContainer.clientWidth / fpvContainer.clientHeight, 0.1, 50);

    buildOfficeEnvironment();
    buildQBot2();
    initMappingView();

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);

        if (mapRenderer && mapCamera) {
            const mContainer = document.getElementById('map-canvas');
            mapCamera.aspect = mContainer.clientWidth / mContainer.clientHeight;
            mapCamera.updateProjectionMatrix();
            mapRenderer.setSize(mContainer.clientWidth, mContainer.clientHeight);
        }
    });

    animate();
}

function buildOfficeEnvironment() {
    officeGroup = new THREE.Group();
    officeGroup.name = "Grid Maze Topography";
    wallMeshes = [];
    
    const floorGeo = new THREE.PlaneGeometry(40, 40);
    const floorMat = new THREE.MeshStandardMaterial({ color: 0x333344, roughness: 0.8 });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.receiveShadow = true;
    floor.name = "Main Floor";
    officeGroup.add(floor);
    
    const gridHelper = new THREE.GridHelper(40, 40, 0x555566, 0x2a2a35);
    gridHelper.position.y = 0.01;
    officeGroup.add(gridHelper);

    const wallMat = new THREE.MeshStandardMaterial({ color: 0x777788, roughness: 0.9 });
    const wallsGroup = new THREE.Group();
    wallsGroup.name = "Structural Walls";

    const wt = 0.4; // Wall thickness (thinner and realistic)

    // Horizontal segments
    for (let z = 0; z < 20; z++) {
        let startX = -1;
        for (let x = 0; x <= 20; x++) {
            if (x < 20 && mazeGrid[z][x] === 1) {
                if (startX === -1) startX = x;
            } else {
                if (startX !== -1) {
                    let endX = x - 1;
                    let w = (endX - startX) * 2 + wt;
                    let cx = ((startX + endX) / 2) * 2 - 19;
                    let cz = z * 2 - 19;
                    const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, 2.5, wt), wallMat);
                    mesh.position.set(cx, 1.25, cz);
                    mesh.castShadow = true;
                    mesh.receiveShadow = true;
                    wallMeshes.push(mesh);
                    wallsGroup.add(mesh);
                    startX = -1;
                }
            }
        }
    }

    // Vertical segments
    for (let x = 0; x < 20; x++) {
        let startZ = -1;
        for (let z = 0; z <= 20; z++) {
            if (z < 20 && mazeGrid[z][x] === 1) {
                if (startZ === -1) startZ = z;
            } else {
                if (startZ !== -1) {
                    let endZ = z - 1;
                    if (endZ > startZ) { // Skip isolated blocks, already drawn horizontally
                        let d = (endZ - startZ) * 2 + wt;
                        let cx = x * 2 - 19;
                        let cz = ((startZ + endZ) / 2) * 2 - 19;
                        // Height 2.49 to prevent top-face Z-fighting where walls overlap
                        const mesh = new THREE.Mesh(new THREE.BoxGeometry(wt, 2.49, d), wallMat);
                        mesh.position.set(cx, 1.245, cz);
                        mesh.castShadow = true;
                        mesh.receiveShadow = true;
                        wallMeshes.push(mesh);
                        wallsGroup.add(mesh);
                    }
                    startZ = -1;
                }
            }
        }
    }

    officeGroup.add(wallsGroup); // Add the generated walls to the visible scene!

    zonesGroup = new THREE.Group();
    zonesGroup.name = "Conditional Zones";

    function createZone(name, w, d, x, z, color, restricted) {
        const mat = new THREE.MeshBasicMaterial({ color: color, transparent: true, opacity: 0.15, side: THREE.DoubleSide });
        const mesh = new THREE.Mesh(new THREE.BoxGeometry(w, 2.5, d), mat);
        mesh.position.set(x, 1.25, z);
        mesh.name = name;
        mesh.userData = { isRestricted: restricted };
        zonesGroup.add(mesh);
    }

    createZone("Corridor Alpha [Default]", 36, 6, 0, -16, 0x0000ff, false);
    createZone("Restricted Area [Server Room]", 14, 14, 11, -7, 0xff0000, true);
    createZone("High Voltage Lab [Restricted]", 14, 14, -11, 7, 0xff0000, true);
    createZone("Main Lobby [Safe]", 14, 14, 11, 7, 0x00ff00, false);

    officeGroup.add(zonesGroup);
    scene.add(officeGroup);
}

function initMappingView() {
    const container = document.getElementById('map-canvas');
    mapScene = new THREE.Scene();
    mapScene.background = new THREE.Color(0x0a0a0a);
    
    // Add a helper grid to the mapping scene so we can see it's working
    const grid = new THREE.GridHelper(40, 40, 0x333333, 0x111111);
    mapScene.add(grid);

    mapCamera = new THREE.PerspectiveCamera(60, 1, 0.1, 1000); // Start with 1 aspect
    mapCamera.position.set(20, 20, 20);
    mapCamera.lookAt(0, 0, 0);

    mapRenderer = new THREE.WebGLRenderer({ antialias: true });
    // Don't set size here if 0, switchTab will handle it
    container.appendChild(mapRenderer.domElement);

    mapControls = new THREE.OrbitControls(mapCamera, mapRenderer.domElement);
    mapControls.target.set(0, 0, 0);
    mapControls.update();

    const ambient = new THREE.AmbientLight(0x404040);
    mapScene.add(ambient);
    const directional = new THREE.DirectionalLight(0xffffff, 0.5);
    directional.position.set(1, 1, 1);
    mapScene.add(directional);

    // Initialize Objects in Map Scene
    const mapMat = new THREE.PointsMaterial({ color: 0x00ffff, size: 0.04, transparent: true, opacity: 0.8 });
    mapPointCloud = new THREE.Points(mapGeometry, mapMat);
    mapPointCloud.name = "Persistent Point Cloud";
    mapScene.add(mapPointCloud);

    const voxelGeo = new THREE.BoxGeometry(voxelSize * 0.95, voxelSize * 0.95, voxelSize * 0.95);
    const voxelMat = new THREE.MeshStandardMaterial({ 
        color: 0x4488ff, 
        transparent: true, 
        opacity: 0.5,
        roughness: 0.3,
        metalness: 0.2
    });
    voxelMesh = new THREE.InstancedMesh(voxelGeo, voxelMat, maxVoxels);
    voxelMesh.name = "Reconstructed Voxel Environment";
    voxelMesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    voxelMesh.count = 0;
    mapScene.add(voxelMesh);

    // Add a robot marker to the map scene
    const markerGeo = new THREE.CylinderGeometry(0.4, 0.4, 0.2, 16);
    const markerMat = new THREE.MeshBasicMaterial({ color: 0xffff00, wireframe: true });
    qbotMarker = new THREE.Mesh(markerGeo, markerMat);
    mapScene.add(qbotMarker);
}

function addVoxel(x, y, z) {
    if (voxelCount >= maxVoxels) return;
    
    const vx = Math.floor(x / voxelSize);
    const vy = Math.floor(y / voxelSize);
    const vz = Math.floor(z / voxelSize);
    const key = `${vx},${vy},${vz}`;
    
    if (!occupiedVoxels.has(key)) {
        occupiedVoxels.add(key);
        
        const matrix = new THREE.Matrix4();
        matrix.setPosition(vx * voxelSize + voxelSize/2, vy * voxelSize + voxelSize/2, vz * voxelSize + voxelSize/2);
        voxelMesh.setMatrixAt(voxelCount, matrix);
        voxelCount++;
        voxelMesh.count = voxelCount;
        voxelMesh.instanceMatrix.needsUpdate = true;
    }
}

function buildQBot2() {
    qbotGroup = new THREE.Group();
    qbotGroup.name = "QUANSER QBot 2";
    
    // Spawn at (1,1) -> (-17, 0, -17)
    qbotGroup.position.set(-17, 0, -17);
    
    const chassisMat = new THREE.MeshStandardMaterial({ color: 0x333333, metalness: 0.6, roughness: 0.2 });
    const topMat = new THREE.MeshStandardMaterial({ color: 0xeeeeee, metalness: 0.2, roughness: 0.5 });
    const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.1, roughness: 0.8 });
    
    // Circular Base
    const baseGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.1, 32);
    const base = new THREE.Mesh(baseGeo, chassisMat);
    base.position.y = 0.15;
    base.castShadow = true;
    base.name = "Chassis Base";
    qbotGroup.add(base);

    // Mid Plate
    const midGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.02, 32);
    const mid = new THREE.Mesh(midGeo, chassisMat);
    mid.position.y = 0.35;
    mid.castShadow = true;
    qbotGroup.add(mid);

    // Top Plate
    const topGeo = new THREE.CylinderGeometry(0.35, 0.35, 0.02, 32);
    const top = new THREE.Mesh(topGeo, chassisMat);
    top.position.y = 0.55;
    top.castShadow = true;
    qbotGroup.add(top);

    // Pillars
    const pillarGeo = new THREE.CylinderGeometry(0.01, 0.01, 0.4, 8);
    for(let i=0; i<4; i++) {
        const pillar = new THREE.Mesh(pillarGeo, chassisMat);
        const angle = (i / 4) * Math.PI * 2;
        pillar.position.set(Math.cos(angle) * 0.3, 0.35, Math.sin(angle) * 0.3);
        qbotGroup.add(pillar);
    }
    
    // LiDAR / Kinect Sensor
    const sensorBoxGeo = new THREE.BoxGeometry(0.2, 0.1, 0.1);
    const sensor = new THREE.Mesh(sensorBoxGeo, topMat);
    sensor.position.set(0.1, 0.6, 0);
    sensor.name = "Kinect V2 Sensor";
    qbotGroup.add(sensor);

    const lidarMat = new THREE.PointsMaterial({ color: 0x00ffcc, size: 0.2, transparent: true, opacity: 0.8 });
    lidarPointCloud = new THREE.Points(lidarGeometry, lidarMat);
    lidarPointCloud.name = "Live LiDAR Scan";
    scene.add(lidarPointCloud);

    fpvCamera.position.set(0.2, 0.62, 0);
    fpvCamera.rotation.y = -Math.PI / 2;
    qbotGroup.add(fpvCamera);

    // Wheels
    const wheelGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.05, 16);
    wheelGeo.rotateX(Math.PI / 2);
    
    const leftWheel = new THREE.Mesh(wheelGeo, wheelMat);
    leftWheel.position.set(0, 0.12, 0.3);
    leftWheel.name = "Left Wheel";
    qbotGroup.add(leftWheel);
    wheelMeshes.push(leftWheel);

    const rightWheel = new THREE.Mesh(wheelGeo, wheelMat);
    rightWheel.position.set(0, 0.12, -0.3);
    rightWheel.name = "Right Wheel";
    qbotGroup.add(rightWheel);
    wheelMeshes.push(rightWheel);

    scene.add(qbotGroup);
}

// A* Implementation for optimal wall-avoidance routing
function aStar(startX, startZ, goalX, goalZ) {
    const openSet = [{x: startX, z: startZ, g: 0, f: 0, parent: null}];
    const closedSet = new Set();
    
    while(openSet.length > 0) {
        openSet.sort((a,b) => a.f - b.f);
        const current = openSet.shift();
        
        if (current.x === goalX && current.z === goalZ) {
            const path = [];
            let curr = current;
            while(curr) {
                path.push({x: curr.x, z: curr.z});
                curr = curr.parent;
            }
            return path.reverse();
        }
        
        closedSet.add(`${current.x},${current.z}`);
        
        const neighbors = [
            {x: current.x+1, z: current.z},
            {x: current.x-1, z: current.z},
            {x: current.x, z: current.z+1},
            {x: current.x, z: current.z-1}
        ];
        
        for (let n of neighbors) {
            if (n.x >= 0 && n.x < 20 && n.z >= 0 && n.z < 20) {
                if (mazeGrid[n.z][n.x] === 0 && !closedSet.has(`${n.x},${n.z}`)) {
                    const g = current.g + 1;
                    const h = Math.abs(n.x - goalX) + Math.abs(n.z - goalZ);
                    const existing = openSet.find(o => o.x === n.x && o.z === n.z);
                    if (!existing || g < existing.g) {
                        if (!existing) openSet.push({x: n.x, z: n.z, g: g, f: g+h, parent: current});
                        else { existing.g = g; existing.f = g+h; existing.parent = current; }
                    }
                }
            }
        }
    }
    return [];
}

function updateVisualPathLine() {
    if (pathLineMesh) {
        scene.remove(pathLineMesh);
        pathLineMesh.geometry.dispose();
        pathLineMesh.material.dispose();
        pathLineMesh = null;
    }
    
    if (currentPath.length === 0 && !targetWaypoint) return;
    
    const points = [];
    points.push(new THREE.Vector3(qbotGroup.position.x, 0.05, qbotGroup.position.z));
    if (targetWaypoint) {
        points.push(new THREE.Vector3(targetWaypoint.x, 0.05, targetWaypoint.z));
    }
    for(let i=0; i<currentPath.length; i++) {
        points.push(new THREE.Vector3((currentPath[i].x*2)-19, 0.05, (currentPath[i].z*2)-19));
    }
    
    if (points.length > 1) {
        const geo = new THREE.BufferGeometry().setFromPoints(points);
        const mat = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 3 });
        pathLineMesh = new THREE.Line(geo, mat);
        scene.add(pathLineMesh);
    }
}

function computeNextPatrolPath() {
    let gridX = Math.round((qbotGroup.position.x + 19) / 2);
    let gridZ = Math.round((qbotGroup.position.z + 19) / 2);
    let goal = patrolWaypoints[currentWaypointIndex];
    
    currentPath = aStar(gridX, gridZ, goal.x, goal.z);
    if (currentPath.length > 0) {
        currentPath.shift(); // remove starting node
    }
    logTelemetry(`[A* PATHFINDER] Optimized collision-free route to Waypoint ${currentWaypointIndex}. Nodes: ${currentPath.length}`);
    updateVisualPathLine();
}


function checkZones() {
    const pos = qbotGroup.position;
    let insideZone = null;
    let isRestricted = false;
    zonesGroup.children.forEach(zone => {
        const box = new THREE.Box3().setFromObject(zone);
        if (box.containsPoint(pos)) {
            insideZone = zone.name;
            isRestricted = zone.userData.isRestricted;
        }
    });
    if (insideZone !== currentZone) {
        if (insideZone) {
            logTelemetry(`[Spatial] Entered: ${insideZone}`);
            if (isRestricted) {
                logTelemetry(`[ALERT] POLICY VIOLATION! Unauthorized Entry into ${insideZone}.`);
                const zoneMesh = zonesGroup.children.find(z => z.name === insideZone);
                zoneMesh.material.opacity = 0.6;
                setTimeout(() => zoneMesh.material.opacity = 0.15, 500);
            }
        } else {
            logTelemetry(`[Spatial] Exited zone. Now in unmarked territory.`);
        }
        currentZone = insideZone;
    }
}

function checkCollisions() {
    const pos = qbotGroup.position;
    // Create a fixed, rotation-independent tiny bounding box in the center of the robot
    const chassisBox = new THREE.Box3(
        new THREE.Vector3(pos.x - 0.25, pos.y, pos.z - 0.25),
        new THREE.Vector3(pos.x + 0.25, pos.y + 0.5, pos.z + 0.25)
    );
    for (let i = 0; i < wallMeshes.length; i++) {
        const wallBox = new THREE.Box3().setFromObject(wallMeshes[i]);
        // Also shrink the wall's collision box slightly to give smooth sliding clearance
        wallBox.expandByScalar(-0.2);
        if (chassisBox.intersectsBox(wallBox)) return true;
    }
    return false;
}

function updateLiDARPointCloud() {
    const origin = qbotGroup.position.clone();
    origin.y = 0.6; 
    const raycaster = new THREE.Raycaster();
    for(let i=0; i<maxPoints; i++) {
        const angle = (i / maxPoints) * Math.PI * 2;
        const dir = new THREE.Vector3(Math.cos(angle + qbotGroup.rotation.y), 0, Math.sin(angle + qbotGroup.rotation.y)).normalize();
        raycaster.set(origin, dir);
        const intersects = raycaster.intersectObjects(wallMeshes);
        let pt = new THREE.Vector3();
        if(intersects.length > 0) {
            pt.copy(intersects[0].point);
            
            // Persistent Mapping: Store point in map buffer
            mapPositions[mapPointIndex * 3] = pt.x;
            mapPositions[mapPointIndex * 3 + 1] = pt.y;
            mapPositions[mapPointIndex * 3 + 2] = pt.z;
            mapPointIndex = (mapPointIndex + 1) % maxMapPoints;

            // Voxel Reconstruction
            addVoxel(pt.x, pt.y, pt.z);
        }
        else pt.copy(origin).add(dir.multiplyScalar(20));
        
        lidarPositions[i*3] = pt.x;
        lidarPositions[i*3+1] = pt.y;
        lidarPositions[i*3+2] = pt.z;
    }
    lidarGeometry.attributes.position.needsUpdate = true;
    mapGeometry.attributes.position.needsUpdate = true;

    // Update UI Stats for Mapping
    const mapPts = document.getElementById('map-pts');
    if (mapPts) mapPts.textContent = mapPointIndex;
    const mapVox = document.getElementById('map-vox');
    if (mapVox) mapVox.textContent = `${voxelCount} Voxels`;
}

function updateSensors(isColliding) {
    const pos = qbotGroup.position;
    document.getElementById('val-pos').textContent = `X: ${pos.x.toFixed(2)} | Z: ${pos.z.toFixed(2)}`;
    let heading = qbotGroup.rotation.y * (180 / Math.PI);
    heading = (heading % 360 + 360) % 360; 
    document.getElementById('val-head').textContent = `${heading.toFixed(1)}°`;
    const colSpan = document.getElementById('val-col');
    if (isColliding) {
        colSpan.textContent = "OBSTRUCTED";
        colSpan.className = "warning";
    } else {
        colSpan.textContent = "CLEAR";
        colSpan.className = "safe";
    }
    document.getElementById('val-zone').textContent = currentZone ? currentZone.split(' ')[0] + " " + currentZone.split(' ')[1] : "Unmarked";
    const raycaster = new THREE.Raycaster();
    const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(qbotGroup.quaternion).normalize();
    const rayPos = qbotGroup.position.clone();
    rayPos.y = 0.6; 
    raycaster.set(rayPos, forward);
    const intersects = raycaster.intersectObjects(wallMeshes);
    if(intersects.length > 0) document.getElementById('val-lidar').textContent = `${intersects[0].distance.toFixed(2)}m`;
    else document.getElementById('val-lidar').textContent = `> 20m`;
}

function animate() {
    requestAnimationFrame(animate);
    updateLiDARPointCloud();
    
    let prevPosition = qbotGroup.position.clone();
    let prevRotation = qbotGroup.rotation.y;

    if (qbotState.action === 'auto_patrol' || qbotState.action === 'moving_to') {
        
        if (qbotState.action === 'auto_patrol' && !targetWaypoint && currentPath.length === 0) {
            currentWaypointIndex = (currentWaypointIndex + 1) % patrolWaypoints.length;
            computeNextPatrolPath();
        }
        
        if (!targetWaypoint && currentPath.length > 0) {
            let nextGridNode = currentPath.shift();
            targetWaypoint = new THREE.Vector3((nextGridNode.x * 2) - 19, 0, (nextGridNode.z * 2) - 19);
            updateVisualPathLine();
        }
        
        if (targetWaypoint) {
            const direction = new THREE.Vector3().subVectors(targetWaypoint, qbotGroup.position);
            direction.y = 0;
            if (direction.length() < 0.15) {
                qbotGroup.position.x = targetWaypoint.x;
                qbotGroup.position.z = targetWaypoint.z;
                
                targetWaypoint = null;
                updateVisualPathLine();
                
                if (qbotState.action === 'moving_to' && currentPath.length === 0) {
                    qbotState.action = 'stand'; 
                }
            } else {
                const targetAngle = Math.atan2(-direction.z, direction.x);
                let diff = targetAngle - qbotGroup.rotation.y;
                while (diff < -Math.PI) diff += Math.PI * 2;
                while (diff > Math.PI) diff -= Math.PI * 2;
                
                if (Math.abs(diff) > 0.05) {
                    qbotGroup.rotation.y += Math.sign(diff) * 0.12;
                    animateWheels(Math.sign(diff) * 0.12, -Math.sign(diff) * 0.12);
                } else {
                    qbotGroup.rotation.y = targetAngle;
                    qbotGroup.translateX(0.08);
                    animateWheels(0.2, 0.2);
                }
            }
        }
    } else if (qbotState.action === 'walk') {
        qbotGroup.translateX(0.04);
        animateWheels(0.1, 0.1);
    } else if (qbotState.action === 'turn_left') {
        qbotGroup.rotation.y += 0.02; 
        animateWheels(-0.05, 0.05);
    } else if (qbotState.action === 'turn_right') {
        qbotGroup.rotation.y -= 0.02; 
        animateWheels(0.05, -0.05);
    }
    
    qbotGroup.updateMatrixWorld();
    let isColliding = checkCollisions();
    if (isColliding) {
        qbotGroup.position.copy(prevPosition);
        qbotGroup.rotation.y = prevRotation;
        if (!qbotState.wasColliding) {
            logTelemetry("[PHYSICS] Collision Detected! Path obstructed by wall.");
            qbotState.wasColliding = true;
        }
    } else {
        qbotState.wasColliding = false;
    }
    
    checkZones();
    updateSensors(isColliding);

    renderer.render(scene, camera);
    fpvRenderer.render(scene, fpvCamera);
    
    if (mapRenderer && mapScene && mapCamera) {
        if (qbotMarker && qbotGroup) {
            qbotMarker.position.copy(qbotGroup.position);
            qbotMarker.rotation.copy(qbotGroup.rotation);
        }
        if (mapControls) mapControls.update();
        mapRenderer.render(mapScene, mapCamera);
    }
}

function animateWheels(leftSpeed, rightSpeed) {
    wheelMeshes[0].rotation.y += leftSpeed;
    wheelMeshes[1].rotation.y += rightSpeed;
}

function logTelemetry(msg) {
    const t = document.getElementById('telemetry-output');
    t.textContent += `> ${msg}\n`;
    t.scrollTop = t.scrollHeight;
}

function connectWebSocket() {
    const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
    const wsUrl = `${protocol}//${window.location.host}/api/ws/sim`;
    const ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        logTelemetry("Connected to WebSocket Relay.");
        
        // Broadcast Live Telemetry to Python clients at 10Hz
        setInterval(() => {
            if (ws.readyState === WebSocket.OPEN && qbotGroup) {
                const pos = qbotGroup.position;
                let heading = qbotGroup.rotation.y * (180 / Math.PI);
                heading = (heading % 360 + 360) % 360; 
                
                const raycaster = new THREE.Raycaster();
                const forward = new THREE.Vector3(1, 0, 0).applyQuaternion(qbotGroup.quaternion).normalize();
                const rayPos = pos.clone();
                rayPos.y = 0.6; 
                raycaster.set(rayPos, forward);
                let dist = 20.0;
                if (wallMeshes) {
                    const intersects = raycaster.intersectObjects(wallMeshes);
                    if(intersects.length > 0) dist = intersects[0].distance;
                }
                
                ws.send(JSON.stringify({
                    telemetry: {
                        x: pos.x,
                        z: pos.z,
                        heading: heading,
                        lidar: dist
                    }
                }));
            }
        }, 100);
    };
    
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.action) {
                // If we are currently running the smart A* patrol, ignore blind manual commands from old ghost scripts!
                if (qbotState.action === 'auto_patrol' && ['walk', 'turn_left', 'turn_right'].includes(data.action)) {
                    return; 
                }
                
                qbotState.action = data.action;
                logTelemetry(`[Drive] Action received: ${data.action}`);
                if (data.action === 'reset') {
                    qbotGroup.position.set(-17,0,-17);
                    qbotGroup.rotation.set(0,0,0);
                    qbotState.action = 'stand';
                    qbotState.wasColliding = false;
                    currentPath = [];
                    targetWaypoint = null;
                    currentWaypointIndex = 0;
                    updateVisualPathLine();
                    
                    // Clear Map on Reset
                    mapPositions.fill(0);
                    mapPointIndex = 0;
                    mapGeometry.attributes.position.needsUpdate = true;
                    
                    // Clear Voxels on Reset
                    occupiedVoxels.clear();
                    voxelCount = 0;
                    voxelMesh.count = 0;
                    voxelMesh.instanceMatrix.needsUpdate = true;
                } else if (data.action === 'auto_patrol') {
                    computeNextPatrolPath();
                } else if (data.action === 'move_to') {
                    let gx = Math.round((data.target[0] + 19) / 2);
                    let gz = Math.round((data.target[1] + 19) / 2);
                    let newGoal = new THREE.Vector3(data.target[0], 0, data.target[1]);
                    
                    // Only recompute A* if the goal has changed significantly to avoid route stuttering
                    if (!qbotState.finalGoal || newGoal.distanceTo(qbotState.finalGoal) > 1.0) {
                        qbotState.finalGoal = newGoal;
                        let sx = Math.round((qbotGroup.position.x + 19) / 2);
                        let sz = Math.round((qbotGroup.position.z + 19) / 2);
                        
                        currentPath = aStar(sx, sz, gx, gz);
                        if (currentPath.length > 0) {
                            currentPath.shift(); // Remove current node
                            let nextNode = currentPath.shift();
                            if (nextNode) {
                                targetWaypoint = new THREE.Vector3((nextNode.x * 2) - 19, 0, (nextNode.z * 2) - 19);
                            }
                        } else {
                            targetWaypoint = newGoal; // Fallback to direct path
                        }
                    }
                    qbotState.action = 'moving_to';
                    updateVisualPathLine();
                }
            } else if (data.log) {
                logTelemetry(data.log);
            }
        } catch (e) {
            logTelemetry(event.data);
        }
    };
    
    ws.onclose = () => {
        logTelemetry("WebSocket disconnected. Retrying in 2s...");
        setTimeout(connectWebSocket, 2000);
    };
}

function switchTab(tabId) {
    document.querySelectorAll('.tab-section').forEach(s => s.classList.remove('active'));
    document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
    
    document.getElementById(`tab-${tabId}`).classList.add('active');
    
    // Find button by text content or data attribute if we had one
    const btns = document.querySelectorAll('.tab-btn');
    btns.forEach(btn => {
        if (btn.textContent.toLowerCase().includes(tabId)) btn.classList.add('active');
    });

    // CRITICAL: Update mapping camera aspect ratio when tab becomes visible
    if (tabId === 'mapping' && mapRenderer && mapCamera) {
        const container = document.getElementById('map-canvas');
        if (container.clientWidth > 0) {
            mapCamera.aspect = container.clientWidth / container.clientHeight;
            mapCamera.updateProjectionMatrix();
            mapRenderer.setSize(container.clientWidth, container.clientHeight);
        }
    }
}

function resetMapData() {
    // Clear Map buffer
    mapPositions.fill(0);
    mapPointIndex = 0;
    mapGeometry.attributes.position.needsUpdate = true;
    
    // Clear Voxels
    occupiedVoxels.clear();
    voxelCount = 0;
    voxelMesh.count = 0;
    voxelMesh.instanceMatrix.needsUpdate = true;
    
    logTelemetry("[Map] Persistent reconstruction data purged.");
}

document.addEventListener("DOMContentLoaded", () => {
    initThreeJS();
    connectWebSocket();
});
