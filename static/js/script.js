/* -------------------------------------------
    Globals & state - Swarm Kamikaze System
    -------------------------------------------*/
let map, gridLayer, pathLayer, drawnItems, drawControl;
let isDrawing = false, cameraOn = false;
let initialCenterSet = false, gridCells = [];
let droneMarkers = {};  // port -> marker mapping
let dronePaths = {};    // port -> polyline mapping
let connectedDrones = [];
let currentCameraPort = null;
let cameraSelectionInProgress = false; // Camera selection lock
let cameraUpdateTimeout = null; // Camera update timeout

// --- RADAR GLOBALS ---
let followTarget = 'LEADER'; // 'LEADER' or Drone Port

// --- PERFORMANCE OPTIMIZATIONS ---
let _statusAbortController = null;   // AbortController for stale request cancellation
let _statusFetchInProgress = false;  // Prevent overlapping status fetches

/** Debounce utility — delays execution until pause in calls */
function debounce(fn, delay) {
  let timer;
  return function (...args) {
    clearTimeout(timer);
    timer = setTimeout(() => fn.apply(this, args), delay);
  };
}

/** Throttle utility — limits execution rate */
function throttle(fn, limit) {
  let lastCall = 0;
  return function (...args) {
    const now = Date.now();
    if (now - lastCall >= limit) {
      lastCall = now;
      return fn.apply(this, args);
    }
  };
}

/* -------------------------------------------
    Panel Toggle (Accordion)
    -------------------------------------------*/
function togglePanel(id) {
  const panel = document.getElementById(id);
  const content = (id === 'radarWidget') ? document.getElementById('radarContainer') : document.getElementById('cameraContent');

  if (!panel || !content) return;

  // Toggle state
  if (content.style.height === '0px') {
    // Expand
    content.style.height = (id === 'radarWidget') ? '24rem' : '270px'; // 24rem = h-96
    content.style.opacity = '1';
    content.style.marginTop = '0';
    panel.classList.remove('h-[40px]'); // remove min height constraint
  } else {
    // Collapse
    content.style.height = '0px';
    content.style.opacity = '0';
    content.style.marginTop = '0';
    // Optional: add a class to panel to reduce specific padding if needed
  }
}

let isTooltipPinned = false;
let pinnedElementId = null;

function formatLocalIDs(localIds) {
  if (!localIds || Object.keys(localIds).length === 0) return "None";
  return Object.entries(localIds)
    .map(([port, lid]) => `<div class="ml-2">• Drone ${port}: Local ID ${lid}</div>`)
    .join('');
}

function setRadarMode(mode) {
  // Overridden: 'leader' -> follow Leader, 'drone' -> follow selected drone
  if (mode === 'leader') {
    followTarget = 'LEADER';
    document.getElementById('radarModeLeader').className = "px-2 py-0.5 text-[10px] rounded bg-blue-600 text-white";
    document.getElementById('radarModeDrone').className = "px-2 py-0.5 text-[10px] rounded text-white/50 hover:text-white";
  } else {
    // Switch to currently selected camera drone, if any
    if (currentCameraPort) {
      followTarget = currentCameraPort;
      document.getElementById('radarModeLeader').className = "px-2 py-0.5 text-[10px] rounded text-white/50 hover:text-white";
      document.getElementById('radarModeDrone').className = "px-2 py-0.5 text-[10px] rounded bg-blue-600 text-white";
    } else {
      showToast("Select a drone first", "info");
    }
  }
}

function drawErrorEllipse(container, x, y, pxPerMeter, covariance) {
  if (!covariance || covariance.length < 2) return;

  // Covariance is NED: 0=North(Y), 1=East(X)
  const varY = covariance[0][0]; // North
  const varX = covariance[1][1]; // East
  const covXY = covariance[0][1]; // North-East

  // Eigenvalues for 2x2 matrix
  const delta = Math.sqrt(Math.pow(varX - varY, 2) + 4 * Math.pow(covXY, 2));
  const lambda1 = (varX + varY + delta) / 2;
  const lambda2 = (varX + varY - delta) / 2;

  const majorAxis = 2 * Math.sqrt(lambda1); // 2-sigma diameter? No, using 1-sigma radius -> 2*sqrt
  const minorAxis = 2 * Math.sqrt(lambda2);

  // Angle
  const theta = 0.5 * Math.atan2(2 * covXY, varX - varY); // Radians

  const stdDevMetrics = Math.sqrt(Math.max(lambda1, lambda2));

  // Threshold Check (< 1.0m hidden)
  if (stdDevMetrics < 1.0) return;

  // Color Logic
  let color = 'rgba(74, 222, 128, 0.3)'; // Green-ish
  let borderColor = 'rgba(74, 222, 128, 0.8)';

  if (stdDevMetrics > 3.0) {
    color = 'rgba(239, 68, 68, 0.3)'; // Red-ish
    borderColor = 'rgba(239, 68, 68, 0.8)';
  }

  const ellipse = document.createElement('div');
  ellipse.className = 'absolute border transform -translate-x-1/2 -translate-y-1/2 rounded-full pointer-events-none';
  ellipse.style.left = `${x}px`;
  ellipse.style.top = `${y}px`;
  ellipse.style.width = `${majorAxis * pxPerMeter}px`;
  ellipse.style.height = `${minorAxis * pxPerMeter}px`;
  ellipse.style.backgroundColor = color;
  ellipse.style.borderColor = borderColor;

  // Physics Rotation: Theta is angle from X-axis. On screen X is East, Y is -North. 
  // NED X=North(Y screen), Y=East(X screen).
  // It's tricky. 
  // Simplified: Rotate by theta (rad) converted to deg.
  // CSS rotate is CW. Math is CCW.
  ellipse.style.transform = `translate(-50%, -50%) rotate(${-theta * 180 / Math.PI}deg)`;
  ellipse.style.zIndex = "1";

  container.appendChild(ellipse);
}

function drawRadar(data) {
  const radarLayer = document.getElementById('radarTargetsLayer');
  if (!radarLayer) return;

  // 1. Determine Center
  let centerLat = 0, centerLon = 0;

  // Find Leader Position first
  let leaderPos = null;
  if (data.drones) {
    Object.values(data.drones).forEach(d => {
      if (d.role === 'LEADER') leaderPos = d;
    });
  }

  if (followTarget === 'LEADER' && leaderPos) {
    centerLat = leaderPos.lat;
    centerLon = leaderPos.lon;
  } else if (followTarget !== 'LEADER' && data.drones && data.drones[followTarget]) {
    const d = data.drones[followTarget];
    centerLat = d.lat;
    centerLon = d.lon;
  } else if (leaderPos) {
    centerLat = leaderPos.lat;
    centerLon = leaderPos.lon;
  } else {
    // Fallback to centroid
    let sLat = 0, sLon = 0, c = 0;
    if (data.drones) Object.values(data.drones).forEach(d => { if (d.lat != 0) { sLat += d.lat; sLon += d.lon; c++ } });
    if (c > 0) { centerLat = sLat / c; centerLon = sLon / c; }
  }

  // 2. Prep Fragment
  const fragment = document.createDocumentFragment();
  const w = radarLayer.clientWidth;
  const h = radarLayer.clientHeight;
  const rangeM = 100.0;
  const pxPerMeter = (w / 2) / rangeM;

  const toScreen = (lat, lon) => {
    const dLat = lat - centerLat;
    const dLon = lon - centerLon;
    const distY = dLat * 111132.0;
    const distX = dLon * 111132.0 * Math.cos(centerLat * Math.PI / 180);
    return {
      x: (w / 2) + (distX * pxPerMeter),
      y: (h / 2) - (distY * pxPerMeter)
    };
  };

  // 3. Draw Lines (Drone -> Target)
  if (data.drones && data.targets) {
    Object.entries(data.drones).forEach(([pid, d]) => {
      // Check engagements/assignments
      let targetId = null;
      let styleClass = 'border-t border-white/40 border-dashed'; // PENDING / Assigned

      // If actively engaging
      if (d.engaged_target_id && data.targets[d.engaged_target_id]) {
        targetId = d.engaged_target_id;
        const tgtStatus = data.targets[targetId].status;

        if (['ECHO_WAIT', 'CENTERING'].includes(tgtStatus)) {
          styleClass = 'border-t-2 border-yellow-400 border-dashed shadow-[0_0_5px_yellow]'; // Preparing/Centering
        } else if (['ENGAGED', 'ATTACKING', 'CONFIRMED_ATTACK'].includes(tgtStatus)) {
          styleClass = 'border-t-2 border-red-500 shadow-[0_0_5px_red]'; // Solid Thick Red (Attacking)
        } else {
          styleClass = 'border-t-2 border-orange-500 border-dashed'; // LOCKED or transitioning
        }
      }
      // Else check targets for assignment
      else {
        Object.entries(data.targets).forEach(([tid, t]) => {
          if (String(t.assigned_to) === String(pid) && !targetId) {
            targetId = tid;
          }
        });
      }

      if (targetId && data.targets[targetId]) {
        const t = data.targets[targetId];
        const start = toScreen(d.lat, d.lon);
        const end = toScreen(t.lat, t.lon);

        const dx = end.x - start.x;
        const dy = end.y - start.y;
        const len = Math.sqrt(dx * dx + dy * dy);
        const angle = Math.atan2(dy, dx) * 180 / Math.PI;

        const line = document.createElement('div');
        line.className = `absolute origin-top-left ${styleClass}`;
        line.style.left = `${start.x}px`;
        line.style.top = `${start.y}px`;
        line.style.width = `${len}px`;
        line.style.transform = `rotate(${angle}deg)`;
        line.style.zIndex = "1";
        fragment.appendChild(line);
      }
    });
  }

  // 4. Draw Targets & Covariance
  if (data.targets) {
    Object.entries(data.targets).forEach(([tid, t]) => {
      if (t.track_state === 'DELETED') return;

      // FIX: Show CONFIRMED targets, or TENTATIVE targets ONLY IF they are actively assigned/engaged
      let isTargetEngaged = false;
      if (t.assigned_to) isTargetEngaged = true;
      else {
        Object.values(data.drones).forEach(dIter => {
          if (String(dIter.engaged_target_id) === String(tid)) isTargetEngaged = true;
        });
      }
      // FIX: Show all targets including unassigned TENTATIVE targets (Requested by User)
      // if (t.track_state !== 'CONFIRMED' && !isTargetEngaged) return;

      const pos = toScreen(t.lat, t.lon);
      if (pos.x < -20 || pos.x > w + 20 || pos.y < -20 || pos.y > h + 20) return;

      // BUG 6 FIX: UI State Drift for Active Protocol Targets
      const activeStates = ['LOCKED', 'ENGAGED', 'ACTIVE', 'CONFIRMED_ATTACK', 'CENTERING', 'ATTACKING'];

      // Draw Ellipse (if locked or engaged or in protocol)
      if (activeStates.includes(t.status)) {
        drawErrorEllipse(fragment, pos.x, pos.y, pxPerMeter, t.covariance);
      }

      const el = document.createElement('div');

      // Strict Colors and Sizes
      let bg = 'bg-gray-500/30 ring-1 ring-gray-400'; // Tentative (Faint but visible)
      let size = 'w-1.5 h-1.5';
      let extra = '';

      if (t.track_state === 'CONFIRMED') {
        bg = 'bg-[#EF4444]'; // Solid Red
        size = 'w-2 h-2';
      }

      // Group targets: larger marker with green halo
      if (t.is_group) {
        size = 'w-4 h-4';
        if (activeStates.includes(t.status)) {
          bg = 'bg-[#EF4444]'; // Red for engaged group
          extra = 'ring-2 ring-red-400 shadow-[0_0_12px_red] animate-pulse';
        } else {
          bg = 'bg-[#22C55E]'; // Green for normal group
          extra = 'ring-2 ring-green-400 shadow-[0_0_10px_rgba(34,197,94,0.6)]';
        }
      } else if (activeStates.includes(t.status)) {
        bg = 'bg-[#EF4444]'; // Solid Red
        size = 'w-3 h-3';
        if (['ECHO_WAIT', 'CENTERING'].includes(t.status)) {
          extra = 'ring-2 ring-yellow-400 shadow-[0_0_10px_yellow] animate-pulse'; // Yellow pulse for centering
        } else {
          extra = 'ring-2 ring-red-400 shadow-[0_0_10px_red] animate-pulse'; // Red pulse for attack
        }
      }

      el.className = `absolute ${size} rounded-full transform -translate-x-1/2 -translate-y-1/2 ${bg} ${extra}`;
      el.style.left = `${pos.x}px`;
      el.style.top = `${pos.y}px`;
      el.style.zIndex = "3";
      el.style.cursor = "pointer"; // Clickable

      // Group member count label (xN)
      if (t.is_group && t.group_member_count > 0) {
        const lbl = document.createElement('div');
        lbl.className = 'absolute text-[12px] font-bold text-green-400 pointer-events-none';
        lbl.style.left = `${pos.x + 10}px`;
        lbl.style.top = `${pos.y - 12}px`;
        lbl.style.zIndex = "4";
        lbl.style.textShadow = '0 0 6px rgba(0,0,0,0.9)';
        lbl.textContent = `x${t.group_member_count}`;
        fragment.appendChild(lbl);
      }

      const showTooltip = (e) => {
        const tt = document.getElementById('radarTooltip');
        tt.classList.remove('hidden');

        // Render content first so we can measure
        const groupLine = t.is_group
          ? `<div class="flex items-center gap-1.5 mb-1.5 pb-1 border-b border-green-500/30">
               <span class="bg-green-500/20 text-green-400 font-bold text-[11px] px-2 py-0.5 rounded-full">\ud83d\udc65 x${t.group_member_count}</span>
               <span class="text-green-300 text-[11px]">Group Target</span>
             </div>`
          : '';
        tt.innerHTML = `
             ${groupLine}
             <div class="flex justify-between items-center border-b border-white/10 pb-1.5 mb-1.5">
               <span class="font-bold text-white tracking-tight text-[13px]">TGT ${tid}</span>
               <span class="${t.status === 'LOCKED' || activeStates.includes(t.status) ? 'text-red-400 font-bold animate-pulse' : 'text-gray-400'} text-[10px] uppercase tracking-wider">${t.status || 'UNK'}</span>
             </div>
             <div class="grid grid-cols-2 gap-x-4 gap-y-1 text-[11px]">
               <span class="text-gray-500">State</span> <span class="text-gray-300 text-right">${t.track_state || '-'}</span>
               <span class="text-gray-500">Assign</span> <span class="text-blue-400 text-right">Drone ${t.assigned_to || 'None'}</span>
               <span class="text-gray-500">Age / Obs</span> <span class="text-gray-300 text-right">${(t.age || 0).toFixed(1)}s / ${t.observation_count || 0}</span>
               <span class="text-gray-500">Dist</span> <span class="text-gray-300 text-right">${t.distance_to_leader > 0 ? t.distance_to_leader.toFixed(1) + 'm' : '-'}</span>
             </div>
             <div class="mt-1.5 pt-1 border-t border-white/5 grid grid-cols-2 gap-x-4 gap-y-0.5 text-[11px]">
               <span class="text-gray-500">GPS</span> <span class="text-gray-300 text-right">${t.lat.toFixed(6)}, ${t.lon.toFixed(6)}</span>
             </div>
             <div class="mt-1.5 pt-1 border-t border-white/5">
               <div class="text-gray-600 text-[9px] uppercase tracking-wider mb-0.5">Local Track Mapping</div>
               <div class="text-gray-400 text-[10px]">${formatLocalIDs(t.local_ids)}</div>
             </div>
             ${isTooltipPinned && pinnedElementId === tid ? '<div class="mt-1.5 text-yellow-500 text-center text-[9px] font-bold tracking-wide">[PINNED]</div>' : '<div class="mt-1.5 text-gray-600 text-center text-[9px] hover:text-white transition-colors">CLICK TO PIN</div>'}
           `;

        // Screen boundary clamping — popup must never go off-screen
        const ttRect = tt.getBoundingClientRect();
        const vpW = window.innerWidth;
        const vpH = window.innerHeight;
        let left = e.pageX + 15;
        let top = e.pageY + 15;
        if (left + ttRect.width > vpW - 10) left = e.pageX - ttRect.width - 15;
        if (top + ttRect.height > vpH - 10) top = e.pageY - ttRect.height - 15;
        if (left < 10) left = 10;
        if (top < 10) top = 10;
        tt.style.left = left + 'px';
        tt.style.top = top + 'px';
      };

      el.onmouseenter = (e) => {
        if (isTooltipPinned && pinnedElementId !== tid) return;
        showTooltip(e);
      };
      el.onmousemove = (e) => {
        if (isTooltipPinned) return;
        const tt = document.getElementById('radarTooltip');
        const ttRect = tt.getBoundingClientRect();
        const vpW = window.innerWidth;
        const vpH = window.innerHeight;
        let left = e.pageX + 15;
        let top = e.pageY + 15;
        if (left + ttRect.width > vpW - 10) left = e.pageX - ttRect.width - 15;
        if (top + ttRect.height > vpH - 10) top = e.pageY - ttRect.height - 15;
        if (left < 10) left = 10;
        if (top < 10) top = 10;
        tt.style.left = left + 'px';
        tt.style.top = top + 'px';
      }
      el.onmouseleave = () => {
        if (!isTooltipPinned) document.getElementById('radarTooltip').classList.add('hidden');
      };

      // Click to Pin
      el.onclick = (e) => {
        e.stopPropagation();
        if (isTooltipPinned && pinnedElementId === tid) {
          isTooltipPinned = false;
          pinnedElementId = null;
          document.getElementById('radarTooltip').classList.add('hidden');
        } else {
          isTooltipPinned = true;
          pinnedElementId = tid;
          showTooltip(e);
        }
      };

      fragment.appendChild(el);
    });
  }

  // 5. Draw Drones
  if (data.drones) {
    Object.entries(data.drones).forEach(([port, d]) => {
      const pos = toScreen(d.lat, d.lon);
      const el = document.createElement('div');

      let color = '#3B82F6'; // Follower Blue
      let zIndex = "4";
      let scale = "scale(1)";

      if (d.role === 'LEADER') {
        color = '#2563EB'; // Leader Dark Blue
        zIndex = "5";
        scale = "scale(1.3)";
      }
      if (d.status === 'ATTACK' || d.action === 'ENGAGING') {
        color = '#EF4444'; // Red
      }

      el.className = `absolute w-0 h-0 transform -translate-x-1/2 -translate-y-1/2 cursor-pointer`;
      el.style.left = `${pos.x}px`;
      el.style.top = `${pos.y}px`;
      el.style.zIndex = zIndex;

      el.style.transform = `translate(-50%, -50%) rotate(${d.heading || 0}deg) ${scale}`;
      el.style.borderLeft = '6px solid transparent';
      el.style.borderRight = '6px solid transparent';
      el.style.borderBottom = `14px solid ${color}`;

      if (d.role === 'LEADER') {
        el.style.filter = "drop-shadow(0 0 6px #3B82F6)";
      }

      const showTooltip = (e) => {
        const tt = document.getElementById('radarTooltip');
        tt.classList.remove('hidden');
        tt.style.justifyContent = 'start';

        const statusColor = (d.status === 'ATTACK' || d.action === 'ENGAGING')
          ? 'text-red-400 font-bold animate-pulse'
          : d.role === 'LEADER' ? 'text-blue-400 font-bold' : 'text-blue-200';

        tt.innerHTML = `
             <div class="flex justify-between items-center border-b border-white/10 pb-1.5 mb-1.5">
               <span class="font-bold text-white tracking-tight text-[13px]">🛩 DRONE ${port}</span>
               <span class="${statusColor} text-[10px] uppercase tracking-wider">${d.role || 'UNK'}</span>
             </div>
             <div class="grid grid-cols-2 gap-x-4 gap-y-1 text-[11px]">
               <span class="text-gray-500">Action</span> <span class="text-gray-300 text-right font-medium">${d.action || 'IDLE'}</span>
               <span class="text-gray-500">Lock</span> <span class="text-gray-300 text-right">${d.lock_status || '-'}</span>
               <span class="text-gray-500">Target</span> <span class="text-red-400 text-right font-bold">${d.engaged_target_id || '-'}</span>
               <span class="text-gray-500">Alt</span> <span class="text-gray-300 text-right">${(d.alt || 0).toFixed(1)}m</span>
               <span class="text-gray-500">Heading</span> <span class="text-gray-300 text-right">${(d.heading || 0).toFixed(0)}°</span>
             </div>
             <div class="mt-1.5 pt-1 border-t border-white/5 grid grid-cols-2 gap-x-4 gap-y-0.5 text-[11px]">
               <span class="text-gray-500">GPS</span> <span class="text-gray-300 text-right">${d.lat.toFixed(6)}, ${d.lon.toFixed(6)}</span>
             </div>
             ${isTooltipPinned && pinnedElementId === port ? '<div class="mt-1.5 text-yellow-500 text-center text-[9px] font-bold tracking-wide">[PINNED]</div>' : '<div class="mt-1.5 text-gray-600 text-center text-[9px] hover:text-white transition-colors">CLICK TO FOLLOW</div>'}
           `;

        // Screen boundary clamping
        const ttRect = tt.getBoundingClientRect();
        const vpW = window.innerWidth;
        const vpH = window.innerHeight;
        let left = e.pageX + 15;
        let top = e.pageY + 15;
        if (left + ttRect.width > vpW - 10) left = e.pageX - ttRect.width - 15;
        if (top + ttRect.height > vpH - 10) top = e.pageY - ttRect.height - 15;
        if (left < 10) left = 10;
        if (top < 10) top = 10;
        tt.style.left = left + 'px';
        tt.style.top = top + 'px';
      };

      el.onmouseenter = (e) => {
        if (isTooltipPinned && pinnedElementId !== port) return;
        showTooltip(e);
      };
      el.onmousemove = (e) => {
        if (!isTooltipPinned) {
          const tt = document.getElementById('radarTooltip');
          const ttRect = tt.getBoundingClientRect();
          const vpW = window.innerWidth;
          const vpH = window.innerHeight;
          let left = e.pageX + 15;
          let top = e.pageY + 15;
          if (left + ttRect.width > vpW - 10) left = e.pageX - ttRect.width - 15;
          if (top + ttRect.height > vpH - 10) top = e.pageY - ttRect.height - 15;
          if (left < 10) left = 10;
          if (top < 10) top = 10;
          tt.style.left = left + 'px';
          tt.style.top = top + 'px';
        }
      };
      el.onmouseleave = () => {
        if (!isTooltipPinned) document.getElementById('radarTooltip').classList.add('hidden');
      };

      el.onclick = (e) => {
        e.stopPropagation();
        followTarget = port;
        setRadarMode('drone');
        showToast(`Following Drone ${port}`, "info");

        if (isTooltipPinned && pinnedElementId === port) {
          isTooltipPinned = false; pinnedElementId = null;
        } else {
          isTooltipPinned = true; pinnedElementId = port;
          showTooltip(e);
        }
      };

      fragment.appendChild(el);
    });
  }

  radarLayer.replaceChildren(fragment);
  const staticIcon = document.getElementById('radarCenterIcon');
  if (staticIcon) staticIcon.style.display = 'none';
}

/* -------------------------------------------
    Toast helper
    -------------------------------------------*/
function showToast(text, type = "info", timeout = 3000) {
  const cont = document.getElementById('toastContainer');
  const el = document.createElement('div');
  el.className = 'toast toast-' + type;
  el.style.position = 'relative';
  
  // Icon mapping
  const icons = {
    success: 'ri-check-line',
    error: 'ri-close-line',
    info: 'ri-information-line',
    warning: 'ri-alert-line'
  };
  
  el.innerHTML = `
    <div class="toast-icon"><i class="${icons[type] || icons.info}"></i></div>
    <div class="toast-content">${text}</div>
    <button class="toast-close" onclick="this.parentElement.remove()"><i class="ri-close-line"></i></button>
    <div class="toast-progress" style="animation-duration: ${timeout}ms"></div>
  `;
  
  cont.appendChild(el);
  setTimeout(() => { 
    el.style.opacity = 0; 
    el.style.transform = 'translateX(20px)';
    setTimeout(() => el.remove(), 250); 
  }, timeout);
}

/* -------------------------------------------
    Init map + draw control + event handlers
    -------------------------------------------*/
function initMap(lat = 39.925533, lon = 32.864353, zoom = 15) {
  map = L.map('map', { zoomControl: true, preferCanvas: true }).setView([lat, lon], zoom);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '© OpenStreetMap contributors'
  }).addTo(map);

  // layers
  gridLayer = L.layerGroup().addTo(map);
  pathLayer = L.layerGroup().addTo(map);

  // drawn items
  drawnItems = new L.FeatureGroup().addTo(map);
  drawControl = new L.Control.Draw({
    edit: { featureGroup: drawnItems, edit: true, remove: true },
    draw: {
      polygon: false, polyline: false, circle: false, marker: false, circlemarker: false,
      rectangle: {
        showArea: true,
        shapeOptions: { color: '#007AFF', fillOpacity: 0.12, weight: 2 }
      }
    }
  });
  map.addControl(drawControl);

  // events
  map.on(L.Draw.Event.DRAWSTART, () => { isDrawing = true; });
  map.on(L.Draw.Event.DRAWSTOP, () => { isDrawing = false; });
  map.on(L.Draw.Event.CREATED, (e) => {
    isDrawing = false;
    drawnItems.clearLayers();
    drawnItems.addLayer(e.layer);
    if (e.layerType === 'rectangle') {
      const bounds = e.layer.getBounds();
      createGrid(bounds);
    }
    updateButtonStates();
  });

  // Add zoom event listener for grid stability
  map.on('zoomend', function () {
    if (gridCells.length > 0) {
      gridLayer.clearLayers();
      gridCells.forEach(c => c.rect.addTo(gridLayer));
    }
  });

  // initial small UI
  updateButtonStates();
}


/* -------------------------------------------
    Grid creation: divides bounds into fixed-size cells
    -------------------------------------------*/
function createGrid(bounds, minCellSizeMeters = 30) {
  gridLayer.clearLayers();
  gridCells = [];

  const north = bounds.getNorth();
  const south = bounds.getSouth();
  const east = bounds.getEast();
  const west = bounds.getWest();

  const latDistance = bounds.getNorthEast().distanceTo(bounds.getSouthEast());
  const lonDistance = bounds.getNorthEast().distanceTo(bounds.getNorthWest());

  const cellsY = Math.max(1, Math.floor(latDistance / minCellSizeMeters));
  const cellsX = Math.max(1, Math.floor(lonDistance / minCellSizeMeters));

  const latStep = (north - south) / cellsY;
  const lonStep = (east - west) / cellsX;

  // Array to store cell centers
  const cellCenters = [];

  // create rectangles
  for (let i = 0; i < cellsY; i++) {
    for (let j = 0; j < cellsX; j++) {
      const cellNorth = north - (i * latStep);
      const cellSouth = north - ((i + 1) * latStep);
      const cellWest = west + (j * lonStep);
      const cellEast = west + ((j + 1) * lonStep);
      const cellBounds = [[cellNorth, cellWest], [cellSouth, cellEast]];

      // Cell Center
      const centerLat = (cellNorth + cellSouth) / 2;
      const centerLon = (cellWest + cellEast) / 2;
      cellCenters.push([centerLat, centerLon]);

      const rect = L.rectangle(cellBounds, {
        className: 'grid-cell grid-cell-unvisited',
        weight: 0.5,
        fillOpacity: 0.35,
        interactive: false
      });
      rect.addTo(gridLayer);
      gridCells.push({
        rect,
        bounds: cellBounds,
        center: [centerLat, centerLon],
        i,
        j,
        status: 'unvisited'
      });
    }
  }

  // Send cell centers to backend (for all drones)
  setMissionArea(cellCenters);

  // Route Visualization
  pathLayer.clearLayers();
  if (cellCenters.length > 1) {
    const tempPath = L.polyline(cellCenters, {
      color: '#FF0000',
      weight: 2,
      dashArray: '5, 10'
    }).addTo(pathLayer);

    setTimeout(() => {
      pathLayer.removeLayer(tempPath);
    }, 1000);
  }

  showToast(`Grid of ${cellsX}x${cellsY} created for all drones.`, "success");
}

/* -------------------------------------------
    Drawing controls (button handlers)
    -------------------------------------------*/
function enableRectangleDrawing() {
  if (connectedDrones.length === 0) {
    showToast("Connect a drone first", "error");
    return;
  }
  new L.Draw.Rectangle(map, drawControl.options.draw.rectangle).enable();
  showToast("Draw an area on the map for all drones.", "info");
}

function clearDrawing() {
  drawnItems.clearLayers();
  gridLayer.clearLayers();
  pathLayer.clearLayers();
  gridCells = [];
  updateButtonStates();
  showToast("Drawings cleared.", "info");
}


/* -------------------------------------------
    Backend / API calls (compatible with app.py)
    -------------------------------------------*/
async function connectToDrone() {
  const conn = document.getElementById('connectionString').value.trim();
  if (!conn) { showToast("Enter connection address.", "error"); return; }
  try {
    const res = await fetch('/connect_drone', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ connection_string: conn }) });
    const data = await res.json();
    showToast(data.message || "Connection request sent.", data.status === 'ok' ? 'success' : 'error');
    if (data.status === 'ok') updateDroneList();
  } catch (err) { showToast("Connection error: " + err, "error"); }
}

async function updateDroneList() {
  try {
    const res = await fetch('/status');
    const data = await res.json();

    // Update connected drone count
    const droneCount = data.connected_drones ? data.connected_drones.length : 0;
    document.getElementById('connectedDroneCount').textContent = droneCount;

    // Store drone list
    connectedDrones = data.connected_drones || [];

  } catch (err) {
    // Silently handle drone list fetch errors
  }
}

async function setMissionArea(coords) {
  try {
    const res = await fetch('/set_area', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ coordinates: coords })
    });
    const data = await res.json();
    showToast(data.message || "Area saved.", data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) {
    showToast("Failed to save area: " + err, "error");
  }
}

async function startMission() {
  if (connectedDrones.length === 0) {
    showToast("No connected drones", "error");
    return;
  }

  try {
    const res = await fetch('/start_mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mission_type: 'collision' }) // Hardcoded default
    });
    const data = await res.json();
    showToast(data.message, data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) {
    showToast("Start mission error: " + err, "error");
  }
}

async function pauseMission() {
  try {
    const res = await fetch('/pause_all', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    });
    const data = await res.json();
    showToast(data.message, data.status === 'ok' ? 'success' : 'info');
  } catch (err) {
    showToast("Pause error: " + err, "error");
  }
}

async function stopMission() {
  try {
    const res = await fetch('/stop_all', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' }
    });
    const data = await res.json();
    showToast(data.message, data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) {
    showToast("Stop error: " + err, "error");
  }
}

// Engage Button Handler
document.getElementById('engageTarget')?.addEventListener('click', async () => {
  if (confirm("Start Attack? (All PENDING targets will be approved)")) {
    try {
      const res = await fetch('/approve_all', { method: 'POST' });
      const data = await res.json();
      showToast(data.message, data.status === 'ok' ? 'success' : 'info');
    } catch (e) {
      showToast("Error: " + e, "error");
    }
  }
});

document.addEventListener('DOMContentLoaded', () => {
  initMap();
  if (typeof initCameraDrag === 'function') initCameraDrag();

  updateStatus();
  setInterval(updateStatus, 1000); // 1 sec period

  // Camera button listener
  document.getElementById('cameraToggle').addEventListener('click', toggleCamera);

  // Drawing
  document.getElementById('drawRectangle').addEventListener('click', enableRectangleDrawing);
  document.getElementById('clearDrawing').addEventListener('click', clearDrawing);

  // Mission
  document.getElementById('startMission').addEventListener('click', startMission);
  document.getElementById('pauseMission').addEventListener('click', pauseMission);
  document.getElementById('stopMission').addEventListener('click', stopMission);



  // Connect button
  document.getElementById('connectButton').addEventListener('click', connectToDrone);

  // BUG 2 FIX: Stop Propagation for Header Controls to prevent Collapse
  document.querySelectorAll('.header button, .header select, .header input').forEach(el => {
    el.addEventListener('click', (e) => {
      e.stopPropagation();
    });
  });

  // Bug 3 FIX: Click elsewhere unpins tooltip
  document.addEventListener('click', (e) => {
    // If clicking map or empty space, unpin
    if (isTooltipPinned) {
      isTooltipPinned = false;
      pinnedElementId = null;
      document.getElementById('radarTooltip').classList.add('hidden');
    }
  });
  // R24: Duplicate listeners removed
});

async function updateStatus() {
  if (isDrawing) return;
  if (_statusFetchInProgress) return; // Prevent overlapping fetches

  // Cancel any stale in-flight requests
  if (_statusAbortController) _statusAbortController.abort();
  _statusAbortController = new AbortController();
  const signal = _statusAbortController.signal;

  _statusFetchInProgress = true;
  try {
    // R25: Parallel fetch for status and swarm data
    const [res, swarmRes] = await Promise.all([
      fetch('/status', { signal }),
      fetch('/swarm_data', { signal })
    ]);
    const data = await res.json();
    const swarmData = await swarmRes.json();

    // Update connected drone count
    const droneCount = data.connected_drones ? data.connected_drones.length : 0;
    document.getElementById('connectedDroneCount').textContent = droneCount;

    // Update drone list
    connectedDrones = data.connected_drones || [];

    // Map and Marker Updates
    if (data.all_drones) {
      updateDronesStatus(data.all_drones);
      updateDroneMarkersOnMap(data.all_drones);
    }

    // Radar Drawing
    drawRadar(swarmData);

    // Selected Drone Highlight (UI)
    updateSelectedDroneUI();

    // Mission Status Text update (show messages for all connected drones)
    let msgSet = new Set();
    if (data.all_drones) {
      for (const port in data.all_drones) {
        const drone = data.all_drones[port];
        let droneMsg = null;
        // Check for active status messages
        if (drone.mission_status && drone.mission_status !== "IDLE") {
          droneMsg = drone.mission_status;
        } else if (drone.status_message && drone.status_message !== "IDLE") {
          droneMsg = drone.status_message;
        }

        if (droneMsg) {
          msgSet.add(droneMsg);
        }
      }
    }

    let msgParts = Array.from(msgSet);
    let msg = msgParts.length > 0 ? msgParts.join(' | ') : "Mission: IDLE";
    const statusBox = document.getElementById('missionStatusText');
    if (statusBox) statusBox.innerText = msg;

    // Initial focus
    if (!initialCenterSet && data.all_drones) {
      const firstDrone = Object.values(data.all_drones)[0];
      if (firstDrone && firstDrone.location && firstDrone.location.lat) {
        map.setView([firstDrone.location.lat, firstDrone.location.lon], 17);
        initialCenterSet = true;
      }
    }

  } catch (err) {
    if (err.name !== 'AbortError') {
      // Silently handle status update errors (expected during drawing)
    }
  } finally {
    _statusFetchInProgress = false;
  }
}

function updateSelectedDroneUI() {
  // Highlight selected drone card
  document.querySelectorAll('[data-port]').forEach(el => {
    if (el.dataset.port == currentCameraPort) {
      el.classList.remove('bg-[#0f1316]', 'border-[#27292b]', 'bg-yellow-900/30', 'border-yellow-700', 'bg-red-900/30', 'border-red-700');
      el.classList.add('border-blue-500', 'bg-blue-500/10');
    } else {
      el.classList.remove('border-blue-500', 'bg-blue-500/10');
    }
  });

  document.getElementById('selectedDronePort').textContent = `Port: ${currentCameraPort || '-'}`;
}

function updateDronesStatus(allDrones) {
  const container = document.getElementById('dronesStatusContainer');
  container.innerHTML = '';

  const ports = Object.keys(allDrones);

  if (ports.length === 0) {
    container.innerHTML = '<div class="text-xs text-white/40 text-center py-4">No drones connected</div>';
    return;
  }

  ports.forEach(port => {
    const drone = allDrones[port];
    const droneCard = document.createElement('div');

    // Background color based on connection status
    let bgColor = 'bg-[#0f1316]';
    let borderColor = 'border-[#27292b]';

    if (drone.status === 'connecting') {
      bgColor = 'bg-yellow-900/30';
      borderColor = 'border-yellow-700';
    } else if (drone.status === 'connected') {
      // Check telemetry data
      const hasRecentData = drone.battery_level > 0 || drone.altitude > 0;
      if (!hasRecentData) {
        // Communication might be lost
        bgColor = 'bg-red-900/30';
        borderColor = 'border-red-700';
      }
    }

    // Direct selection class to avoid CSS transition flicker
    if (parseInt(port) === currentCameraPort) {
      bgColor = 'bg-blue-500/10';
      borderColor = 'border-blue-500';
    }

    droneCard.className = `p-3 rounded-lg ${bgColor} border ${borderColor} cursor-pointer hover:bg-opacity-80 transition-all`;
    droneCard.setAttribute('data-port', port);

    const statusColor = drone.is_mission_active ? 'text-green-400' : 'text-gray-400';
    const missionIcon = drone.is_mission_active ? 'ri-flight-takeoff-fill' : 'ri-flight-land-fill';

    droneCard.innerHTML = `
      <div class="flex items-center justify-between mb-2">
        <div class="flex items-center gap-2">
          <i class="${missionIcon} ${statusColor}"></i>
          <span class="text-sm font-semibold text-white">Port ${port}</span>
        </div>
        <span class="text-xs px-2 py-1 rounded ${drone.is_mission_active ? 'bg-green-900 text-green-300' : 'bg-gray-700 text-gray-300'}">
          ${drone.is_mission_active ? 'Active' : 'Pending'}
        </span>
      </div>
      <div class="grid grid-cols-3 gap-2 text-xs">
        <div class="text-center">
          <div class="text-white/60">Battery</div>
          <div class="font-medium text-green-400">${drone.battery_level || 0}%</div>
        </div>
        <div class="text-center">
          <div class="text-white/60">Altitude</div>
          <div class="font-medium">${(drone.altitude || 0).toFixed(1)}m</div>
        </div>
        <div class="text-center">
          <div class="text-white/60">Mode</div>
          <div class="font-medium">${drone.mode || 'N/A'}</div>
        </div>
      </div>
      </div>
    `;

    // Select camera on card click
    droneCard.addEventListener('click', () => {
      selectDroneCamera(parseInt(port));
    });

    container.appendChild(droneCard);
  });
}

function updateDroneMarkersOnMap(allDrones) {
  const ports = Object.keys(allDrones);

  ports.forEach(port => {
    const drone = allDrones[port];

    if (drone.location && drone.location.lat && drone.location.lon) {
      const lat = drone.location.lat;
      const lon = drone.location.lon;

      // Create or update Marker
      if (!droneMarkers[port]) {
        const markerColor = drone.is_mission_active ? '#30D158' : '#007AFF';
        const droneIcon = L.divIcon({
          className: 'drone-marker',
          html: `<div style="background:${markerColor};width:14px;height:14px;border-radius:50%;border:2px solid white;"></div>`,
          iconSize: [18, 18],
          iconAnchor: [9, 9]
        });
        droneMarkers[port] = L.marker([lat, lon], { icon: droneIcon }).addTo(map);
        droneMarkers[port].bindTooltip(`Drone ${port}`, { permanent: false, direction: 'top' });
      } else {
        droneMarkers[port].setLatLng([lat, lon]);

        // Update Color
        const markerColor = drone.is_mission_active ? '#30D158' : '#007AFF';
        const droneIcon = L.divIcon({
          className: 'drone-marker',
          html: `<div style="background:${markerColor};width:14px;height:14px;border-radius:50%;border:2px solid white;"></div>`,
          iconSize: [18, 18],
          iconAnchor: [9, 9]
        });
        droneMarkers[port].setIcon(droneIcon);
      }

      // Path Drawing
      if (!dronePaths[port]) {
        const pathColor = drone.is_mission_active ? '#30D158' : '#007AFF';
        dronePaths[port] = L.polyline([], { color: pathColor, weight: 2, opacity: 0.6 }).addTo(map);
      }

      const lastPoints = dronePaths[port].getLatLngs();
      const isNewPoint = !lastPoints.length ||
        lastPoints[lastPoints.length - 1].lat.toFixed(6) !== lat.toFixed(6) ||
        lastPoints[lastPoints.length - 1].lng.toFixed(6) !== lon.toFixed(6);

      if (isNewPoint) {
        dronePaths[port].addLatLng([lat, lon]);
        // R26: Cap polyline to prevent memory leak
        const MAX_PATH_POINTS = 500;
        const pts = dronePaths[port].getLatLngs();
        if (pts.length > MAX_PATH_POINTS) {
          dronePaths[port].setLatLngs(pts.slice(-MAX_PATH_POINTS));
        }
      }
    }
  });

  // Remove disconnected markers
  Object.keys(droneMarkers).forEach(port => {
    if (!ports.includes(port)) {
      map.removeLayer(droneMarkers[port]);
      delete droneMarkers[port];
      if (dronePaths[port]) {
        map.removeLayer(dronePaths[port]);
        delete dronePaths[port];
      }
    }
  });
}

/* -------------------------------------------
    Camera selection - From Drone Card (Debounced)
    -------------------------------------------*/
async function selectDroneCamera(port) {
  // Already selected?
  if (currentCameraPort === port) {
    return;
  }

  // In progress?
  if (cameraSelectionInProgress) {
    return;
  }

  cameraSelectionInProgress = true;

  // Stop current camera update
  if (cameraUpdateTimeout) {
    clearTimeout(cameraUpdateTimeout);
    cameraUpdateTimeout = null;
  }

  try {
    const res = await fetch('/select_camera', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ port: parseInt(port) })
    });
    const data = await res.json();

    if (data.status === 'ok') {
      // Clear previous port
      const oldPort = currentCameraPort;
      currentCameraPort = parseInt(port);
      document.getElementById('selectedDronePort').textContent = `Port: ${port}`;

      // Reset image and update if camera is on
      if (cameraOn) {
        const img = document.getElementById('cameraFeed');
        // Port changed, force update src
        img.src = `/camera_feed?port=${currentCameraPort}&ts=${Date.now()}`;
        // Start new update loop
        cameraUpdateTimeout = setTimeout(updateCameraFeed, 100);
      }

      showToast(`Drone ${port} camera selected`, 'success', 2000);
    } else {
      showToast(data.message, 'error');
    }
  } catch (err) {
    showToast('Failed to select camera', 'error');
  } finally {
    // Unlock after 500ms
    setTimeout(() => {
      cameraSelectionInProgress = false;
    }, 500);
  }
}

/* -------------------------------------------
    Camera toggle and feed update
    -------------------------------------------*/
function toggleCamera() {
  cameraOn = !cameraOn;
  const overlay = document.getElementById('cameraOverlay');
  const circ = document.getElementById('cameraToggleCircle');
  const btn = document.getElementById('cameraToggle');
  const feedImg = document.getElementById('cameraFeed');

  if (cameraOn) {
    circ.style.transform = 'translateX(28px)';
    btn.classList.add('bg-green-600');
    // Camera ON, start feed
    if (currentCameraPort) {
      feedImg.src = `/camera_feed?port=${currentCameraPort}&ts=${Date.now()}`;
      cameraUpdateTimeout = setTimeout(updateCameraFeed, 100);
    }
  } else {
    circ.style.transform = 'translateX(4px)';
    btn.classList.remove('bg-green-600');
    // Camera OFF, clear timeout
    if (cameraUpdateTimeout) {
      clearTimeout(cameraUpdateTimeout);
      cameraUpdateTimeout = null;
    }
    feedImg.src = '/static/images/logo.png';
  }
}

function updateCameraFeed() {
  if (!cameraOn || !currentCameraPort) {
    if (cameraUpdateTimeout) {
      clearTimeout(cameraUpdateTimeout);
      cameraUpdateTimeout = null;
    }
    return;
  }

  const img = document.getElementById('cameraFeed');
  const newSrc = `/camera_feed?port=${currentCameraPort}&ts=${Date.now()}`;

  // Change src only if port is different
  if (!img.src.includes(`port=${currentCameraPort}`)) {
    img.src = newSrc;
  }

  cameraUpdateTimeout = setTimeout(updateCameraFeed, 100);  // 10 FPS
}

/* -------------------------------------------
    Button states / UI helpers
    -------------------------------------------*/
function updateButtonStates() {
  const hasConnectedDrones = connectedDrones.length > 0;
  document.getElementById('startMission').disabled = !hasConnectedDrones;
  document.getElementById('stopMission').disabled = !hasConnectedDrones;
  document.getElementById('pauseMission').disabled = !hasConnectedDrones;
}

/* -------------------------------------------
    Camera overlay drag (handle)
    -------------------------------------------*/
function initCameraDrag() {
  const overlay = document.getElementById('cameraOverlay');
  const handle = overlay.querySelector('.draggable-handle');
  let dragging = false, offsetX = 0, offsetY = 0;

  handle.addEventListener('mousedown', (e) => {
    dragging = true;
    const rect = overlay.getBoundingClientRect();
    offsetX = e.clientX - rect.left;
    offsetY = e.clientY - rect.top;
    overlay.style.transition = 'none';
    handle.style.cursor = 'grabbing';
    e.preventDefault();
  });
  document.addEventListener('mousemove', (e) => {
    if (!dragging) return;
    const mapRect = document.getElementById('map').getBoundingClientRect();
    let left = e.clientX - offsetX;
    let top = e.clientY - offsetY;
    left = Math.max(mapRect.left + 8, Math.min(left, mapRect.right - overlay.offsetWidth - 8));
    top = Math.max(mapRect.top + 8, Math.min(top, mapRect.bottom - overlay.offsetHeight - 8));
    overlay.style.left = left + 'px';
    overlay.style.top = top + 'px';
    overlay.style.right = 'unset';
    overlay.style.bottom = 'unset';
  });
  document.addEventListener('mouseup', () => {
    if (dragging) { dragging = false; overlay.style.transition = 'all .12s ease'; handle.style.cursor = 'grab'; }
  });

  handle.addEventListener('touchstart', (e) => {
    dragging = true;
    const t = e.touches[0];
    const rect = overlay.getBoundingClientRect();
    offsetX = t.clientX - rect.left; offsetY = t.clientY - rect.top;
    overlay.style.transition = 'none';
  });
  document.addEventListener('touchmove', (e) => {
    if (!dragging) return;
    const t = e.touches[0];
    const mapRect = document.getElementById('map').getBoundingClientRect();
    let left = t.clientX - offsetX;
    let top = t.clientY - offsetY;
    left = Math.max(mapRect.left + 8, Math.min(left, mapRect.right - overlay.offsetWidth - 8));
    top = Math.max(mapRect.top + 8, Math.min(top, mapRect.bottom - overlay.offsetHeight - 8));
    overlay.style.left = left + 'px';
    overlay.style.top = top + 'px';
    overlay.style.right = 'unset';
    overlay.style.bottom = 'unset';
  });
  document.addEventListener('touchend', () => { if (dragging) { dragging = false; overlay.style.transition = 'all .12s ease'; } });
}


