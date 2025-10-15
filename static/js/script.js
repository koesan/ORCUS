/* -------------------------------------------
    Globals & state - Multi-Drone System
    -------------------------------------------*/
let map, gridLayer, pathLayer, drawnItems, drawControl;
let isDrawing = false, cameraOn = false;
let initialCenterSet = false, pathPoints = [], gridCells = [];
let selectedMissionType = "collision"; // Varsayılan görev tipi
let droneMarkers = {};  // port -> marker mapping
let dronePaths = {};    // port -> polyline mapping
let selectedAreaPort = null; // Hangi dron için alan seçildi
let connectedDrones = [];
let currentCameraPort = null;
let cameraSelectionInProgress = false; // Kamera seçim kilidi
let cameraUpdateTimeout = null; // Kamera güncelleme timeout'u

/* -------------------------------------------
    Toast helper
    -------------------------------------------*/
function showToast(text, type = "info", timeout = 3000) {
  const cont = document.getElementById('toastContainer');
  const el = document.createElement('div');
  el.className = 'toast ' + (type === 'success' ? 'toast-success' : type === 'error' ? 'toast-error' : 'toast-info');
  el.textContent = text;
  cont.appendChild(el);
  setTimeout(() => { el.style.opacity = 0; setTimeout(() => el.remove(), 300); }, timeout);
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
  map.on('zoomend', function() {
    if (gridCells.length > 0) {
      gridLayer.clearLayers();
      gridCells.forEach(c => c.rect.addTo(gridLayer));
    }
  });

  // initial small UI
  updateButtonStates();
}


/* -------------------------------------------
    Grid creation: verilen bounds'u sabit boyutta hücrelere böler
    -------------------------------------------*/
function createGrid(bounds, minCellSizeMeters = 30) {
    gridLayer.clearLayers();
    gridCells = [];
    pathPoints = [];

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

    // Hücre merkezlerini saklamak için bir dizi
    const cellCenters = [];

    // create rectangles
    for (let i = 0; i < cellsY; i++) {
        for (let j = 0; j < cellsX; j++) {
            const cellNorth = north - (i * latStep);
            const cellSouth = north - ((i + 1) * latStep);
            const cellWest = west + (j * lonStep);
            const cellEast = west + ((j + 1) * lonStep);
            const cellBounds = [[cellNorth, cellWest], [cellSouth, cellEast]];

            // Hücre merkezi
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

    // Hücre merkezlerini backend'e gönder (tüm dronlar için)
    setMissionArea(cellCenters);

    // Rota görselleştirme
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

    showToast(`Tüm dronlar için ${cellsX}x${cellsY} ızgara oluşturuldu.`, "success");
}

/* -------------------------------------------
    Drawing controls (button handlers)
    -------------------------------------------*/
function enableRectangleDrawing() {
  if (connectedDrones.length === 0) {
    showToast("Önce dron bağlayın", "error");
    return;
  }
  new L.Draw.Rectangle(map, drawControl.options.draw.rectangle).enable();
  showToast("Tüm dronlar için harita üzerinde alan çizin.", "info");
}

function clearDrawing() {
  drawnItems.clearLayers();
  gridLayer.clearLayers();
  pathLayer.clearLayers();
  gridCells = [];
  pathPoints = [];
  selectedAreaPort = null;
  updateButtonStates();
  showToast("Çizimler temizlendi.", "info");
}


/* -------------------------------------------
    Backend / API calls (app.py ile uyumlu)
    -------------------------------------------*/
async function connectToDrone() {
  const conn = document.getElementById('connectionString').value.trim();
  if (!conn) { showToast("Bağlantı adresi giriniz.", "error"); return; }
  try {
    const res = await fetch('/connect_drone', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ connection_string: conn }) });
    const data = await res.json();
    showToast(data.message || "Bağlantı isteği gönderildi.", data.status === 'ok' ? 'success' : 'error');
    if (data.status === 'ok') updateDroneList();
  } catch (err) { showToast("Bağlantı hatası: " + err, "error"); }
}

async function updateDroneList() {
  try {
    const res = await fetch('/status');
    const data = await res.json();
    
    // Bağlı dron sayısını güncelle
    const droneCount = data.connected_drones ? data.connected_drones.length : 0;
    document.getElementById('connectedDroneCount').textContent = droneCount;
    
    // Dron listesini sakla
    connectedDrones = data.connected_drones || [];
    
  } catch (err) {
    console.error("Drone listesi alınamadı:", err);
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
    showToast(data.message || "Alan kaydedildi.", data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) { 
    showToast("Alan kaydedilemedi: " + err, "error"); 
  }
}

async function startMission() {
  if (connectedDrones.length === 0) {
    showToast("Bağlı dron yok", "error");
    return;
  }
  
  if (!selectedMissionType) {
    showToast("Lütfen bir görev seçin", "error");
    return;
  }

  try {
    const res = await fetch('/start_mission', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mission_type: selectedMissionType })
    });
    const data = await res.json();
    showToast(data.message, data.status === 'ok' ? 'success' : 'error');
    updateButtonStates();
  } catch (err) {
    showToast("Görev başlatma hatası: " + err, "error");
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
    showToast("Duraklatma hatası: " + err, "error");
  }
}

async function resumeMission() {
  try {
    const res = await fetch('/resume_all', { 
      method: 'POST', 
      headers: { 'Content-Type': 'application/json' } 
    });
    const data = await res.json();
    showToast(data.message, data.status === 'ok' ? 'success' : 'info');
  } catch (err) {
    showToast("Devam ettirme hatası: " + err, "error");
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
    showToast("Durdurma hatası: " + err, "error");
  }
}

/* -------------------------------------------
    Periodic status polling -> updates map, telemetry, grid, path
    -------------------------------------------*/
async function updateStatus() {
  if (isDrawing) return;

  try {
    const res = await fetch('/status');
    if (!res.ok) throw new Error('Network response not ok');
    const data = await res.json();

    // Bağlı dron sayısı
    const droneCount = data.connected_drones ? data.connected_drones.length : 0;
    document.getElementById('connectedDroneCount').textContent = droneCount;
    
    // Dron listesini güncelle
    connectedDrones = data.connected_drones || [];

    // Tüm dronların durumunu göster
    if (data.all_drones) {
      updateDronesStatus(data.all_drones);
      updateDroneMarkersOnMap(data.all_drones);
    }

    // İlk odaklanma
    if (!initialCenterSet && data.all_drones) {
      const firstDrone = Object.values(data.all_drones)[0];
      if (firstDrone && firstDrone.location && firstDrone.location.lat) {
        map.setView([firstDrone.location.lat, firstDrone.location.lon], 17);
        initialCenterSet = true;
      }
    }

  } catch (err) {
    console.debug('Status update error:', err);
  }

  updateButtonStates();
}

function updateDronesStatus(allDrones) {
  const container = document.getElementById('dronesStatusContainer');
  container.innerHTML = '';
  
  const ports = Object.keys(allDrones);
  
  if (ports.length === 0) {
    container.innerHTML = '<div class="text-xs text-white/40 text-center py-4">Bağlı dron yok</div>';
    return;
  }
  
  ports.forEach(port => {
    const drone = allDrones[port];
    const droneCard = document.createElement('div');
    
    // Bağlantı durumuna göre arka plan rengi
    let bgColor = 'bg-[#0f1316]';
    let borderColor = 'border-[#27292b]';
    
    if (drone.status === 'connecting') {
      bgColor = 'bg-yellow-900/30';
      borderColor = 'border-yellow-700';
    } else if (drone.status === 'connected') {
      // Telemetri verisi alınıyor mu kontrol et
      const hasRecentData = drone.battery_level > 0 || drone.altitude > 0;
      if (!hasRecentData) {
        // İletişim kesilmiş olabilir
        bgColor = 'bg-red-900/30';
        borderColor = 'border-red-700';
      }
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
          ${drone.is_mission_active ? 'Aktif' : 'Beklemede'}
        </span>
      </div>
      <div class="grid grid-cols-3 gap-2 text-xs">
        <div class="text-center">
          <div class="text-white/60">Batarya</div>
          <div class="font-medium text-green-400">${drone.battery_level || 0}%</div>
        </div>
        <div class="text-center">
          <div class="text-white/60">Yükseklik</div>
          <div class="font-medium">${(drone.altitude || 0).toFixed(1)}m</div>
        </div>
        <div class="text-center">
          <div class="text-white/60">Mod</div>
          <div class="font-medium">${drone.mode || 'N/A'}</div>
        </div>
      </div>
      ${drone.is_mission_active ? `<div class="mt-2 text-xs text-white/70 truncate">${drone.mission_status || ''}</div>` : ''}
    `;
    
    // Karta tıklandığında kamerayı değiştir
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
      
      // Marker oluştur veya güncelle
      if (!droneMarkers[port]) {
        const markerColor = drone.is_mission_active ? '#30D158' : '#007AFF';
        const droneIcon = L.divIcon({ 
          className: 'drone-marker', 
          html: `<div style="background:${markerColor};width:14px;height:14px;border-radius:50%;border:2px solid white;"></div>`,
          iconSize: [18, 18], 
          iconAnchor: [9, 9] 
        });
        droneMarkers[port] = L.marker([lat, lon], { icon: droneIcon }).addTo(map);
        droneMarkers[port].bindTooltip(`Dron ${port}`, { permanent: false, direction: 'top' });
      } else {
        droneMarkers[port].setLatLng([lat, lon]);
        
        // Renk güncelle
        const markerColor = drone.is_mission_active ? '#30D158' : '#007AFF';
        const droneIcon = L.divIcon({ 
          className: 'drone-marker', 
          html: `<div style="background:${markerColor};width:14px;height:14px;border-radius:50%;border:2px solid white;"></div>`,
          iconSize: [18, 18], 
          iconAnchor: [9, 9] 
        });
        droneMarkers[port].setIcon(droneIcon);
      }
      
      // Yol çizimi
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
      }
    }
  });
  
  // Bağlantısı kesilen dronların marker'larını kaldır
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
    Kamera seçimi - Dron kartından (Debounced)
    -------------------------------------------*/
async function selectDroneCamera(port) {
  // Zaten seçili mi?
  if (currentCameraPort === port) {
    return;
  }
  
  // İşlem devam ediyor mu?
  if (cameraSelectionInProgress) {
    console.log('Kamera seçimi zaten devam ediyor, atlanıyor...');
    return;
  }
  
  cameraSelectionInProgress = true;
  
  // Mevcut kamera update'ini durdur
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
      // Önceki port'u temizle
      const oldPort = currentCameraPort;
      currentCameraPort = parseInt(port);
      document.getElementById('selectedDronePort').textContent = `Port: ${port}`;
      
      // Kamera açıksa görüntüyü resetle ve güncelle
      if (cameraOn) {
        const img = document.getElementById('cameraFeed');
        // Port değişti, src'yi zorla güncelle
        img.src = `/camera_feed?port=${currentCameraPort}&ts=${Date.now()}`;
        // Yeni update döngüsünü başlat
        cameraUpdateTimeout = setTimeout(updateCameraFeed, 100);
      }
      
      showToast(`Dron ${port} kamerası seçildi`, 'success', 2000);
    } else {
      showToast(data.message, 'error');
    }
  } catch (err) {
    console.error('Kamera seçim hatası:', err);
    showToast('Kamera seçilemedi', 'error');
  } finally {
    // Kilidi 500ms sonra aç
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
    // Kamera açıldı, feed'i başlat
    if (currentCameraPort) {
      feedImg.src = `/camera_feed?port=${currentCameraPort}&ts=${Date.now()}`;
      cameraUpdateTimeout = setTimeout(updateCameraFeed, 100);
    }
  } else {
    circ.style.transform = 'translateX(4px)';
    btn.classList.remove('bg-green-600');
    // Kamera kapatıldı, timeout'u temizle
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
  
  // Sadece farklı port ise src'yi değiştir
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

/* -------------------------------------------
    UI event bindings
    -------------------------------------------*/
document.addEventListener('DOMContentLoaded', function () {
  initMap();
  initCameraDrag();

  // Buttons
  document.getElementById('connectButton').addEventListener('click', connectToDrone);
  document.getElementById('drawRectangle').addEventListener('click', enableRectangleDrawing);
  document.getElementById('clearDrawing').addEventListener('click', clearDrawing);
  document.getElementById('startMission').addEventListener('click', startMission);
  document.getElementById('stopMission').addEventListener('click', stopMission);
  document.getElementById('pauseMission').addEventListener('click', pauseMission);

  document.getElementById('cameraToggle').addEventListener('click', toggleCamera);

  // Görev seçimi açılır menüsü
  document.getElementById('missionSelector').addEventListener('click', () => {
    const md = document.getElementById('missionDropdown');
    md.classList.toggle('hidden');
  });

  document.querySelectorAll('#missionDropdown div').forEach(item => {
    item.addEventListener('click', function() {
      selectedMissionType = this.getAttribute('data-value');
      document.getElementById('selectedMissionText').textContent = this.textContent;
      document.getElementById('missionDropdown').classList.add('hidden');
    });
  });
  
  // Varsayılan görev seçimi
  selectedMissionType = 'collision';
  document.getElementById('selectedMissionText').textContent = 'Çarpma Görevi';

  // periodic status
  setInterval(updateStatus, 2000);
  updateStatus();
});
