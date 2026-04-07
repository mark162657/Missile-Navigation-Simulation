let map;
let startMarker, targetMarker;
let waypointMarkers = [];
let clickMode = 'none';
let currentDemInfo = null;
let missileConfigs = [];
let abortController = null;

// DOM Elements
const demSelect = document.getElementById('dem-select');
const configSelect = document.getElementById('config-select');
const configSummary = document.getElementById('config-summary');
const startLatInput = document.getElementById('start-lat');
const startLonInput = document.getElementById('start-lon');
const targetLatInput = document.getElementById('target-lat');
const targetLonInput = document.getElementById('target-lon');
const waypointList = document.getElementById('waypoint-list');
const planBtn = document.getElementById('plan-btn');
const stopBtn = document.getElementById('stop-btn');
const statusDiv = document.getElementById('status');
const tokenInput = document.getElementById('mapbox-token');

// Initialize app
async function init() {
    await loadInitialData();
    
    tokenInput.addEventListener('change', () => {
        if (tokenInput.value.trim()) {
            initMap(tokenInput.value.trim());
            document.getElementById('token-required').style.display = 'none';
        }
    });

    demSelect.addEventListener('change', onDemChanged);
    configSelect.addEventListener('change', updateConfigSummary);
    
    document.querySelectorAll('.mode-btn').forEach(btn => {
        btn.addEventListener('click', () => {
            document.querySelectorAll('.mode-btn').forEach(b => b.classList.remove('active'));
            btn.classList.add('active');
            clickMode = btn.dataset.mode;
        });
    });

    [startLatInput, startLonInput, targetLatInput, targetLonInput].forEach(el => {
        el.addEventListener('change', updateMarkersFromInputs);
    });

    document.getElementById('clear-waypoints').addEventListener('click', clearWaypoints);
    planBtn.addEventListener('click', runPathfinding);
    
    stopBtn.addEventListener('click', () => {
        if (abortController) {
            abortController.abort();
            statusDiv.innerText = "Pathfinding cancelled by user.";
        }
    });
}

async function loadInitialData() {
    try {
        const [demsRes, configsRes] = await Promise.all([
            fetch('/api/dems'),
            fetch('/api/missile-configs')
        ]);
        
        const dems = await demsRes.json();
        const configs = await configsRes.json();
        missileConfigs = configs.configs;

        dems.dems.forEach(dem => {
            const opt = document.createElement('option');
            opt.value = dem;
            opt.textContent = dem;
            demSelect.appendChild(opt);
        });

        missileConfigs.forEach(cfg => {
            const opt = document.createElement('option');
            opt.value = cfg.name;
            opt.textContent = cfg.name;
            configSelect.appendChild(opt);
        });

        updateConfigSummary();
        if (demSelect.value) onDemChanged();
    } catch (err) {
        statusDiv.innerHTML = `<span class="error">Data load failed: ${err.message}</span>`;
    }
}

function updateConfigSummary() {
    const cfg = missileConfigs.find(c => c.name === configSelect.value);
    if (cfg) {
        configSummary.innerText = `Cruise: ${cfg.cruise_speed}km/h | Alt: ${cfg.min_altitude}-${cfg.max_altitude}m | G: ${cfg.max_g_force}`;
    }
}

async function onDemChanged() {
    const demName = demSelect.value;
    try {
        const res = await fetch(`/api/dem-info/${demName}`);
        currentDemInfo = await res.json();
        
        if (map) {
            updateDemBoundary();
            map.flyTo({
                center: [currentDemInfo.center[1], currentDemInfo.center[0]],
                zoom: 8
            });
        }
        statusDiv.innerText = `DEM ${demName} loaded.`;
    } catch (err) {
        statusDiv.innerHTML = `<span class="error">DEM info failed: ${err.message}</span>`;
    }
}

function initMap(token) {
    mapboxgl.accessToken = token;
    map = new mapboxgl.Map({
        container: 'map',
        style: 'mapbox://styles/mapbox/satellite-v9',
        center: [95.5, 57.0],
        zoom: 4,
        pitch: 45,
        antialias: true
    });

    map.on('load', () => {
        map.addSource('mapbox-dem', {
            'type': 'raster-dem',
            'url': 'mapbox://mapbox.mapbox-terrain-dem-v1',
            'tileSize': 512,
            'maxzoom': 14
        });
        map.setTerrain({ 'source': 'mapbox-dem', 'exaggeration': 1.5 });
        map.addLayer({ 'id': 'sky', 'type': 'sky', 'paint': { 'sky-type': 'atmosphere' } });
        
        if (currentDemInfo) updateDemBoundary();
    });

    map.on('click', onMapClick);
}

function updateDemBoundary() {
    if (!map.getSource('dem-boundary')) {
        map.addSource('dem-boundary', { 'type': 'geojson', 'data': getBoundaryGeoJson() });
        map.addLayer({
            'id': 'dem-outline',
            'type': 'line',
            'source': 'dem-boundary',
            'paint': { 'line-color': '#00ff00', 'line-width': 2, 'line-dasharray': [2, 1] }
        });
    } else {
        map.getSource('dem-boundary').setData(getBoundaryGeoJson());
    }
}

function getBoundaryGeoJson() {
    const b = currentDemInfo.bounds;
    return {
        'type': 'Feature',
        'geometry': {
            'type': 'Polygon',
            'coordinates': [[[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]], [b[0], b[1]]]]
        }
    };
}

function onMapClick(e) {
    if (clickMode === 'none') return;

    const lat = e.lngLat.lat;
    const lon = e.lngLat.lng;

    if (clickMode === 'start') {
        startLatInput.value = lat.toFixed(6);
        startLonInput.value = lon.toFixed(6);
        updateMarkersFromInputs();
    } else if (clickMode === 'target') {
        targetLatInput.value = lat.toFixed(6);
        targetLonInput.value = lon.toFixed(6);
        updateMarkersFromInputs();
    } else if (clickMode === 'waypoint') {
        addWaypoint(lat, lon);
    }
    
    updateMissionLegs();
}

function updateMarkersFromInputs() {
    if (!map) return;
    
    const sLat = parseFloat(startLatInput.value);
    const sLon = parseFloat(startLonInput.value);
    const tLat = parseFloat(targetLatInput.value);
    const tLon = parseFloat(targetLonInput.value);

    if (!isNaN(sLat) && !isNaN(sLon)) {
        if (!startMarker) startMarker = new mapboxgl.Marker({ color: '#28a745' }).setLngLat([sLon, sLat]).addTo(map);
        else startMarker.setLngLat([sLon, sLat]);
    }

    if (!isNaN(tLat) && !isNaN(tLon)) {
        if (!targetMarker) targetMarker = new mapboxgl.Marker({ color: '#dc3545' }).setLngLat([tLon, tLat]).addTo(map);
        else targetMarker.setLngLat([tLon, tLat]);
    }
    
    updateMissionLegs();
}

function addWaypoint(lat, lon) {
    const wp = { lat, lon };
    const marker = new mapboxgl.Marker({ color: '#ffc107', scale: 0.8 })
        .setLngLat([lon, lat])
        .addTo(map);
    
    waypointMarkers.push({ data: wp, marker });
    refreshWaypointUI();
    updateMissionLegs();
}

function refreshWaypointUI() {
    waypointList.innerHTML = '';
    waypointMarkers.forEach((wp, idx) => {
        const div = document.createElement('div');
        div.className = 'waypoint-item';
        div.innerHTML = `<span>W${idx+1}: ${wp.data.lat.toFixed(4)}, ${wp.data.lon.toFixed(4)}</span>`;
        waypointList.appendChild(div);
    });
}

function clearWaypoints() {
    waypointMarkers.forEach(wp => wp.marker.remove());
    waypointMarkers = [];
    refreshWaypointUI();
    updateMissionLegs();
}

function updateMissionLegs() {
    if (!map || !map.isStyleLoaded()) return;

    const coords = [];
    const sLat = parseFloat(startLatInput.value);
    const sLon = parseFloat(startLonInput.value);
    if (!isNaN(sLat)) coords.push([sLon, sLat]);
    
    waypointMarkers.forEach(wp => coords.push([wp.data.lon, wp.data.lat]));
    
    const tLat = parseFloat(targetLatInput.value);
    const tLon = parseFloat(targetLonInput.value);
    if (!isNaN(tLat)) coords.push([tLon, tLat]);

    if (coords.length < 2) {
        if (map.getLayer('mission-legs')) map.removeLayer('mission-legs');
        return;
    }

    const geojson = {
        'type': 'Feature',
        'geometry': { 'type': 'LineString', 'coordinates': coords }
    };

    if (!map.getSource('mission-legs')) {
        map.addSource('mission-legs', { 'type': 'geojson', 'data': geojson });
        map.addLayer({
            'id': 'mission-legs',
            'type': 'line',
            'source': 'mission-legs',
            'paint': { 'line-color': '#8390fa', 'line-width': 2, 'line-dasharray': [2, 1] }
        });
    } else {
        map.getSource('mission-legs').setData(geojson);
    }
}

async function runPathfinding() {
    const sLat = parseFloat(startLatInput.value);
    const sLon = parseFloat(startLonInput.value);
    const tLat = parseFloat(targetLatInput.value);
    const tLon = parseFloat(targetLonInput.value);

    if (isNaN(sLat) || isNaN(tLat)) {
        statusDiv.innerHTML = '<span class="error">Start and Target required!</span>';
        return;
    }

    planBtn.disabled = true;
    stopBtn.style.display = 'block';
    statusDiv.innerText = "Calculating multi-leg path...";
    
    abortController = new AbortController();

    const body = {
        dem_name: demSelect.value,
        start: { lat: sLat, lon: sLon },
        target: { lat: tLat, lon: tLon },
        waypoints: waypointMarkers.map(m => m.data),
        heuristic_weight: 2.0
    };

    try {
        const res = await fetch('/api/plan-path', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(body),
            signal: abortController.signal
        });

        if (!res.ok) {
            const err = await res.json();
            throw new Error(err.detail || "Server error");
        }

        const data = await res.json();
        drawPath(data.path);
        statusDiv.innerText = `Success! Path contains ${data.path.length} points.`;
    } catch (err) {
        if (err.name === 'AbortError') {
            console.log("Pathfinding aborted by user");
        } else {
            console.error(err);
            statusDiv.innerHTML = `<span class="error">Error: ${err.message}</span>`;
        }
    } finally {
        planBtn.disabled = false;
        stopBtn.style.display = 'none';
        abortController = null;
    }
}

function drawPath(path) {
    const coords = path.map(p => [p[1], p[0]]); // [lon, lat]
    const geojson = {
        'type': 'Feature',
        'geometry': { 'type': 'LineString', 'coordinates': coords }
    };

    if (!map.getSource('route')) {
        map.addSource('route', { 'type': 'geojson', 'data': geojson });
        map.addLayer({
            'id': 'route',
            'type': 'line',
            'source': 'route',
            'layout': { 'line-join': 'round', 'line-cap': 'round' },
            'paint': { 'line-color': '#ffae00', 'line-width': 4 }
        });
    } else {
        map.getSource('route').setData(geojson);
    }

    const bounds = coords.reduce((b, c) => b.extend(c), new mapboxgl.LngLatBounds(coords[0], coords[0]));
    map.fitBounds(bounds, { padding: 50 });
}

init();
