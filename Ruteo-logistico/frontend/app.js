// Configuración inicial del mapa
const map = L.map('map').setView([-34.6037, -58.3816], 12); // Centro en Buenos Aires
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);

// Variables de estado
let waypoints = [];
let markers = [];
let routeLayer = null;

// Manejadores de eventos
map.on('click', function(e) {
    addWaypoint(e.latlng);
});

document.getElementById('optimize-btn').addEventListener('click', optimizeRoute);

// Funciones principales
function addWaypoint(latlng) {
    const id = Date.now();
    waypoints.push({ id, lat: latlng.lat, lng: latlng.lng });
    
    const marker = L.marker(latlng, { draggable: true })
        .bindPopup(`Punto ${waypoints.length}`)
        .addTo(map);
    marker.id = id;
    
    marker.on('dragend', function(e) {
        const index = waypoints.findIndex(wp => wp.id === this.id);
        if (index !== -1) {
            waypoints[index].lat = e.target.getLatLng().lat;
            waypoints[index].lng = e.target.getLatLng().lng;
        }
    });
    
    markers.push(marker);
    updateWaypointList();
}

async function optimizeRoute() {
    if (waypoints.length < 2) {
        alert('Se necesitan al menos 2 puntos para optimizar una ruta');
        return;
    }
    
    // Calcula matriz de distancias con Turf.js (sin API externa)
    const distanceMatrix = waypoints.map(from => {
        return waypoints.map(to => {
            const fromPoint = turf.point([from.lng, from.lat]);
            const toPoint = turf.point([to.lng, to.lat]);
            return turf.distance(fromPoint, toPoint) * 1000; // En metros
        });
    });

    try {
        const response = await fetch('https://ruteo-logistico.onrender.com', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ 
                locations: waypoints,
                distance_matrix: distanceMatrix 
            })
        });
        
        const data = await response.json();
        if (data.status === 'success') {
            displayOptimizedRoute(data.route);
        } else {
            alert('Error al optimizar la ruta: ' + data.message);
        }
    } catch (error) {
        console.error('Error:', error);
        alert('Error al conectar con el servidor');
    }
}

function displayOptimizedRoute(optimizedRoute) {
    if (routeLayer) map.removeLayer(routeLayer);
    
    const latlngs = optimizedRoute.map(point => [point.lat, point.lng]);
    routeLayer = L.polyline(latlngs, { color: 'blue', weight: 5 }).addTo(map);
    map.fitBounds(routeLayer.getBounds());
    
    document.getElementById('route-info').style.display = 'block';
    const routeSteps = document.getElementById('route-steps');
    routeSteps.innerHTML = '';
    
    optimizedRoute.forEach((point, index) => {
        const step = document.createElement('div');
        step.className = 'mb-1';
        step.innerHTML = `<strong>${index + 1}.</strong> Lat: ${point.lat.toFixed(4)}, Lng: ${point.lng.toFixed(4)}`;
        routeSteps.appendChild(step);
    });
    
    let totalDistance = 0;
    for (let i = 0; i < latlngs.length - 1; i++) {
        totalDistance += map.distance(latlngs[i], latlngs[i + 1]);
    }
    document.getElementById('total-distance').textContent = (totalDistance / 1000).toFixed(2);
}

function updateWaypointList() {
    const list = document.getElementById('waypoint-list');
    list.innerHTML = '';
    
    waypoints.forEach((point, index) => {
        const item = document.createElement('div');
        item.className = 'd-flex justify-content-between align-items-center mb-2 p-2 border';
        item.innerHTML = `
            <span><strong>Punto ${index + 1}:</strong> ${point.lat.toFixed(4)}, ${point.lng.toFixed(4)}</span>
            <button class="btn btn-sm btn-danger" data-id="${point.id}">X</button>
        `;
        
        item.querySelector('button').addEventListener('click', function() {
            const id = parseInt(this.getAttribute('data-id'));
            removeWaypoint(id);
        });
        
        list.appendChild(item);
    });
}

function removeWaypoint(id) {
    const index = waypoints.findIndex(wp => wp.id === id);
    if (index !== -1) {
        waypoints.splice(index, 1);
        map.removeLayer(markers[index]);
        markers.splice(index, 1);
        updateWaypointList();
    }
}