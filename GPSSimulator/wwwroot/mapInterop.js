window.mapInterop = (() => {
    let map = null;
    let routeLayer = null;
    let posMarker = null;
    let startMarker = null;
    let endMarker = null;

    // Custom icon helpers
    function makeCircleIcon(color) {
        return L.divIcon({
            className: '',
            html: `<div style="width:14px;height:14px;border-radius:50%;background:${color};border:2px solid #fff;box-shadow:0 0 4px rgba(0,0,0,.5);"></div>`,
            iconSize: [14, 14],
            iconAnchor: [7, 7]
        });
    }

    function makePulseIcon(color) {
        return L.divIcon({
            className: '',
            html: `<div style="position:relative;width:18px;height:18px;">
                     <div style="position:absolute;width:18px;height:18px;border-radius:50%;background:${color};border:2px solid #fff;box-shadow:0 0 6px rgba(0,0,0,.6);animation:map-pulse 1.4s infinite;"></div>
                   </div>`,
            iconSize: [18, 18],
            iconAnchor: [9, 9]
        });
    }

    return {
        initMap(elementId, lat, lon, zoom) {
            if (map) {
                map.remove();
                map = null;
                routeLayer = null;
                posMarker = null;
                startMarker = null;
                endMarker = null;
            }

            map = L.map(elementId, { zoomControl: true }).setView([lat, lon], zoom || 14);

            L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
                maxZoom: 19,
                attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            }).addTo(map);

            // Inject pulse animation once
            if (!document.getElementById('map-pulse-style')) {
                const style = document.createElement('style');
                style.id = 'map-pulse-style';
                style.textContent = `@keyframes map-pulse { 0%,100%{transform:scale(1);opacity:1} 50%{transform:scale(1.5);opacity:.6} }`;
                document.head.appendChild(style);
            }

            posMarker = L.marker([lat, lon], { icon: makePulseIcon('#0d6efd'), zIndexOffset: 1000 })
                .addTo(map)
                .bindTooltip('Current position', { permanent: false });
        },

        setRoute(points) {
            if (!map) return;

            // Remove previous route layers
            if (routeLayer) { map.removeLayer(routeLayer); routeLayer = null; }
            if (startMarker) { map.removeLayer(startMarker); startMarker = null; }
            if (endMarker)   { map.removeLayer(endMarker);   endMarker = null; }

            if (!points || points.length === 0) return;

            const latlngs = points.map(p => [p.lat, p.lon]);

            routeLayer = L.polyline(latlngs, {
                color: '#0d6efd',
                weight: 3,
                opacity: 0.75
            }).addTo(map);

            startMarker = L.marker(latlngs[0], { icon: makeCircleIcon('#198754') })
                .addTo(map)
                .bindTooltip('Trip start', { permanent: false });

            endMarker = L.marker(latlngs[latlngs.length - 1], { icon: makeCircleIcon('#dc3545') })
                .addTo(map)
                .bindTooltip('Trip end', { permanent: false });

            // Move position marker to route start
            if (posMarker) posMarker.setLatLng(latlngs[0]);

            map.fitBounds(routeLayer.getBounds(), { padding: [24, 24] });
        },

        clearRoute() {
            if (routeLayer) { map.removeLayer(routeLayer); routeLayer = null; }
            if (startMarker) { map.removeLayer(startMarker); startMarker = null; }
            if (endMarker)   { map.removeLayer(endMarker);   endMarker = null; }
        },

        updatePosition(lat, lon) {
            if (!map || !posMarker) return;
            posMarker.setLatLng([lat, lon]);
            // Only pan if the marker drifts outside the visible area
            if (!map.getBounds().contains([lat, lon])) {
                map.panTo([lat, lon]);
            }
        },

        flyTo(lat, lon, zoom) {
            if (!map) return;
            map.flyTo([lat, lon], zoom || 14);
        },

        invalidateSize() {
            if (map) map.invalidateSize();
        }
    };
})();
