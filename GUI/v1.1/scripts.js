
let mapOptions = {
    center:[42.407, -71.1206],
    zoom:18
}

let map = new L.map('map' , mapOptions);

let layer = new L.TileLayer('http://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png');
map.addLayer(layer);

// up to 5 waypoints allowed
let markers = [];

function updateTooltipsAndFields() {
    for (let i = 0; i < markers.length; i++) {
        markers[i].unbindTooltip();
        markers[i].bindTooltip(`${i + 1}`, {permanent: true}).openTooltip();
        document.getElementById(`latitude${i + 1}`).value = `Lat#${i+1}: ` + markers[i].getLatLng().lat;
        document.getElementById(`longitude${i + 1}`).value = `Long#${i+1}: ` + markers[i].getLatLng().lng;
    }
    // reset unused lat/long fields
    for (let i = markers.length; i < 5; i++) {
        document.getElementById(`latitude${i + 1}`).value = `Latitude ${i+1}`;
        document.getElementById(`longitude${i + 1}`).value = `Longitude ${i+1}`;
    }
}

map.on('click', (event)=> {
    if (markers.length >= 5) {
        map.removeLayer(markers[0]);
        markers.shift();
    }

    let newMarker = L.marker([event.latlng.lat, event.latlng.lng]).addTo(map);
    markers.push(newMarker);

    // on right click, remove marker
    newMarker.on('contextmenu', function () {
        map.removeLayer(this);
        markers = markers.filter(marker => marker !== this);
        updateTooltipsAndFields();
    });

    updateTooltipsAndFields();
    
})
