angular.module('appControllers').controller('GPSCtrl', GPSCtrl);
GPSCtrl.$inject = ['$rootScope', '$scope', '$state', '$http', '$interval', '$timeout'];

// ── Constellation helpers (module-level, shared with canvas functions) ─────────
// Prefixes match Stratux SatelliteID format produced in gnss.go:
// G=GPS  R=GLONASS  E=Galileo  B=BeiDou  S=SBAS  Q=QZSS  U=Unknown
// NACp → horizontal accuracy bound per AC 20-165A
var GNSS_NACP_DESC = [
    'EPU ≥ 18.52 km (10nm)',  // 0
    'EPU < 18.52 km (10nm)',       // 1
    'EPU < 7.408 km (4nm)',        // 2
    'EPU < 3.704 km (2nm)',        // 3
    'EPU < 1852 m (1nm)',          // 4
    'EPU < 926 m (0.5nm)',         // 5
    'EPU < 555.6 m (0.3nm)',       // 6
    'EPU < 185.2 m (0.1nm)',       // 7
    'EPU < 92.6 m (0.05nm)',       // 8
    'EPU < 30 m',                  // 9
    'EPU < 10 m',                  // 10
    'EPU < 3 m'                    // 11
];

var GNSS_SAT_SYSTEMS = {
    'G': { name: 'GPS',     color: '#3a9ff5' },
    'R': { name: 'GLONASS', color: '#e84040' },
    'E': { name: 'Galileo', color: '#40c870' },
    'B': { name: 'BeiDou',  color: '#f09020' },
    'S': { name: 'SBAS',    color: '#b060e0' },
    'Q': { name: 'QZSS',    color: '#20c8c8' }
};

function gnssSystem(id) {
    return GNSS_SAT_SYSTEMS[(id || '')[0]] || { name: 'Unknown', color: '#888888' };
}

function hexToRgba(hex, alpha) {
    var r = parseInt(hex.slice(1, 3), 16);
    var g = parseInt(hex.slice(3, 5), 16);
    var b = parseInt(hex.slice(5, 7), 16);
    return 'rgba(' + r + ',' + g + ',' + b + ',' + alpha + ')';
}

// ── Sky / constellation plot ────────────────────────────────────────────────────
function drawSkyPlot(canvas, size, satellites, showAll) {
    canvas.width  = size;
    canvas.height = size;

    var ctx  = canvas.getContext('2d');
    var cx   = size / 2;
    var cy   = size / 2;
    var rMax = size / 2 - 22;

    ctx.fillStyle = '#f8f9fa';
    ctx.beginPath();
    ctx.arc(cx, cy, rMax + 2, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = '#ccc';
    ctx.lineWidth   = 1;
    ctx.stroke();

    // Elevation rings at 0°, 30°, 60°
    [0, 30, 60].forEach(function (el) {
        var r = (90 - el) / 90 * rMax;
        ctx.beginPath();
        ctx.arc(cx, cy, r, 0, 2 * Math.PI);
        ctx.strokeStyle = '#ccc';
        ctx.lineWidth   = 1;
        ctx.setLineDash([3, 3]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle    = '#aaa';
        ctx.font         = '9px sans-serif';
        ctx.textAlign    = 'left';
        ctx.textBaseline = 'top';
        ctx.fillText(el + '°', cx + 3, cy - r + 2);
    });

    // Azimuth spokes every 45°
    for (var a = 0; a < 360; a += 45) {
        var rad = (a - 90) * Math.PI / 180;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + rMax * Math.cos(rad), cy + rMax * Math.sin(rad));
        ctx.strokeStyle = '#ddd';
        ctx.lineWidth   = 1;
        ctx.stroke();
    }

    // Cardinal labels
    var cardinals = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' };
    ctx.font      = 'bold 11px sans-serif';
    ctx.fillStyle = '#555';
    Object.keys(cardinals).forEach(function (deg) {
        var r2 = (parseInt(deg) - 90) * Math.PI / 180;
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(cardinals[deg],
            cx + (rMax + 14) * Math.cos(r2),
            cy + (rMax + 14) * Math.sin(r2));
    });

    // Satellites
    var dotR    = Math.max(9, Math.min(14, size / 32));
    var visible = showAll ? satellites : satellites.filter(function (s) { return s.InSolution; });

    visible.forEach(function (sat) {
        if (sat.Elevation < 0 || sat.Azimuth < 0) return;
        var r   = (90 - sat.Elevation) / 90 * rMax;
        var ang = (sat.Azimuth - 90) * Math.PI / 180;
        var sx  = cx + r * Math.cos(ang);
        var sy  = cy + r * Math.sin(ang);
        var col = gnssSystem(sat.SatelliteID).color;

        ctx.beginPath();
        ctx.arc(sx, sy, dotR, 0, 2 * Math.PI);
        if (sat.InSolution) {
            ctx.fillStyle   = col;
            ctx.fill();
            ctx.strokeStyle = hexToRgba(col, 0.4);
            ctx.lineWidth   = 1;
            ctx.stroke();
        } else {
            ctx.fillStyle   = hexToRgba(col, 0.12);
            ctx.fill();
            ctx.strokeStyle = col;
            ctx.lineWidth   = 1.5;
            ctx.stroke();
        }

        ctx.fillStyle    = sat.InSolution ? '#fff' : col;
        ctx.font         = 'bold ' + Math.max(7, dotR - 3) + 'px sans-serif';
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(sat.SatelliteID, sx, sy);
    });
}

// ── Signal strength bar chart ───────────────────────────────────────────────────
function drawSignalBars(canvas, width, height, satellites, showAll) {
    var visible  = showAll ? satellites : satellites.filter(function (s) { return s.InSolution; });
    var padLeft  = 30, padBot = 30, padTop = 10, padRight = 6;
    var chartW   = width  - padLeft - padRight;
    var chartH   = height - padTop  - padBot;
    var n        = visible.length;
    var actualMax = n > 0 ? visible.reduce(function (m, s) { return Math.max(m, s.Signal); }, 0) : 0;
    var maxSig   = Math.max(50, Math.ceil(actualMax / 10) * 10);
    var step     = n > 0 ? chartW / n : chartW;
    var barW     = Math.max(10, Math.min(36, step * 0.72));

    canvas.width  = width;
    canvas.height = height;

    var ctx = canvas.getContext('2d');

    ctx.fillStyle = '#f8f9fa';
    ctx.fillRect(0, 0, width, height);

    // Y-axis grid + labels (dynamic steps up to maxSig)
    var gridSteps = [];
    for (var g = 0; g <= maxSig; g += 10) { gridSteps.push(g); }
    gridSteps.forEach(function (v) {
        var y = padTop + chartH - (v / maxSig * chartH);
        ctx.setLineDash([2, 3]);
        ctx.strokeStyle = '#ddd';
        ctx.lineWidth   = 1;
        ctx.beginPath();
        ctx.moveTo(padLeft, y);
        ctx.lineTo(width - padRight, y);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle    = '#aaa';
        ctx.font         = '9px sans-serif';
        ctx.textAlign    = 'right';
        ctx.textBaseline = 'middle';
        ctx.fillText(v, padLeft - 3, y);
    });

    ctx.save();
    ctx.translate(10, padTop + chartH / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillStyle    = '#aaa';
    ctx.font         = '9px sans-serif';
    ctx.textAlign    = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('dB-Hz', 0, 0);
    ctx.restore();

    if (n === 0) {
        ctx.fillStyle    = '#999';
        ctx.font         = '12px sans-serif';
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText('No satellites', width / 2, height / 2);
        return;
    }

    visible.forEach(function (sat, i) {
        var sig  = Math.max(0, sat.Signal);
        var barH = sig / maxSig * chartH;
        var bx   = padLeft + i * step + (step - barW) / 2;
        var by   = padTop + chartH - barH;
        var col  = gnssSystem(sat.SatelliteID).color;

        if (barH > 0) {
            ctx.fillStyle = sat.InSolution ? col : hexToRgba(col, 0.3);
            ctx.fillRect(Math.round(bx), Math.round(by), Math.round(barW), Math.round(barH));
        }

        if (barH >= 14) {
            ctx.fillStyle    = sat.InSolution ? 'rgba(255,255,255,0.9)' : hexToRgba(col, 0.7);
            ctx.font         = 'bold 9px sans-serif';
            ctx.textAlign    = 'center';
            ctx.textBaseline = 'top';
            ctx.fillText(sig, bx + barW / 2, by + 2);
        }

        ctx.fillStyle    = sat.InSolution ? '#333' : '#aaa';
        ctx.font         = (barW < 22 ? '7' : '9') + 'px sans-serif';
        ctx.textAlign    = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(sat.SatelliteID, bx + barW / 2, padTop + chartH + 4);
    });
}

// ── Controller ──────────────────────────────────────────────────────────────────
function GPSCtrl($rootScope, $scope, $state, $http, $interval, $timeout) {
    $scope.$parent.helppage = 'plates/gnss-help.html';
    $scope.data_list = [];
    $scope.showAll   = false;

    // Template helpers
    $scope.satSystem = function (id) { return gnssSystem(id).name; };
    $scope.satColor  = function (id) { return gnssSystem(id).color; };
    $scope.displaySatellites = function () {
        if ($scope.showAll) return $scope.data_list;
        return $scope.data_list.filter(function (s) { return s.InSolution; });
    };

    // ── WebSocket (/situation) ─────────────────────────────────────────────────
    function connect($scope) {
        if (($scope === undefined) || ($scope === null)) return;

        if (($scope.socket === undefined) || ($scope.socket === null)) {
            socket = new WebSocket(URL_GPS_WS);
            $scope.socket = socket;
        }

        $scope.ConnectState = "Disconnected";

        socket.onopen  = function ()    { $scope.ConnectState = "Connected"; };
        socket.onclose = function ()    {
            $scope.ConnectState = "Disconnected";
            $scope.$apply();
            delete $scope.socket;
            setTimeout(function () { connect($scope); }, 1000);
        };
        socket.onerror = function ()    {
            $scope.ConnectState = "Error";
            resetSituation();
            $scope.$apply();
        };
        socket.onmessage = function (msg) {
            if ($scope === undefined || $scope === null) { socket.close(); return; }
            loadSituation(msg.data);
            $scope.$apply();
        };
    }

    // ── Situation processing ───────────────────────────────────────────────────
    function pad2(n) { return n < 10 ? '0' + n : '' + n; }

    function loadSituation(data) {
        var s = angular.fromJson(data);

        $scope.Satellites             = s.GPSSatellites;
        $scope.GPS_satellites_tracked = s.GPSSatellitesTracked;
        $scope.GPS_satellites_seen    = s.GPSSatellitesSeen;
        $scope.Quality                = s.GPSFixQuality;
        $scope.GPS_PositionSampleRate = s.GPSPositionSampleRate.toFixed(1);

        var solutionText = "Unknown";
        if      (s.GPSFixQuality === 0) solutionText = "No Fix";
        else if (s.GPSFixQuality === 1) solutionText = "3D GPS";
        else if (s.GPSFixQuality === 2) solutionText = "3D GPS + SBAS";
        else if (s.GPSFixQuality === 6) solutionText = "Dead Reckoning";
        $scope.SolutionText = solutionText;

        $scope.gps_horizontal_accuracy = s.GPSHorizontalAccuracy.toFixed(1);
        if ($scope.gps_horizontal_accuracy > 19999) {
            $scope.gps_horizontal_accuracy = "∞";
            $scope.gps_lat   = "--";
            $scope.gps_lon   = "--";
            $scope.gps_track = "--";
            $scope.gps_speed = "--";
        }

        $scope.gps_vertical_accuracy = (s.GPSVerticalAccuracy * 3.2808).toFixed(1);
        if ($scope.gps_vertical_accuracy > 9999) {
            $scope.gps_vertical_accuracy = "∞";
            $scope.gps_alt        = "--";
            $scope.gps_vert_speed = "--";
        }

        $scope.gps_lat                    = s.GPSLatitude.toFixed(5);
        $scope.gps_lon                    = s.GPSLongitude.toFixed(5);
        $scope.gps_alt                    = s.GPSAltitudeMSL.toFixed(1);
        $scope.gps_height_above_ellipsoid = s.GPSHeightAboveEllipsoid.toFixed(1);
        $scope.gps_track                  = s.GPSTrueCourse.toFixed(1);
        $scope.gps_speed                  = s.GPSGroundSpeed.toFixed(1);
        $scope.gps_vert_speed             = s.GPSVerticalSpeed.toFixed(1);

        if (s.GPSLatitude === 0 && s.GPSLongitude === 0) {
            $scope.gps_lat = $scope.gps_lon = "--";
            $scope.gps_alt = $scope.gps_height_above_ellipsoid = "--";
            $scope.gps_track = $scope.gps_speed = $scope.gps_vert_speed = "--";
        }

        $scope.gps_hdop  = (s.GPSHDOP > 0) ? s.GPSHDOP.toFixed(1) : '--';
        $scope.gps_pdop  = (s.GPSPDOP > 0) ? s.GPSPDOP.toFixed(1) : '--';
        $scope.gps_nacp      = (s.GPSNACp !== undefined) ? s.GPSNACp : '--';
        $scope.gps_nacp_desc = GNSS_NACP_DESC[s.GPSNACp] || '';

        var t = new Date(s.GPSTime);
        $scope.gps_time_display = (!isNaN(t.getTime()) && t.getFullYear() >= 2000)
            ? pad2(t.getUTCHours()) + ':' + pad2(t.getUTCMinutes()) + ':' + pad2(t.getUTCSeconds()) + ' UTC'
            : '--:--:-- UTC';

        // Update OL position marker
        if (olPositionFeature && s.GPSLatitude !== 0 && s.GPSLongitude !== 0) {
            var coords = ol.proj.fromLonLat([s.GPSLongitude, s.GPSLatitude]);
            olPositionFeature.setGeometry(new ol.geom.Point(coords));
            if (olMap) {
                if (!olZoomedToFix) {
                    olMap.setView(new ol.View({ center: coords, zoom: 11, enableRotation: false }));
                    olZoomedToFix = true;
                } else {
                    olMap.getView().setCenter(coords);
                }
            }
        }
    }

    function resetSituation() {
        $scope.gps_lat = $scope.gps_lon = "--";
        $scope.gps_alt = $scope.gps_height_above_ellipsoid = "--";
        $scope.gps_track = $scope.gps_speed = $scope.gps_vert_speed = "--";
        $scope.gps_hdop = $scope.gps_pdop = "--";
        $scope.gps_time_display = "--:--:-- UTC";
    }

    // ── Satellite polling ──────────────────────────────────────────────────────
    function getSatellites() {
        $http.get(URL_SATELLITES_GET).then(function (response) {
            loadSatellites(response.data);
        }, function () {});
    }

    function loadSatellites(satellites) {
        if (($scope === undefined) || ($scope === null)) return;
        $scope.data_list.length = 0;
        for (var key in satellites) {
            var sat = satellites[key];
            $scope.data_list.push({
                SatelliteNMEA: sat.SatelliteNMEA,
                SatelliteID:   sat.SatelliteID,
                Elevation:     sat.Elevation,
                Azimuth:       sat.Azimuth,
                Signal:        sat.Signal,
                InSolution:    sat.InSolution
            });
        }
        $scope.redrawCharts();
    }

    var updateSatellites = $interval(getSatellites, 1000, 0, false);

    // ── Canvas charts ──────────────────────────────────────────────────────────
    $scope.redrawCharts = function () {
        var skyCanvas = document.getElementById('gnss_sky_plot');
        var barCanvas = document.getElementById('gnss_signal_bars');
        var skyWrap   = document.getElementById('gnss_sky_wrap');
        var barWrap   = document.getElementById('gnss_bar_wrap');
        if (!skyCanvas || !barCanvas || !skyWrap || !barWrap) return;

        var size = skyWrap.offsetWidth - 12;
        if (size <= 0) return;

        drawSkyPlot(skyCanvas, size, $scope.data_list, $scope.showAll);
        drawSignalBars(barCanvas, barWrap.offsetWidth - 12, size, $scope.data_list, $scope.showAll);
    };

    var resizeHandler = function () { $scope.redrawCharts(); };
    window.addEventListener('resize', resizeHandler);

    // ── OpenLayers map ─────────────────────────────────────────────────────────
    var olMap             = null;
    var olPositionFeature = null;
    var olZoomedToFix     = false;

    $timeout(function () {
        olPositionFeature = new ol.Feature();
        olPositionFeature.setStyle(new ol.style.Style({
            image: new ol.style.Circle({
                radius: 8,
                fill:   new ol.style.Fill({ color: 'rgba(255, 127, 0, 0.85)' }),
                stroke: new ol.style.Stroke({ color: '#ffffff', width: 2 })
            })
        }));

        var osmLayer = new ol.layer.Tile({
            title: '<i class="fa fa-cloud"></i> OSM',
            type:  'base',
            source: new ol.source.OSM()
        });

        olMap = new ol.Map({
            target: 'gnss_map_display',
            layers: [
                osmLayer,
                new ol.layer.Vector({
                    source: new ol.source.Vector({ features: [olPositionFeature] }),
                    zIndex: 10
                })
            ],
            controls: ol.control.defaults({ attribution: false, rotate: false }),
            view: new ol.View({
                center: ol.proj.fromLonLat([10.0, 52.0]),
                zoom: 5,
                enableRotation: false
            })
        });
        olMap.addControl(new ol.control.LayerSwitcher());

        // Load offline MBTiles base layers from local Stratux tile server (same as Map plate)
        $http.get(URL_GET_TILESETS).then(function (response) {
            var tilesets = angular.fromJson(response.data);
            var hasLocalBase = false;
            for (var file in tilesets) {
                var meta     = tilesets[file];
                if (!meta.type || meta.type !== 'baselayer') continue;
                var name     = meta.name ? meta.name : file;
                var format   = meta.format  ? meta.format           : 'png';
                var minzoom  = meta.minzoom ? parseInt(meta.minzoom) : 1;
                var maxzoom  = meta.maxzoom ? parseInt(meta.maxzoom) : 18;
                var ext = [-180, -85, 180, 85];
                if (meta.bounds) { ext = meta.bounds.split(',').map(Number); }
                ext = ol.proj.transformExtent(ext, 'EPSG:4326', 'EPSG:3857');

                var layer;
                if (format.toLowerCase() === 'pbf') {
                    var vt = new ol.layer.VectorTile({
                        title:  name,
                        type:   'base',
                        extent: ext,
                        source: new ol.source.VectorTile({
                            url:     URL_GET_TILE + '/' + file + '/{z}/{x}/{-y}.' + format,
                            format:  new ol.format.MVT(),
                            maxZoom: maxzoom,
                            minZoom: minzoom
                        })
                    });
                    if (meta.stratux_style_url) {
                        fetch(meta.stratux_style_url).then(function (r) {
                            r.json().then(function (style) { olms.stylefunction(vt, style, meta.id); });
                        });
                    }
                    layer = vt;
                } else {
                    layer = new ol.layer.Tile({
                        title:  name,
                        type:   'base',
                        extent: ext,
                        source: new ol.source.XYZ({
                            url:     URL_GET_TILE + '/' + file + '/{z}/{x}/{-y}.' + format,
                            maxZoom: maxzoom,
                            minZoom: minzoom
                        })
                    });
                }
                olMap.getLayers().insertAt(0, layer);
                hasLocalBase = true;
            }
            if (hasLocalBase) { osmLayer.setVisible(false); }
        }, function () { /* tileset fetch failed — keep OSM */ });

        $scope.redrawCharts();
    }, 100);

    // ── Lifecycle ──────────────────────────────────────────────────────────────
    $state.get('gnss').onEnter = function () {};

    $state.get('gnss').onExit = function () {
        if (($scope.socket !== undefined) && ($scope.socket !== null)) {
            $scope.socket.close();
            $scope.socket = null;
        }
        $interval.cancel(updateSatellites);
        window.removeEventListener('resize', resizeHandler);
        if (olMap) {
            olMap.setTarget(null);
            olMap = null;
        }
        olPositionFeature = null;
    };

    connect($scope);
}
