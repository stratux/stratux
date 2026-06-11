angular.module('appControllers').controller('GPSCtrl', GPSCtrl);
GPSCtrl.$inject = ['$rootScope', '$scope', '$state', '$http', '$interval'];

function GPSCtrl($rootScope, $scope, $state, $http, $interval) {
    $scope.$parent.helppage = 'plates/gps-help.html';
    $scope.data_list = [];

    function connect($scope) {
        if (($scope === undefined) || ($scope === null))
            return;

        if (($scope.socket === undefined) || ($scope.socket === null)) {
            socket = new WebSocket(URL_GPS_WS);
            $scope.socket = socket;
        }

        $scope.ConnectState = "Disconnected";

        socket.onopen = function (msg) {
            $scope.ConnectState = "Connected";
        };

        socket.onclose = function (msg) {
            $scope.ConnectState = "Disconnected";
            $scope.$apply();
            delete $scope.socket;
            setTimeout(function() { connect($scope); }, 1000);
        };

        socket.onerror = function (msg) {
            $scope.ConnectState = "Error";
            resetSituation();
            $scope.$apply();
        };

        socket.onmessage = function (msg) {
            if ($scope === undefined || $scope === null) {
                socket.close();
                return;
            }
            loadSituation(msg.data);
            $scope.$apply();
        };
    }

    var display_area_size = -1;

    function sizeMap() {
        var width = 0;
        var div = document.getElementById("map_display");
        if (div == null) {
            return width;
        }
        var el = div.parentElement;
        width = el.offsetWidth;
        if (width !== display_area_size) {
            display_area_size = width;
            $scope.map_width = width;
            $scope.map_height = width;
        }
        return width;
    }

    function setGeoReferenceMap(la, lo) {
        var map_width  = 2530;
        var map_height = 1603;
        var map_zero_x = 1192;
        var map_zero_y = 1124;
        var font_size  = 18;

        sizeMap();
        var div_width  = $scope.map_width;
        var div_height = $scope.map_height;

        var x = (map_width * (180 + lo) / 360) - (map_width/2 - map_zero_x);

        var la_rad  = la * Math.PI / 180;
        var merc_n  = Math.log(Math.tan((la_rad / 2) + (Math.PI / 4)));
        var y = (map_height / 2) - (map_width * merc_n / (2 * Math.PI)) - (map_height/2 - map_zero_y);

        $scope.map_pos_x    = map_width  - Math.round(x - (div_width  / 2));
        $scope.map_pos_y    = map_height - Math.round(y - (div_height / 2));
        $scope.map_mark_x   = Math.round((div_width  - (font_size * 0.85)) / 2);
        $scope.map_mark_y   = Math.round((div_height - font_size) / 2);
    }

    function loadSituation(data) {
        var situation = angular.fromJson(data);

        $scope.Satellites              = situation.GPSSatellites;
        $scope.GPS_satellites_tracked  = situation.GPSSatellitesTracked;
        $scope.GPS_satellites_seen     = situation.GPSSatellitesSeen;
        $scope.Quality                 = situation.GPSFixQuality;
        $scope.GPS_PositionSampleRate  = situation.GPSPositionSampleRate.toFixed(1);

        var solutionText = "Unknown";
        if (situation.GPSFixQuality === 0) {
            solutionText = "No Fix";
        } else if (situation.GPSFixQuality === 1) {
            solutionText = "3D GPS";
        } else if (situation.GPSFixQuality === 2) {
            solutionText = "3D GPS + SBAS";
        } else if (situation.GPSFixQuality === 6) {
            solutionText = "Dead Reckoning";
        }
        $scope.SolutionText = solutionText;

        $scope.gps_horizontal_accuracy = situation.GPSHorizontalAccuracy.toFixed(1);
        if ($scope.gps_horizontal_accuracy > 19999) {
            $scope.gps_horizontal_accuracy = "∞";
            $scope.gps_lat   = "--";
            $scope.gps_lon   = "--";
            $scope.gps_track = "--";
            $scope.gps_speed = "--";
            $scope.map_opacity      = 0.2;
            $scope.map_mark_opacity = 0;
        } else {
            $scope.map_opacity      = 1;
            $scope.map_mark_opacity = 1;
        }

        $scope.gps_vertical_accuracy = (situation.GPSVerticalAccuracy * 3.2808).toFixed(1);
        if ($scope.gps_vertical_accuracy > 9999) {
            $scope.gps_vertical_accuracy = "∞";
            $scope.gps_alt        = "--";
            $scope.gps_vert_speed = "--";
        }

        $scope.gps_lat                    = situation.GPSLatitude.toFixed(5);
        $scope.gps_lon                    = situation.GPSLongitude.toFixed(5);
        $scope.gps_alt                    = situation.GPSAltitudeMSL.toFixed(1);
        $scope.gps_height_above_ellipsoid = situation.GPSHeightAboveEllipsoid.toFixed(1);
        $scope.gps_track                  = situation.GPSTrueCourse.toFixed(1);
        $scope.gps_speed                  = situation.GPSGroundSpeed.toFixed(1);
        $scope.gps_vert_speed             = situation.GPSVerticalSpeed.toFixed(1);

        if ($scope.gps_lat == 0 && $scope.gps_lon == 0) {
            $scope.gps_lat                    = "--";
            $scope.gps_lon                    = "--";
            $scope.gps_alt                    = "--";
            $scope.gps_height_above_ellipsoid = "--";
            $scope.gps_track                  = "--";
            $scope.gps_speed                  = "--";
            $scope.gps_vert_speed             = "--";
        }

        setGeoReferenceMap(situation.GPSLatitude, situation.GPSLongitude);
    }

    function resetSituation() {
        $scope.gps_lat                    = "--";
        $scope.gps_lon                    = "--";
        $scope.gps_alt                    = "--";
        $scope.gps_height_above_ellipsoid = "--";
        $scope.gps_track                  = "--";
        $scope.gps_speed                  = "--";
        $scope.gps_vert_speed             = "--";
    }

    function getSatellites() {
        $http.get(URL_SATELLITES_GET).then(function (response) {
            loadSatellites(response.data);
        }, function (response) {
            $scope.raw_data = "error getting satellite data";
        });
    }

    function setSatellite(obj, new_satellite) {
        new_satellite.SatelliteNMEA = obj.SatelliteNMEA;
        new_satellite.SatelliteID   = obj.SatelliteID;
        new_satellite.Elevation     = obj.Elevation;
        new_satellite.Azimuth       = obj.Azimuth;
        new_satellite.Signal        = obj.Signal;
        new_satellite.InSolution    = obj.InSolution;
    }

    function loadSatellites(satellites) {
        if (($scope === undefined) || ($scope === null))
            return;

        $scope.data_list.length = 0;
        for (var key in satellites) {
            var new_satellite = {};
            setSatellite(satellites[key], new_satellite);
            $scope.data_list.push(new_satellite);
        }
    }

    var updateSatellites = $interval(getSatellites, 1000, 0, false);

    $state.get('gps').onEnter = function () {};

    $state.get('gps').onExit = function () {
        if (($scope.socket !== undefined) && ($scope.socket !== null)) {
            $scope.socket.close();
            $scope.socket = null;
        }
        $interval.cancel(updateSatellites);
    };

    connect($scope);
}
