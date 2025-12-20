angular.module('appControllers').controller('AISCtrl', AISCtrl);
AISCtrl.$inject = ['$rootScope', '$scope', '$state', '$http', '$interval'];

// AIS Traffic Controller - Displays only AIS vessel traffic
function AISCtrl($rootScope, $scope, $state, $http, $interval) {

	$scope.$parent.helppage = 'plates/ais-help.html';
	$scope.ais_list = [];
	$scope.ConnectState = "Disconnected";
	$scope.GPS_connected = false;
	$scope.AIS_connected = false;
	$scope.ExternalAIS_connected = false;
	$scope.AIS_messages_total = 0;

	// AIS vessel type descriptions
	var vesselTypes = {
		0: "Not available",
		20: "WIG",
		30: "Fishing",
		31: "Towing",
		32: "Towing large",
		33: "Dredging",
		34: "Diving ops",
		35: "Military ops",
		36: "Sailing",
		37: "Pleasure craft",
		40: "High speed",
		50: "Pilot vessel",
		51: "SAR",
		52: "Tug",
		53: "Port tender",
		54: "Anti-pollution",
		55: "Law enforce",
		60: "Passenger",
		70: "Cargo",
		80: "Tanker",
		90: "Other"
	};

	function getVesselType(typeCode) {
		if (typeCode === 0) return "Unknown";
		// Get the base type (first digit * 10)
		var baseType = Math.floor(typeCode / 10) * 10;
		if (vesselTypes[typeCode]) {
			return vesselTypes[typeCode];
		} else if (vesselTypes[baseType]) {
			return vesselTypes[baseType];
		}
		return "Type " + typeCode;
	}

	function setVessel(obj, vessel) {
		vessel.icao_int = obj.Icao_addr;
		vessel.mmsi = obj.Icao_addr.toString();
		vessel.name = obj.Tail && obj.Tail.trim().length > 0 ? obj.Tail.trim() : null;
		vessel.callsign = obj.Reg && obj.Reg.trim().length > 0 ? obj.Reg.trim() : null;
		vessel.lat = obj.Lat;
		vessel.lon = obj.Lng;
		vessel.speed = obj.Speed_valid ? Math.round(obj.Speed) : 0;
		vessel.heading = obj.Speed_valid ? Math.round(obj.Track) : 0;
		vessel.bearing = obj.BearingDist_valid ? obj.Bearing : 0;
		vessel.dist = obj.BearingDist_valid ? obj.Distance / 1852 : 0; // Convert to nautical miles
		vessel.age = obj.Age;
		vessel.vesselType = getVesselType(obj.SurfaceVehicleType || 0);
		vessel.vesselTypeCode = obj.SurfaceVehicleType || 0;
		vessel.Position_valid = obj.Position_valid;
		vessel.Last_source = obj.Last_source;
	}

	function isSameVessel(mmsi1, mmsi2) {
		return mmsi1 === mmsi2;
	}

	var socket = null;

	function connect($scope) {
		if (($scope === undefined) || ($scope === null))
			return;

		if (($scope.socket === undefined) || ($scope.socket === null)) {
			socket = new WebSocket(URL_TRAFFIC_WS);
			$scope.socket = socket;
		}

		$scope.ConnectState = "Disconnected";

		socket.onopen = function (msg) {
			$scope.ConnectState = "Connected";
			$scope.$apply();
		};

		socket.onclose = function (msg) {
			$scope.ConnectState = "Disconnected";
			$scope.$apply();
			setTimeout(function() { connect($scope); }, 1000);
		};

		socket.onerror = function (msg) {
			$scope.ConnectState = "Problem";
			$scope.$apply();
		};

		socket.onmessage = function (msg) {
			var message = JSON.parse(msg.data);

			// Only process AIS traffic (Last_source = 8 for AIS)
			// Also check TargetType = 5 for AIS
			if (message.Last_source !== 8) {
				return; // Skip non-AIS traffic
			}

			// Find existing vessel
			var existingIdx = -1;
			for (var i = 0; i < $scope.ais_list.length; i++) {
				if (isSameVessel($scope.ais_list[i].icao_int, message.Icao_addr)) {
					setVessel(message, $scope.ais_list[i]);
					existingIdx = i;
					break;
				}
			}

			// Add new vessel if not found and has valid position
			if (existingIdx < 0 && message.Position_valid) {
				var new_vessel = {};
				setVessel(message, new_vessel);
				$scope.ais_list.unshift(new_vessel);
			}

			$scope.$apply();
		};
	}

	// Get status updates (GPS connection, AIS connection status)
	var getStatus = $interval(function () {
		$http.get(URL_STATUS_GET).then(function (response) {
			var status = angular.fromJson(response.data);
			$scope.GPS_connected = status.GPS_connected;
			$scope.AIS_connected = status.AIS_connected;
			$scope.ExternalAIS_connected = status.ExternalAIS_connected;
			$scope.AIS_messages_total = status.AIS_messages_total + (status.ExternalAIS_messages_total || 0);
		}, function (response) {
			// Error handling - silent
		});
	}, 1000, 0, false);

	// Clean up stale vessels every 10 seconds
	// AIS vessels are retained for 15 minutes (900 seconds)
	var clearStaleVessels = $interval(function () {
		for (var i = $scope.ais_list.length; i > 0; i--) {
			if ($scope.ais_list[i - 1].age > 900) {
				$scope.ais_list.splice(i - 1, 1);
			}
		}
	}, 10000, 0, false);

	// Initialize connection
	connect($scope);

	// Cleanup on page exit
	$state.get('ais').onExit = function () {
		if (($scope.socket !== undefined) && ($scope.socket !== null)) {
			$scope.socket.close();
			$scope.socket = null;
		}
		$interval.cancel(getStatus);
		$interval.cancel(clearStaleVessels);
	};
}
