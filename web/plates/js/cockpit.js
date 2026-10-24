angular.module('appControllers').controller('CockpitCtrl', CockpitCtrl); // get the main module contollers set
CockpitCtrl.$inject = ['$scope', '$http', '$interval']; // Inject my dependencies

// Large, high-contrast status page meant to be glanced at in flight.
function CockpitCtrl($scope, $http, $interval) {

	$scope.$parent.helppage = 'plates/status-help.html';

	var STALE_MS = 3000;
	var socket = null;
	var destroyed = false;
	var lastUpdate = 0;

	$scope.linkOk = false;
	$scope.s = {};
	$scope.enabled = {};

	$http.get(URL_SETTINGS_GET).then(function (response) {
		var settings = angular.fromJson(response.data);
		$scope.enabled = {
			uat: settings.UAT_Enabled,
			es: settings.ES_Enabled,
			ogn: settings.OGN_Enabled,
			ais: settings.AIS_Enabled,
			gps: settings.GPS_Enabled,
			imu: settings.IMU_Sensor_Enabled,
			bmp: settings.BMP_Sensor_Enabled
		};
	});

	function update(status) {
		var s = {};
		s.gpsFix = stxGpsHasFix(status.GPS_solution);
		s.gpsSolution = status.GPS_solution || '--';
		s.gpsLevel = !status.GPS_connected ? 'warning' : (s.gpsFix ? 'good' : 'caution');
		s.nacp = s.gpsFix ? status.GPS_NACp : '--';
		s.nacpLevel = stxNacpLevel(status.GPS_NACp, status.GPS_solution);
		s.accuracy = s.gpsFix ? '±' + status.GPS_position_accuracy.toFixed(0) + ' m' : 'no position';
		s.satsUsed = status.GPS_satellites_locked;
		s.satsTracked = status.GPS_satellites_tracked;
		s.satsLevel = status.GPS_satellites_locked >= 6 ? 'good' : (status.GPS_satellites_locked >= 4 ? 'caution' : 'warning');

		s.clients = status.Connected_Users;
		s.clientsLevel = status.Connected_Users > 0 ? 'good' : 'warning';

		s.traffic = (status.ES_traffic_targets_tracking || 0) + (status.UAT_traffic_targets_tracking || 0);
		s.esMsgs = status.ES_messages_last_minute;
		s.uatMsgs = status.UAT_messages_last_minute;
		s.ognMsgs = status.OGN_messages_last_minute;
		s.aisMsgs = status.AIS_messages_last_minute;

		s.cpuTemp = status.CPUTemp > 0 ? status.CPUTemp.toFixed(0) + '°C' : '--';
		s.cpuLevel = stxCpuTempLevel(status.CPUTemp);

		s.sd = stxSdProtection(status);
		s.imu = status.IMUConnected;
		s.bmp = status.BMPConnected;
		s.errors = status.Errors || [];
		$scope.s = s;
	}

	function connect() {
		if (destroyed)
			return;
		socket = new WebSocket(URL_STATUS_WS);
		socket.onmessage = function (msg) {
			lastUpdate = Date.now();
			update(JSON.parse(msg.data));
			$scope.linkOk = true;
			$scope.$apply();
		};
		socket.onclose = function () {
			socket = null;
			if (!destroyed)
				setTimeout(connect, 1000);
		};
	}

	// The status socket pushes roughly once a second; flag the page as stale if that stops,
	// so old "good" values are never mistaken for current ones.
	var staleCheck = $interval(function () {
		$scope.linkOk = (Date.now() - lastUpdate) < STALE_MS;
	}, 1000);

	$scope.$on('$destroy', function () {
		destroyed = true;
		$interval.cancel(staleCheck);
		if (socket)
			socket.close();
	});

	connect();
}
