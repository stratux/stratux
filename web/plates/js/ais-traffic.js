angular.module('appControllers').controller('AISTrafficCtrl', AISTrafficCtrl);
AISTrafficCtrl.$inject = ['$rootScope', '$scope', '$state', '$http', '$interval'];

function AISTrafficCtrl($rootScope, $scope, $state, $http, $interval) {

	$scope.$parent.helppage = 'plates/ais-traffic-help.html';
	$scope.ais_data_list = [];
	$scope.AIS_connected = false;
	$scope.DaisyAIS_connected = false;
	$scope.GPS_connected = false;
	$scope.AIS_messages_total = 0;

	// Ship type mapping based on AIS specification
	var shipTypes = {
		0: 'Unknown',
		20: 'Wing in Ground',
		30: 'Fishing',
		31: 'Towing',
		32: 'Towing (large)',
		33: 'Dredging',
		34: 'Diving',
		35: 'Military',
		36: 'Sailing',
		37: 'Pleasure Craft',
		40: 'High Speed Craft',
		50: 'Pilot Vessel',
		51: 'SAR',
		52: 'Tug',
		53: 'Port Tender',
		54: 'Anti-pollution',
		55: 'Law Enforcement',
		60: 'Passenger',
		70: 'Cargo',
		80: 'Tanker',
		90: 'Other'
	};

	$scope.getShipType = function(typeCode) {
		if (!typeCode) return 'Unknown';
		// Get the category (first digit * 10)
		var category = Math.floor(typeCode / 10) * 10;
		return shipTypes[category] || shipTypes[typeCode] || 'Type ' + typeCode;
	};

	function fetchAISTraffic() {
		$http.get(URL_AIS_TRAFFIC_GET).then(function(response) {
			var data = angular.fromJson(response.data);
			if (data !== null) {
				$scope.ais_data_list = data;
			} else {
				$scope.ais_data_list = [];
			}
		}, function(error) {
			$scope.ais_data_list = [];
		});
	}

	function fetchStatus() {
		$http.get(URL_STATUS_GET).then(function(response) {
			var status = angular.fromJson(response.data);
			$scope.AIS_connected = status.AIS_connected;
			$scope.DaisyAIS_connected = status.DaisyAIS_connected;
			$scope.GPS_connected = status.GPS_connected;
			$scope.AIS_messages_total = status.AIS_messages_total;
		});
	}

	// Initial fetch
	fetchAISTraffic();
	fetchStatus();

	// Set up refresh interval (every 1 second)
	var updateInterval = $interval(function() {
		fetchAISTraffic();
		fetchStatus();
	}, 1000);

	// Clean up on controller destroy
	$scope.$on('$destroy', function() {
		if (angular.isDefined(updateInterval)) {
			$interval.cancel(updateInterval);
		}
	});
}
