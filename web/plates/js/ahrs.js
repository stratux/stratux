function AHRSRenderer(locationId) {
    this.width = -1;
    this.height = -1;

    this.locationId = locationId;
    this.canvas = document.getElementById(this.locationId);
    this.resize();

    // State variables
    this.pitch = 0;
    this.roll = 0;
    this.heading = 0;
    this.slipSkid = 0;
    this.altitude = 0;
    this.messages = [];

    var display = SVG(this.locationId).viewbox(-200, -200, 400, 400).group();

    this.ai = display.group().addClass('ai');

    var defs = this.ai.defs(),
        earthClip = defs.rect(2400, 1200).x(-1200).y(0),
        screenClip = defs.rect(400, 400).cx(0).cy(0);
    this.pitchClip = defs.circle(320).cx(0).cy(0);
    this.rollClip = defs.polygon('0,0 -200,-200 200,-200');

    this.ai = this.ai.clipWith(screenClip).group();

    // card is the earth+sky+pitch marks, moves with both pitch and roll.
    this.pitchScale = 0.5;
    this.card = this.ai.group();
    this.card.circle(2400).cx(0).cy(0).addClass('sky'); // Sky
    this.card.line(-1200, 0, 1200, 0).addClass('marks'); // Horizon line
    this.card.circle(2400).cx(0).cy(0).addClass('earth').clipWith(earthClip); // Earth

    var pitchMarks = this.card.group().addClass('marks').clipWith(this.pitchClip);
    var y;
    for (var i = -1050; i <= 1050; i += 50) {
        y = i * this.pitchScale;
        if (i % 100 === 0) {
            pitchMarks.line(-30, y, 30, y);
            if (i !== 0) {
                pitchMarks.text(Math.abs(i) <= 900 ? Math.abs(i / 10).toString() : '80').x(-55).cy(y).addClass('markText');
                pitchMarks.text(Math.abs(i) <= 900 ? Math.abs(i / 10).toString() : '80').x(+55).cy(y).addClass('markText');
            }
        } else {
            pitchMarks.line(-15, y, 15, y);
        }
    }

    this.rollMarks = this.ai.group().addClass('marks').clipWith(this.rollClip);
    for (i = -180; i < 180; i += 10) {
        if (i === 0) {
            this.rollMarks.polygon('-10,-189 0,-175 10,-189').style('stroke-width', 0);
        }
        else if (i % 30 === 0) {
            this.rollMarks.line(0, -175, 0, -195).rotate(i, 0, 0);
        } else {
            this.rollMarks.line(0, -175, 0, -189).rotate(i, 0, 0);
        }
    }

    var rollPointer = this.ai.group().addClass('marks');
    rollPointer.polygon('-10,-160 0,-174 10,-160').style('stroke-width', 0);
    rollPointer.polygon('-10,+160 0,+174 10,+160').style('stroke-width', 0);
    this.skidBar = this.ai.rect(20, 6).cx(0).y(-158).style('stroke-width', 0).addClass('marks');

    var pointer = this.ai.group().addClass('pointer');
    pointer.polygon('-150,-3 -78,-3 -75,0 -78,3 -150,3');
    pointer.polygon('+150,-3 +78,-3 +75,0 +78,3 +150,3');
    pointer.polygon('-75,25 0,0 75,25 25,25 25,20 -25,20 -25,25').addClass('pointerBG');
    pointer.polygon('-75,25 0,0 75,25 0,10');
    pointer.line(0, 0, 0, 10);

    this.headingMarks = this.ai.group().addClass('marks');
    for (i = -200; i <= 920; i += 20) {
        if (i % 60 === 0) {
            this.headingMarks.line(i, 175, i, 178);
            this.headingMarks.text(((i < 0 ? (i / 2 + 360) : i / 2) % 360).toString()).x(i).cy(185).addClass('markText');
            this.headingMarks.line(i, 192, i, 195);
        } else {
            this.headingMarks.line(i, 175, i, 195).style('stroke-width', 1);
        }
    }

    this.err = display.group().addClass('error').group();
    this.err.rect(400, 400).cx(0).cy(0);
    this.err.line(-200, -200, 200, +200);
    this.err.line(-200, +200, 200, -200);

    this.message = display.group().cy(-85);
    this.msgText = this.message.text("").addClass('msgText').build(true);
    var tb = this.msgText.bbox();
    this.msgTextBg = this.message.rect(tb.x, tb.y, tb.w, tb.h).stroke({'width': 1}).after(this.msgText);
}

AHRSRenderer.prototype = {
    constructor: AHRSRenderer,

    resize: function () {
        var canvasWidth = this.canvas.parentElement.offsetWidth - 12;

        if (canvasWidth !== this.width) {
            this.width = canvasWidth;
            this.height = canvasWidth * 0.5;

            this.canvas.width = this.width;
            this.canvas.height = this.height;
        }
    },

    update: function (pitch, roll, heading, slipSkid) {
        this.pitch = pitch;
        this.roll = roll;
        this.heading = heading;
        this.slipSkid = slipSkid;
        if (this.slipSkid < -10) {
            this.slipSkid = -10;
        } else if (this.slipSkid > 10) {
            this.slipSkid = +10;
        }

        if (this.messages.length > 0) {
            this.message.hide();
            this.msgText.clear();
            var msgs = this.messages;
            this.msgText.text(function (add) {
                for (var i = 0; i < msgs.length; i++) {
                    add.tspan(msgs[i]).center(0, 20 * i).newLine();
                }
            });
            var tb = this.msgText.bbox();
            this.msgTextBg.attr({x: tb.x - 3, y: tb.y - 2, width: tb.w + 6, height: tb.h + 4});
            this.message.show();
        } else {
            this.message.hide();
            this.msgText.clear();
        }

        this.pitchClip.translate(0, -10 * this.pitch * this.pitchScale);
        this.rollClip.rotate(this.roll, 0, 0);
        this.card.rotate(0, 0, 0).translate(0, 10 * this.pitch * this.pitchScale);
        this.card.rotate(-this.roll, 0, -10 * this.pitch * this.pitchScale);
        this.rollMarks.rotate(-this.roll, 0, 0);
        this.headingMarks.translate(-2 * (this.heading % 360), 0);
        this.skidBar.translate(-2 * this.slipSkid, 0);
    },

    turn_on: function () {
        this.err.hide();
        this.ai.show();
        this.update(this.pitch, this.roll, this.heading, this.slipSkid);
    },

    turn_off: function () {
        this.update(this.pitch, this.roll, this.heading, this.slipSkid);
        this.ai.hide();
        this.err.show();
    }
};

// ── AHRSCtrl ──────────────────────────────────────────────────────────────────

angular.module('appControllers').controller('AHRSCtrl', AHRSCtrl);
AHRSCtrl.$inject = ['$rootScope', '$scope', '$state', '$http', '$interval'];

const MSG_GROUND_TEST = ["GROUND TEST MODE - GPS REQUIRED", "DO NOT USE IN FLIGHT WITHOUT GPS"],
    MSG_LEVELING    = ["\n", "CALIBRATING", "FLY STRAIGHT AND DO NOT MOVE SENSOR"],
    MSG_PSEUDO_AHRS = ["WARNING - USING GPS PSEUDO AHRS", "CONNECT AN AHRS BOARD TO USE TRUE AHRS"],
    MSG_NO_AHRS     = ["NO AHRS AVAILABLE", "MUST HAVE IMU AND/OR GPS FOR AHRS"];

function AHRSCtrl($rootScope, $scope, $state, $http, $interval) {
    $scope.$parent.helppage = 'plates/ahrs-help.html';
    $scope.isHidden = false;
    $scope.noSleep = new NoSleep();

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

    var statusGPS = document.getElementById("status-gps"),
        statusIMU = document.getElementById("status-imu"),
        statusBMP = document.getElementById("status-bmp"),
        statusCal = document.getElementById("status-calibrating");

    function loadSituation(data) {
        var situation = angular.fromJson(data);

        $scope.gps_horizontal_accuracy = situation.GPSHorizontalAccuracy.toFixed(1);

        var msg_ix = ahrs.messages.indexOf(MSG_GROUND_TEST[0]);
        if (msg_ix < 0 && $scope.IMU_Sensor_Enabled && $scope.gps_horizontal_accuracy >= 30) {
            ahrs.messages = ahrs.messages.concat(MSG_GROUND_TEST);
        } else if (msg_ix >= 0 && $scope.gps_horizontal_accuracy < 30) {
            ahrs.messages.splice(msg_ix, MSG_GROUND_TEST.length);
        }

        msg_ix = ahrs.messages.indexOf(MSG_NO_AHRS[0]);
        if (msg_ix < 0 && !$scope.IMU_Sensor_Enabled && $scope.gps_horizontal_accuracy >= 30) {
            ahrs.messages = ahrs.messages.concat(MSG_NO_AHRS);
            ahrs.turn_off();
        } else if (msg_ix >= 0 && ($scope.IMU_Sensor_Enabled || $scope.gps_horizontal_accuracy < 30)) {
            ahrs.messages.splice(msg_ix, MSG_NO_AHRS.length);
            ahrs.turn_on();
        }

        $scope.press_time = Date.parse(situation.BaroLastMeasurementTime);
        $scope.gps_time   = Date.parse(situation.GPSLastGPSTimeStratuxTime);
        if ($scope.gps_time - $scope.press_time < 1000) {
            $scope.ahrs_alt = Math.round(situation.BaroPressureAltitude.toFixed(0));
        } else {
            $scope.ahrs_alt = "---";
        }

        $scope.ahrs_time = Date.parse(situation.AHRSLastAttitudeTime);
        if ($scope.gps_time - $scope.ahrs_time < 1000) {
            $scope.ahrs_heading = Math.round(situation.AHRSGyroHeading.toFixed(0));
            if ($scope.ahrs_heading > 360) {
                $scope.ahrs_heading = "---";
            } else if ($scope.ahrs_heading < 0.5) {
                $scope.ahrs_heading = 360;
            }
            $scope.ahrs_pitch = situation.AHRSPitch.toFixed(1);
            if ($scope.ahrs_pitch > 360) {
                $scope.ahrs_pitch = "--";
            }
            $scope.ahrs_roll = situation.AHRSRoll.toFixed(1);
            if ($scope.ahrs_roll > 360) {
                $scope.ahrs_roll = "--";
            }
            $scope.ahrs_slip_skid = situation.AHRSSlipSkid.toFixed(1);
            if ($scope.ahrs_slip_skid > 360) {
                $scope.ahrs_slip_skid = "--";
            }
            ahrs.update(situation.AHRSPitch, situation.AHRSRoll, situation.AHRSGyroHeading, situation.AHRSSlipSkid);

            $scope.ahrs_heading_mag = situation.AHRSMagHeading.toFixed(0);
            if ($scope.ahrs_heading_mag > 360) {
                $scope.ahrs_heading_mag = "---";
            }
            $scope.ahrs_gload = situation.AHRSGLoad.toFixed(2);
            if ($scope.ahrs_gload > 360) {
                $scope.ahrs_gload = "--";
            } else if (gMeter !== undefined) {
                gMeter.update(situation.AHRSGLoad, situation.AHRSGLoadMin, situation.AHRSGLoadMax);
            }

            if (situation.AHRSTurnRate > 360) {
                $scope.ahrs_turn_rate = "--";
            } else if (situation.AHRSTurnRate > 0.6031) {
                $scope.ahrs_turn_rate = (6 / situation.AHRSTurnRate).toFixed(1);
            } else {
                $scope.ahrs_turn_rate = '∞';
            }
        } else {
            $scope.ahrs_heading     = "---";
            $scope.ahrs_pitch       = "---";
            $scope.ahrs_roll        = "--";
            $scope.ahrs_slip_skid   = "--";
            $scope.ahrs_heading_mag = "---";
            $scope.ahrs_gload       = "--";
            $scope.ahrs_turn_rate   = "--";
        }

        if (situation.AHRSStatus & 0x01) {
            statusGPS.classList.remove("off");
            statusGPS.classList.add("on");
        } else {
            statusGPS.classList.add("off");
            statusGPS.classList.remove("on");
        }
        if (situation.AHRSStatus & 0x02) {
            statusIMU.classList.remove("off");
            statusIMU.classList.add("on");
        } else {
            statusIMU.classList.add("off");
            statusIMU.classList.remove("on");
        }
        if (situation.AHRSStatus & 0x04) {
            statusBMP.classList.remove("off");
            statusBMP.classList.remove("on");
            statusBMP.classList.remove("warn");
            if (situation.BaroSourceType == 4)
                statusBMP.classList.add("warn");
            else
                statusBMP.classList.add("on");
        } else {
            statusBMP.classList.remove("warn");
            statusBMP.classList.remove("on");
            statusBMP.classList.add("off");
        }
        if (situation.AHRSStatus & 0x08) {
            statusCal.classList.add("blink");
            statusCal.classList.remove("on");
            statusCal.innerText = "Caging";
            $scope.IsCaging = true;
        } else {
            statusCal.classList.remove("blink");
            statusCal.classList.add("on");
            statusCal.innerText = "Ready";
            $scope.IsCaging = false;
        }

        msg_ix = ahrs.messages.indexOf(MSG_LEVELING[0]);
        if (msg_ix < 0 && $scope.IsCaging) {
            ahrs.messages = ahrs.messages.concat(MSG_LEVELING);
            ahrs.turn_off();
        } else if (msg_ix >= 0 && !$scope.IsCaging) {
            ahrs.messages.splice(msg_ix, MSG_LEVELING.length);
            ahrs.turn_on();
        }

        $scope.IsPseudoAHRS = (!$scope.IMU_Sensor_Enabled && $scope.gps_horizontal_accuracy < 30);
        msg_ix = ahrs.messages.indexOf(MSG_PSEUDO_AHRS[0]);
        if (msg_ix < 0 && $scope.IsPseudoAHRS) {
            ahrs.messages = ahrs.messages.concat(MSG_PSEUDO_AHRS);
        } else if (msg_ix >= 0 && !$scope.IsPseudoAHRS) {
            ahrs.messages.splice(msg_ix, MSG_PSEUDO_AHRS.length);
        }
    }

    function resetSituation() {
        $scope.ahrs_heading     = "---";
        $scope.ahrs_pitch       = "--";
        $scope.ahrs_roll        = "--";
        $scope.ahrs_slip_skid   = "--";
        $scope.ahrs_heading_mag = "---";
        $scope.ahrs_turn_rate   = "--";
        $scope.ahrs_gload       = "--";
        statusGPS.classList.add("off");
        statusGPS.classList.remove("on");
        statusIMU.classList.add("off");
        statusIMU.classList.remove("on");
        statusBMP.classList.add("off");
        statusBMP.classList.remove("on");
        statusBMP.classList.remove("warn");
        statusCal.classList.add("off");
        statusCal.classList.remove("on");
        statusCal.innerText = "Error";
    }

    $state.get('ahrs').onEnter = function () {};

    $state.get('ahrs').onExit = function () {
        $scope.noSleep.disable();
        delete $scope.noSleep;

        if (($scope.socket !== undefined) && ($scope.socket !== null)) {
            $scope.socket.close();
            $scope.socket = null;
        }
    };

    var ahrs = new AHRSRenderer("ahrs_display");
    ahrs.turn_on();

    $scope.hideClick = function () {
        $scope.isHidden = !$scope.isHidden;
        var disp = "block";
        if ($scope.isHidden) {
            disp = "none";
            $scope.noSleep.enable();
        } else {
            $scope.noSleep.disable();
        }
        var hiders = document.querySelectorAll(".hider");
        for (var i = 0; i < hiders.length; i++) {
            hiders[i].style.display = disp;
        }
    };

    $scope.AHRSCage = function () {
        if (!$scope.IsCaging) {
            $http.post(URL_AHRS_CAGE).then(function (response) {
            }, function (response) {
                ahrs.messages = ahrs.messages.concat(response.data);
                window.setTimeout(function () {
                    ahrs.messages.splice(ahrs.messages.indexOf(response.data), 1);
                }, 1000);
            });
        }
    };

    $scope.AHRSCalibrate = function () {
        if (!$scope.IsCaging) {
            $http.post(URL_AHRS_CAL).then(function (response) {
            }, function (response) {
                ahrs.messages = ahrs.messages.concat(response.data);
                window.setTimeout(function () {
                    ahrs.messages.splice(ahrs.messages.indexOf(response.data), 1);
                }, 1000);
            });
        }
    };

    $scope.GMeterReset = function () {
        $http.post(URL_GMETER_RESET).then(function () {}, function () {});
    };

    function getAHRSSettings() {
        $http.get(URL_SETTINGS_GET).then(function (response) {
            var settings = angular.fromJson(response.data);
            $scope.IMU_Sensor_Enabled = settings.IMU_Sensor_Enabled;
            if (settings.GLimits === "" || settings.GLimits === undefined) {
                settings.GLimits = "-1.76 4.4";
            }
            var glims = settings.GLimits.split(" ");
            $scope.gLimNegative = parseFloat(glims[0]);
            $scope.gLimPositive = parseFloat(glims[1]);
            gMeter = new GMeterRenderer("gMeter_display", $scope.gLimNegative, $scope.gLimPositive, $scope.GMeterReset);
        }, function (response) {});
    }

    var gMeter;
    getAHRSSettings();
    connect($scope);
}

// ── GMeterRenderer ────────────────────────────────────────────────────────────

function GMeterRenderer(locationId, nlim, plim, resetCallback) {
    if (nlim > plim) {
        this.nlim = plim;
        this.plim = nlim;
    } else {
        this.nlim = nlim;
        this.plim = plim;
    }
    this.nticks = Math.floor(this.plim + 1) - Math.ceil(this.nlim - 1) + 1;

    this.width = -1;
    this.height = -1;

    this.locationId = locationId;
    this.canvas = document.getElementById(this.locationId);
    this.resize();

    // State variables
    this.g = 1;
    this.min = 1;
    this.max = 1;

    // Draw the G Meter using the svg.js library
    var gMeter = SVG(this.locationId).viewbox(-200, -200, 400, 400).group().addClass('gMeter');

    var el, card = gMeter.group().addClass('card');
    card.circle(390).cx(0).cy(0);
    card.line(-150, 0, -190, 0)
        .addClass('marks one');
    for (var i = Math.ceil(this.nlim - 1); i <= Math.floor(this.plim + 1); i++) {
        if (i % 2 === 0) {
            el = card.line(-150, 0, -190, 0).addClass('big');
            card.text(i.toString())
                .addClass('text')
                .cx(-105).cy(0)
                .transform({rotation: (i - 1) / this.nticks * 360, cx: 0, cy: 0, relative: true})
                .transform({rotation: -(i - 1) / this.nticks * 360, relative: true});
        } else {
            el = card.line(-165, 0, -190, 0);

        }
        el.addClass('marks')
            .rotate((i - 1) / this.nticks * 360, 0, 0);
    }
    card.line(-140, 0, -190, 0).addClass('marks limit').rotate((this.plim - 1) / this.nticks * 360, 0, 0);
    card.line(-140, 0, -190, 0).addClass('marks limit').rotate((this.nlim - 1) / this.nticks * 360, 0, 0);

    var ax = -Math.cos(2 * Math.PI / this.nticks),
        ay = -Math.sin(2 * Math.PI / this.nticks);
    card.path('M -175 0, A 175 175 0 0 1 ' + 175 * ax + ' ' + 175 * ay)
        .rotate(Math.floor(this.plim) / this.nticks * 360, 0, 0)
        .addClass('marks')
        .style('fill-opacity', '0');
    card.path('M -180 0, A 180 180 0 0 1 ' + 180 * ax + ' ' + 180 * ay)
        .rotate(Math.floor(this.plim) / this.nticks * 360, 0, 0)
        .addClass('marks')
        .style('fill-opacity', '0');

    this.min_el = gMeter.group().addClass('min');
    this.min_el.polygon('0,0 -170,0 -160,-5 0,-5').addClass('pointer');
    this.min_el.polygon('0,0 -170,0 -160,+5 0,+5').addClass('pointerBG');

    this.pointer_el = gMeter.group().addClass('g');
    this.pointer_el.polygon('0,0 -170,0 -150,-10 0,-10').addClass('pointer');
    this.pointer_el.polygon('0,0 -170,0 -150,+10 0,+10').addClass('pointerBG');

    this.max_el = gMeter.group().addClass('max');
    this.max_el.polygon('0,0 -170,0 -150,-5 0,-5').addClass('pointer');
    this.max_el.polygon('0,0 -170,0 -150,+5 0,+5').addClass('pointerBG');

    gMeter.circle(40).cx(0).cy(0).addClass('center');

    var reset = gMeter.group().cx(-165).cy(165).addClass('reset');
    reset.circle(60).cx(0).cy(0).addClass('reset');
    reset.text('RESET').cx(0).cy(0).addClass('text');
    reset.on('click', function () {
        reset.animate(200).rotate(20, 0, 0);
        resetCallback();
        reset.animate(200).rotate(0, 0, 0);
    }, this);
}

GMeterRenderer.prototype = {
    constructor: GMeterRenderer,

    resize: function () {
        var canvasWidth = this.canvas.parentElement.offsetWidth - 12;

        if (canvasWidth !== this.width) {
            this.width = canvasWidth;
            this.height = canvasWidth * 0.5;

            this.canvas.width = this.width;
            this.canvas.height = this.height;
        }
    },

    update: function (g, gmin, gmax) {
        this.g = g;
        this.min = gmin;
        this.max = gmax;

        this.pointer_el.rotate((g - 1) / this.nticks * 360, 0, 0);
        this.max_el.rotate((this.max - 1) / this.nticks * 360, 0, 0);
        this.min_el.rotate((this.min - 1) / this.nticks * 360, 0, 0);
    }
};
