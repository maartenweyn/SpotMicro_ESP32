var bluetooth = {
    serviceUuids: { // 
        serviceUUID: "00FF",
        orientationSetValueCharacteristic: "FF01",
        orientationActuatorValueCharacteristic: "FF02",
        orientationSensorValueCharacteristic: "FF03", 
    },
    writeWithoutResponse: true,
    connectedDevice: {},
    connectingDevice: {},
    lastConnectedDeviceId: false,
    connected: false,
    messages: [],
    currentmessage: new Uint8Array(100),
    currentmessagepointer: 0,
    background_timer_settings: {
        timerInterval: 30000, // interval between ticks of the timer in milliseconds (Default: 60000)
        startOnBoot: false, // enable this to start timer after the device was restarted (Default: false)
        stopOnTerminate: true, // set to true to force stop timer in case the app is terminated (User closed the app and etc.) (Default: true)
        hours: -1, // delay timer to start at certain time (Default: -1)
        minutes: -1, // delay timer to start at certain time (Default: -1)
    },
    initialize: function () {
        debug.log('Initialising bluetooth ...');
        bluetooth.refreshDeviceList();
        debug.log('Bluetooth Initialised', 'success');
        window.BackgroundTimer.onTimerEvent(bluetooth.timer_callback);
        window.BackgroundTimer.start(bluetooth.timerstart_successCallback, bluetooth.timerstart_errorCallback, bluetooth.background_timer_settings);

        //autoconnect

        // var previousConnectedDevice = storage.getItem('connectedDevice');
        // if (previousConnectedDevice != undefined) {
        //     ble.autoConnect(previousConnectedDevice.id, bluetooth.onConnect, bluetooth.onDisconnectDevice);
        // }
    },
    refreshDeviceList: function () {
        var onlyCorrectCharacteristic = true;
        $('#ble-found-devices').empty();
        var characteristics = (onlyCorrectCharacteristic) ? [bluetooth.serviceUuids.serviceUUID] : [];
        console.log("Scanning for Spot");


        var onSuccess = function(position) {
            console.log('Latitude: '          + position.coords.latitude          + '\n' +
                  'Longitude: '         + position.coords.longitude         + '\n' +
                  'Altitude: '          + position.coords.altitude          + '\n' +
                  'Accuracy: '          + position.coords.accuracy          + '\n' +
                  'Altitude Accuracy: ' + position.coords.altitudeAccuracy  + '\n' +
                  'Heading: '           + position.coords.heading           + '\n' +
                  'Speed: '             + position.coords.speed             + '\n' +
                  'Timestamp: '         + position.timestamp                + '\n');
        };


        // hack for android 10 with current ble plugin
        //navigator.geolocation.getCurrentPosition(onSuccess, app.onError);
        navigator.geolocation.getCurrentPosition(function() {}, app.onError);


        ble.scan(characteristics, 5, bluetooth.onDiscoverDevice, app.onError);

    },
    onDiscoverDevice: function (device) {
        console.log("onDiscoverDevice " + device.name + " " + device.id);
        var previousConnectedDevice = storage.getItem('connectedDevice');
        // console.log("previousConnectedDevice " + previousConnectedDevice);

        if (previousConnectedDevice != undefined)
            debug.log('previousConnectedDevice ' + previousConnectedDevice.name, 'success');
        else
            debug.log('no previousConnectedDevice ', 'error');

        //if (device.name.toLowerCase().replace(/[\W_]+/g, "").indexOf('cme') > -1) {
        var html = '<ons-list-item modifier="chevron" data-device-id="' + device.id + '" data-device-name="' + device.name + '" tappable>' +
            '<span class="list-item__title">' + device.name + '</span>' +
            '<span class="list-item__subtitle">' + device.id + '</span>' +
            '</ons-list-item>';

        $('#ble-found-devices').append(html);
        //}

        if (previousConnectedDevice) {
            if (device.id == previousConnectedDevice.id) {
                debug.log('discovered previous device ' + previousConnectedDevice.name, 'success');
                bluetooth.connectDevice(previousConnectedDevice.id, previousConnectedDevice.name);
            }
        }

    },
    onConnect: function (peripheral) {
        bluetooth.connectedDevice = {
            id: bluetooth.connectingDevice.id,
            name: bluetooth.connectingDevice.name
        };

        console.log("onConnect " + bluetooth.connectedDevice.id);

        var characteristics = peripheral.characteristics;
        console.log(JSON.stringify(characteristics));

        // used to send disconnected messages 
        bluetooth.lastConnectedDedviceId = bluetooth.connectedDevice.id;

        storage.setItem('connectedDevice', bluetooth.connectedDevice);

        // subscribe for incoming data
        // ble.startNotification(bluetooth.connectedDevice.id,
        //     bluetooth.serviceUuids.serviceUUID,
        //     bluetooth.serviceUuids.orientationActuatorValueCharacteristic,
        //     bluetooth.onActuatorValue,
        //     bluetooth.onError);

        ble.read(bluetooth.connectedDevice.id,
            bluetooth.serviceUuids.serviceUUID,
            bluetooth.serviceUuids.orientationActuatorValueCharacteristic,
            bluetooth.onActuatorValue,
            bluetooth.onError);

        ble.startNotification(bluetooth.connectedDevice.id,
            bluetooth.serviceUuids.serviceUUID,
            bluetooth.serviceUuids.orientationActuatorValueCharacteristic,
            bluetooth.onActuatorValue,
            bluetooth.onError);

        // ble.read(device_id, service_uuid, characteristic_uuid, success, failure);

        // ble.startNotification(bluetooth.connectedDevice.id,
        //     bluetooth.serviceUuids.serviceUUID,
        //     bluetooth.serviceUuids.orientationSensorValueCharacteristic,
        //     bluetooth.onSensorValue,
        //     bluetooth.onError)

        debug.log('Connected to ' + bluetooth.connectedDevice.id, 'success');
        //mqttclient.addMessage('device,1');

        bluetooth.toggleConnectionButtons();
        window.BackgroundTimer.stop(bluetooth.timerstop_successCallback, bluetooth.timerstop_errorCallback);

        //bluetooth.sendTime();
        bluetooth.connected = true;
    },
    connectDevice: function (deviceId, deviceName) {
        bluetooth.connectingDevice = {
            id: deviceId,
            name: deviceName
        };
        debug.log('connecting to ' + deviceId);
        ble.connect(deviceId, bluetooth.onConnect, bluetooth.onError);
    },
    sendData(data) {
        ble.write(bluetooth.connectedDevice.id, bluetooth.serviceUuids.serviceUUID, bluetooth.serviceUuids.orientationSetValueCharacteristic, data, bluetooth.onSend, bluetooth.onError);
    },
    sendOrientation(omega, phi, psi, x, y, z) {
        var data = new Int16Array(6);
        data[0] = omega;
        data[1] = phi;
        data[2] = psi;
        data[3] = x;
        data[4] = y;
        data[5] = z;
        if (bluetooth.connected) {
            //console.log("Sending data");
            ble.write(bluetooth.connectedDevice.id, bluetooth.serviceUuids.serviceUUID, bluetooth.serviceUuids.orientationSetValueCharacteristic, data.buffer, bluetooth.onSend, bluetooth.onError);
        };
    },
    onDisconnectDevice: function () {
        debug.log('Disconnected from ' + bluetooth.lastConnectedDeviceId, 'success');
        bluetooth.connectedDevice = {};
        bluetooth.toggleConnectionButtons();
        bluetooth.connected = false;
    },
    disconnectDevice: function (event) {
        debug.log('Disconnecting from ' + bluetooth.connectedDevice.id);

        try {
            ble.disconnect(bluetooth.connectedDevice.id, bluetooth.onDisconnectDevice, bluetooth.onError);
            bluetooth.toggleConnectionButtons();
        } catch (error) {
            debug.log('Disconnecting failed', 'error');
            console.log(error);
        }
    },
    onSend: function () {
        debug.log('Has send data', 'success');
    },
    onActuatorValue: function (data) {
        //data = new Uint8Array(data);
        var stringdata = toHexString(data, 12);
        
        debug.log("Actuator: " + stringdata);
    },
    onSensorValue: function (data) {
        // data = new Uint8Array(data);
        var stringdata = toHexString(data, 6);
        
        debug.log("Sensor: " + stringdata);
    },
    timer_callback: function() {
        ble.isConnected(bluetooth.connectedDevice.id, function () {
            window.BackgroundTimer.stop(bluetooth.timerstop_successCallback, bluetooth.timerstop_errorCallback);
        }, function () {
            bluetooth.refreshDeviceList();
        });
    },
    timerstart_successCallback: function() {
        debug.log("BLE: timer started", 'success');
    },
    timerstart_errorCallback: function(e) {
        debug.log("BLE error: could not start timer", 'error');
    },
    timerstop_successCallback: function() {
        debug.log("BLE: timer stopped", 'success');
    },
    timerstop_errorCallback: function(e) {
        debug.log("BLE error: could not stop timer", 'error');
    },
    onError: function (reason) {
        debug.log("BLE error: " + JSON.stringify(reason), 'error');
        ble.isConnected(bluetooth.connectedDevice.id, function () {
            debug.log('error, but still connected');
        }, function () {
            bluetooth.connectedDevice = {};
            debug.log('error and disconnected from ' + bluetooth.lastConnectedDeviceId, 'success');
            bluetooth.toggleConnectionButtons();
            //window.BackgroundTimer.start(bluetooth.timerstart_successCallback, bluetooth.timerstart_errorCallback, bluetooth.background_timer_settings);
        });
    },
    toggleConnectionButtons: function () {
        var connected = (bluetooth.connectedDevice.id !== undefined);
        console.log('current ble connection status: ' + ((connected) ? 'connected' : 'not connected'));

        if (connected) {
            var html = '<ons-list-item>' +
                '<span class="list-item__title">' + bluetooth.connectedDevice.name + '</span>' +
                '<span class="list-item__subtitle">' + bluetooth.connectedDevice.id + '</span>' +
                '</ons-list-item>';
            $('#ble-connected-device').html(html);

            $('.ble-not-connected').hide();
            $('.ble-connected').show();
        } else {
            $('#ble-connected-device').html('no device connected');
            $('.ble-not-connected').show();
            $('.ble-connected').hide();
        }
    },
    refreshSentMessageList: function () {
        $('#ble-received-messages').empty();

        if (bluetooth.messages.length > 20) {
            bluetooth.messages.shift();
        }

        $.each(bluetooth.messages, function (index, data) {
            var messageLine = '<ons-list-item>' +
                '<span class="list-item__title">' + data.data + '</span>' +
                '<span class="list-item__subtitle">' + data.timestamp + '</span>' +
                '</ons-list-item>';

            $('#ble-received-messages').prepend(messageLine);
        });

    }
};

/*
helpers
*/
// ASCII only
function bytesToString(buffer) {
    return String.fromCharCode.apply(null, new Uint8Array(buffer));
}


function toHexString(buffer) {
    var data = new Uint8Array(buffer);
    var resultstring = "";
    for (var i = 0, l = data.length; i < l; i++) {
        var string = (new Number(data[i])).toString(16);
        while(string.length < 2) {
            string = '0'+string;
        }
        resultstring += string;
    }
	return resultstring;
}

// ASCII only
function stringToBytes(string) {
    var array = new Uint8Array(string.length);
    for (var i = 0, l = string.length; i < l; i++) {
        array[i] = string.charCodeAt(i);
    }
    return array.buffer;
}