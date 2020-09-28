var app = {
    user: {},
    debug: false,
    currentView: 0, // 0: controls, 1: ble
    initialize: function () {
        console.log('app initialize');
        document.addEventListener('deviceready', this.onDeviceReady.bind(this), false);
    },

    onDeviceReady: function () {
        debug.log('device ready', 'success');
        app.bindEvents();
        bluetooth.initialize();
        controller.initialize();

        bluetooth.toggleConnectionButtons();

        $('#headerbar').on('click', '#ble_button', function (e) {
            if (app.currentView == 0) {
                $('#device').show();
                $('#controls').hide();
                app.currentView  = 1;
            } else {
                $('#device').hide();
                $('#controls').show();
                app.currentView  = 0;
            }
        });

        $('#device').on('click', '#refreshDeviceList', function (e) {
                bluetooth.refreshDeviceList();
        });
        $('#ble-found-devices').on('click', 'ons-list-item', function (e) {
                bluetooth.connectDevice($(this).attr("data-device-id"), $(this).attr("data-device-name"));
        });
        $('#device').on('click', '#disconnectDevice', function (e) {
                bluetooth.disconnectDevice(e);
        });
    },

    bindEvents: function () {
        // setTimeout(function () {
        //     mqttclient.addMessage('app,1');
        // }, 3000);

        document.addEventListener("pause", app.onDevicePause, false);
        document.addEventListener("resume", app.onDeviceResume, false);
        document.addEventListener("menubutton", app.onMenuKeyDown, false);
    },

    onDevicePause: function () {
        debug.log('in pause');
    },
    onDeviceResume: function () {
        debug.log('out of pause');
        bluetooth.refreshDeviceList();
    },
    onMenuKeyDown: function () {
        debug.log('menubuttonpressed');
    },
    onError: function (error) {
        debug.log(JSON.stringify(error), 'error');
    }
};

app.initialize();