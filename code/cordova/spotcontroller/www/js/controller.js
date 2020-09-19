var controller = {
    useSonar: true,
    rotationRanges: {
        omega: 30,
        phi: 10,
        psi: 25,
        xm: -50,
        ym: -50,
        zm: 40,
    },
    Joy1 : null,
    Joy2: null,
    initialize: function () {
        debug.log("UUID " + device.uuid, "success");

        var joy1Param = { "title": "joystick1", "autoReturnToCenter": true };
        controller.Joy1 = new JoyStick('joy1Div', joy1Param);

        var joy2Param = { "title": "joystick2", "autoReturnToCenter": true };
        controller.Joy2 = new JoyStick('joy2Div', joy2Param);

        setInterval(controller.timer_callback, 500);
    },
    sendButton: function (button) { 
        var buttonvalue = button.charCodeAt(0);
        debug.log("Button " + button + " " + buttonvalue, "success");
        var data = new Uint8Array(2);
        data[0] = 1; // control command
        data[1] = buttonvalue; 
        bluetooth.sendData(data.buffer);
    },
    toggleSonar: function() {
        controller.useSonar = !controller.useSonar;
        debug.log("useSonar " + controller.useSonar, "success");

        var data = new Uint8Array(3);
        data[0] = 2; // options command
        data[1] = 1;  // sonar 
        data[2] = controller.useSonar; 
        bluetooth.sendData(data.buffer);

        if (controller.useSonar) {
            document.querySelector('#icon_sonar').setAttribute('icon', 'md-eye');
        } else {
            document.querySelector('#icon_sonar').setAttribute('icon', 'md-eye-off');
        }
    },
    timer_callback: function() {
        var Joy1X = controller.Joy1.GetPosX();
        var Joy1Y = controller.Joy1.GetPosY();
        var Joy2X = controller.Joy2.GetPosX();
        var Joy2Y = controller.Joy2.GetPosY();
        // console.log("Joy1 " + Joy1X + " " + Joy1Y);
        // console.log("Joy2 " + Joy2X + " " + Joy2Y);

        omega = (controller.rotationRanges.omega * (Joy1X - 100) / 50).toFixed(0)
        phi = 0;
        psi = (controller.rotationRanges.psi * (Joy1Y - 100) / 50).toFixed(0);
        zm = (controller.rotationRanges.xm * (Joy2X - 100) / 50).toFixed(0);
        xm = 0;
        ym = (controller.rotationRanges.ym * (Joy2Y - 100) / 50).toFixed(0);

        var statusLine = '<div class="log-item">' +
            '<div>POSITION: (' + Joy1X + ', ' + Joy1Y + ') (' + Joy2X + ', ' + Joy2Y + ')' +  
            ' -> (' + omega + ', ' + phi + ', ' + psi + ') (' + xm + ', ' + xm + ', ' + zm + ') </div>' +
            '</div>';

        $('#status').html(statusLine);

        // console.log("Rotation " + omega + " " + phi + " " + psi + " " + xm + " " + ym + " " + zm);
        bluetooth.sendOrientation(omega, phi, psi, xm, ym, zm);
    },
}