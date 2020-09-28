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
    currentRotation: {
        omega: 0,
        phi: 0,
        psi: 0,
        xm: 0,
        ym: 0,
        zm: 0,
    },
    Joy1 : null,
    Joy2: null,
    slider1: null,
    slider2: null,
    sleep: false,
    initialize: function () {
        debug.log("UUID " + device.uuid, "success");

        var joy1Param = { "title": "joystick1", "autoReturnToCenter": true };
        controller.Joy1 = new JoyStick('joy1Div', joy1Param);

        var joy2Param = { "title": "joystick2", "autoReturnToCenter": true };
        controller.Joy2 = new JoyStick('joy2Div', joy2Param);

        controller.slider1 = document.getElementById('slider1Div');
        controller.slider2 = document.getElementById('slider2Div');
        controller.slider1.addEventListener("touchend", controller.resetslider1, false);
        controller.slider2.addEventListener("touchend", controller.resetslider2, false);

        sleepwakebutton = document.getElementById('sleep_wake_button');
        sleepwakebutton.onclick = controller.sleepwake;

        setInterval(controller.timer_callback, 500);
    },
    resetslider1: function() {
        controller.slider1.value = 50;
    },
    resetslider2: function() {
        controller.slider2.value = 50;
    },
    sleepwake: function() {
        controller.sleep = !controller.sleep;
        if (controller.sleep) {
            $('#sleep_wake_button').html("WAKE");
            bluetooth.sendOrientation(0, 0, 0, -40, -170, 0);
        } else {
            $('#sleep_wake_button').html("SLEEP");
            bluetooth.sendOrientation(0, 0, 0, 0, 0, 0);
        }
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
        if (controller.sleep) return;

        var Joy1X = controller.Joy1.GetPosX();
        var Joy1Y = controller.Joy1.GetPosY();
        var Joy2X = controller.Joy2.GetPosX();
        var Joy2Y = controller.Joy2.GetPosY();
        // console.log("Joy1 " + Joy1X + " " + Joy1Y);
        // console.log("Joy2 " + Joy2X + " " + Joy2Y);
        var slider1 = controller.slider1.value;
        var slider2 = controller.slider2.value;

        omega = (controller.rotationRanges.omega * (Joy1X - 100) / 50).toFixed(0)
        phi = (controller.rotationRanges.phi * (slider1 - 50) / 50).toFixed(0);
        psi = (controller.rotationRanges.psi * (Joy1Y - 100) / 50).toFixed(0);
        zm = (controller.rotationRanges.xm * (Joy2X - 100) / 50).toFixed(0);
        xm = (controller.rotationRanges.xm * (slider2 - 50) / 50).toFixed(0);
        ym = (controller.rotationRanges.ym * (Joy2Y - 100) / 50).toFixed(0);

        var statusLine = '<div>' +
            // '<div>POSITION: (' + (Joy1X) + ', ' + (Joy1Y) + ') (' + (Joy2X) + ', ' + (Joy2Y) + ')' +  
            // ' -> (' + omega + ', ' + phi + ', ' + psi + ') (' + xm + ', ' + ym + ', ' + zm + ') </div>' +
            '<div>POSITION: (' + omega + ', ' + phi + ', ' + psi + ') (' + xm + ', ' + ym + ', ' + zm + ') </div>' + 
            '</div>';

        $('#status').html(statusLine);

        if ((omega != controller.currentRotation.omega) ||
            (phi != controller.currentRotation.phi) ||
            (psi != controller.currentRotation.psi) ||
            (zm != controller.currentRotation.zm) ||
            (xm != controller.currentRotation.xm) ||
            (ym != controller.currentRotation.ym)) {
                // console.log("Rotation " + omega + " " + phi + " " + psi + " " + xm + " " + ym + " " + zm);
                bluetooth.sendOrientation(omega, phi, psi, xm, ym, zm);
                controller.currentRotation.omega = omega;
                controller.currentRotation.phi = phi;
                controller.currentRotation.psi = psi;
                controller.currentRotation.zm = zm;
                controller.currentRotation.xm = xm;
                controller.currentRotation.ym = ym;
            }


    },
}