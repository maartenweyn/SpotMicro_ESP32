var debug = {
    print: function (message, type) {
        message = (typeof (message) == 'object') ? JSON.stringify(message) : message;
        var messageColor = "black";
        switch (type) {
            case "error":
                messageColor = "red";
                break;
            case "success":
                messageColor = "green";
                break;
        }
        var messageLine = '<div class="log-item">' +
            '<div class="timestamp">' + moment().format() + '</div>' +
            '<div style="color: ' + messageColor + ';">' + message + '</div>' +
            '</div>';
        $('#logs').prepend(messageLine);
    },
    log: function (message, type) {
        console.log(message);
        this.print(message, type);
    },
    clear: function () {
        $('#debugList').empty();
    }
}