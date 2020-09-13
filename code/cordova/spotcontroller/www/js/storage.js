var storage = {
    setItem: function (referenceName, object) {
        try {
            var objectAsString = JSON.stringify(object);
            window.localStorage.setItem(referenceName, objectAsString);
            debug.log('item stored: ' + referenceName + ': ' + objectAsString, 'success');
            return true;
        } catch (error) {
            console.log(error);
        }
    },
    getItem: function (referenceName, defaultValue) {
        try {
            var objectAsString = window.localStorage.getItem(referenceName);
            var object = JSON.parse(objectAsString);
            return (object) ? object : defaultValue;
        } catch (error) {
            console.log(error);
        }
    },
    removeItem: function (referenceName) {
        try {
            window.localStorage.removeItem(referenceName);
            debug.log('stored item removed: ' + referenceName, 'success');
            return true;
        } catch (error) {
            console.log(error);
        }
    }
}