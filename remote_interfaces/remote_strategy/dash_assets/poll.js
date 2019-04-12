// Function to fire an event on a DOM object
function eventFire(el, etype){
  if (el.fireEvent) {
    el.fireEvent('on' + etype);
  } else {
    var evObj = document.createEvent('Events');
    evObj.initEvent(etype, true, false);
    el.dispatchEvent(evObj);
  }
}

// After a set timeout, make sure to check for an update to the enabled status.
// Trigger a series of updates only IFF there has been an update to the enabled
// status
var enabled = false;

function check_enabled() {
    // Send an AJAX request to see if status has updated on the remote server
    fetch('/ros_api/enabled')
        .then(function(data) {
            return data.json();
        })
        .then(function(data) {
            if (data.enabled !== enabled) {
                eventFire(document.getElementById('enable-component'), 'click');
            }
            enabled = data.enabled;
            return null;
        })
        .catch(console.error);
}

// Poll for a status update every second
var poll_interval = setInterval(check_enabled, 1000);
