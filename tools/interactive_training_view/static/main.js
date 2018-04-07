var mainApp = angular.module("mainApp", []);

mainApp.controller("main_controller", function($scope) {

    var priv = {};

    // ------------------------------------------------------------------------
    // Websocket setup

    var ws = new WebSocket("ws://localhost:32001");

    ws.onopen = function() {
        console.info("Websocket connection opened");
    };

    ws.onmessage = function(evt) {
        var data = evt.data;
        var msgobj = JSON.parse(data);
        priv.handle_ws_message(msgobj);
    };

    ws.onclose = function() {
        console.error("Websocket disconnected");
    }

    priv.handle_ws_message = function(msgobj) {
        console.info("Received a ws message");
        console.info(msgobj);
        var t = msgobj._t;
        if (t == "task_complete") {
            priv.image_refresh_left();
            priv.image_refresh_right();
        }
    };

    priv.send = function(msgobj) {
        var data = JSON.stringify(msgobj);
        ws.send(data);
    };

    // ------------------------------------------------------------------------
    // Buttons

    $scope.button_load = function() {
        console.info("Load button pressed");
        priv.send({
            _t: "button_load"
        });
    };

    $scope.button_next = function() {
        console.info("Next button pressed");
        priv.send({
            _t: "button_next"
        });
    };

    // ------------------------------------------------------------------------
    // Images

    priv.image_refresh_left = function() {
        priv.image_refresh_by_id("main_image_left", "left");
    };

    priv.image_refresh_right = function() {
        priv.image_refresh_by_id("main_image_right", "right");
    };

    priv.image_refresh_by_id = function(elem_id, image_source) {
        var elem = document.getElementById(elem_id);
        var new_source = image_source + "?" + new Date().getTime();
        elem.src = new_source;
    };

});