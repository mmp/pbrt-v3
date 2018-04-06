var express = require("express")
var app = express()
var path = require("path")
var WebSocket = require("ws");

// ============================================================================
// Constants

var EXPRESS_PORT = 32000;
var WS_PORT = 32001;

// ============================================================================
// ExpressJS

app.get("/left", (req, res) => {
    res.send("Requested left");
});

app.get("/right", (req, res) => {
    res.send("Requested right");
});

app.use(express.static(path.join(__dirname, "..", "static")));

app.listen(EXPRESS_PORT, () => {
    console.info("Express started on port " + EXPRESS_PORT)
});

// ============================================================================
// WebSocket

var wss = new WebSocket.Server({ 
    port: WS_PORT
});

var handle_ws_message = function(ws, data) {
    var msgobj = JSON.parse(data);
    var t = msgobj._t;
    console.info("Got a ["+ t +"] message");
};

wss.on("connection", function(ws) {
    ws.on("message", function(data) {
        handle_ws_message(ws, data);
    });
});
