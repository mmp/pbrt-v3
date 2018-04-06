var express = require("express")
var app = express()
var path = require("path")
var WebSocket = require("ws");
const {spawn} = require("child_process");
var randomInt = require("random-int");

var priv = {};

// Subprocess data
var prc = {}; // proc examples_count on_task_complete

// ============================================================================
// Constants

var EXPRESS_PORT = 32000;
var WS_PORT = 32001;

var rootdir = path.join(__dirname, "..", "..", "..");

// ============================================================================
// ExpressJS

app.get("/left", (req, res) => {
    res.sendFile(path.join(rootdir, "expected.png"));
});

app.get("/right", (req, res) => {
    res.sendFile(path.join(rootdir, "created.png"));
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

priv.handle_ws_message = function(ws, data) {
    var msgobj = JSON.parse(data);
    var t = msgobj._t;
    var handler_name = "handler_" + t;
    var handler = priv[handler_name];
    if (!handler) {
        console.info("No handler for type " + t);
        return;
    }
    handler(msgobj, ws);
};

wss.on("connection", function(ws) {
    ws.on("message", function(data) {
        priv.handle_ws_message(ws, data);
    });
});

// ============================================================================
// Main handlers

priv.handler_button_load = function(msgobj, ws) {
    console.info("Load");
    priv.start_python_subprocess();
};

priv.handler_button_next = function(msgobj, ws) {
    console.info("Next");
    // Generate random number
    var seq = randomInt(0, prc.examples_count - 1);
    // Register next subprocess callback
    prc.on_task_complete = function() {
        ws.send(JSON.stringify({
            _t: "task_complete"
        }));
    };
    // Write to process
    priv.python_subprocess_write("" + seq + "\n");
};

// ============================================================================
// Python process management

function emitLines (stream) {
    var backlog = ''
    stream.on('data', function (data) {
        backlog += data
        var n = backlog.indexOf('\n')
        // got a \n? emit one or more 'line' events
        while (~n) {
        stream.emit('line', backlog.substring(0, n))
        backlog = backlog.substring(n + 1)
        n = backlog.indexOf('\n')
        }
    })
    stream.on('end', function () {
        if (backlog) {
        stream.emit('line', backlog)
        }
    })
};

priv.start_python_subprocess = function() {
    var interactive_py_path = path.join(__dirname, "..", "..", "..", "ml", "main_interactive_view.py");
    prc.proc = spawn("python3", [interactive_py_path]);
    prc.proc.stdout.on("line", (line) => {
        console.info("STDOUT: ["+ line +"]");
        if (line.startsWith("Loaded")) {
            sp = line.split(" ");
            sp1 = sp[1];
            examples_count = parseInt(sp1);
            console.info("Parsed: examples count is " + examples_count);
            prc.examples_count = examples_count;
        } else if (line.startsWith("#EVALUATECOMPLETE")) {
            prc.on_task_complete();
        }
    });
    emitLines(prc.proc.stdout);
    prc.proc.stderr.on("line", (line) => {
        console.info("STDERR: ["+ line +"]");
    });
    prc.proc.on("close", function(code, signal) {
        console.info("Exited with ["+ code +"] ["+ signal +"]");
        prc.proc = null;
    });
};

priv.python_subprocess_write = function(data) {
    prc.proc.stdin.write(data);
};