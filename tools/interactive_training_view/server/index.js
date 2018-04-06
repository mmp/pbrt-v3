var express = require("express")
var app = express()
var path = require("path")

// ============================================================================
// Constants

var EXPRESS_PORT = 32000;

// ============================================================================
// ExpressJS

app.use(express.static(path.join(__dirname, "..", "static")));

app.listen(EXPRESS_PORT, () => {
    console.info("Express started on port " + EXPRESS_PORT)
});