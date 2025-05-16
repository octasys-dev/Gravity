// server-udp.js

const express = require("express");
const dgram = require("dgram");
const path = require("path");

const app = express();
const PORT = 8080;

const enablePrint = process.argv.includes("-p");

// Create UDP socket
const udp = dgram.createSocket("udp4");

// Init fallback values
let lastKnownCoords = {};
for (let i = 0; i < 11; i++) {
  lastKnownCoords[`id${i}`] = [0, 0];
}

// Handle incoming UDP messages
udp.on("message", (msg) => {
  const text = msg.toString().trim().replace(/[;,]/g, "");
  const values = text.split(/\s+/).map(Number);

  if (values.length === 22 && values.every(n => !isNaN(n))) {
    const newCoords = {};
    for (let i = 0; i < 11; i++) {
      newCoords[`id${i}`] = [values[i * 2], values[i * 2 + 1]];
    }

    lastKnownCoords = newCoords;

    if (enablePrint) {
      const line = Object.entries(newCoords)
        .map(([id, [x, y]]) => `${id}:x=${x.toFixed(2)} y=${y.toFixed(2)}`)
        .join(" | ");
      console.log(line);
    }
  } else {
    console.warn("Invalid UDP data received:", text);
  }
});

// Start UDP server
udp.bind(5000, () => {
  console.log("UDP listener active on port 5000");
});

// Serve frontend
app.use(express.static(path.join(__dirname)));

// REST-API: get latest coords
app.get("/data", (req, res) => {
  res.json(lastKnownCoords);
});

// Start HTTP server
app.listen(PORT, () => {
  console.log(`Start frontend: http://localhost:${PORT}`);
});
