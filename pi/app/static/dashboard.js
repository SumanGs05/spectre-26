/**
 * UAV-AudioLoc Dashboard — Real-time polar plot and telemetry display.
 *
 * Connects via WebSocket to the Pi server and renders:
 * - Polar plot with azimuth needle and confidence ring
 * - Detection history trail (fading dots)
 * - Numeric readouts for all telemetry fields
 * - Detection log table
 */

(function () {
    "use strict";

    const canvas = document.getElementById("polar-plot");
    const ctx = canvas.getContext("2d");

    const azimuthEl = document.getElementById("azimuth-value");
    const confidenceBar = document.getElementById("confidence-bar");
    const confidenceVal = document.getElementById("confidence-value");
    const snrVal = document.getElementById("snr-value");
    const classVal = document.getElementById("class-value");
    const classConfVal = document.getElementById("class-conf-value");
    const detectionInd = document.getElementById("detection-indicator");
    const frameVal = document.getElementById("frame-value");
    const connStatus = document.getElementById("connection-status");
    const logBody = document.getElementById("log-body");

    const MAX_TRAIL = 60;
    const trail = [];

    let currentAzimuth = 0;
    let currentConfidence = 0;
    let ws = null;
    let reconnectTimer = null;

    // --- Polar Plot Rendering ---

    function drawPolarPlot() {
        const w = canvas.width;
        const h = canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        const radius = Math.min(cx, cy) - 40;

        ctx.clearRect(0, 0, w, h);

        // Background
        ctx.fillStyle = "#111827";
        ctx.fillRect(0, 0, w, h);

        // Concentric rings
        ctx.strokeStyle = "rgba(148, 163, 184, 0.15)";
        ctx.lineWidth = 1;
        for (let r = 0.25; r <= 1; r += 0.25) {
            ctx.beginPath();
            ctx.arc(cx, cy, radius * r, 0, Math.PI * 2);
            ctx.stroke();
        }

        // Cardinal direction lines and labels
        ctx.strokeStyle = "rgba(148, 163, 184, 0.2)";
        ctx.lineWidth = 1;
        const labels = [
            { angle: -90, text: "NOSE" },
            { angle: 0, text: "90" },
            { angle: 90, text: "180" },
            { angle: 180, text: "270" },
        ];

        for (const { angle, text } of labels) {
            const rad = (angle * Math.PI) / 180;
            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + radius * Math.cos(rad), cy + radius * Math.sin(rad));
            ctx.stroke();

            ctx.fillStyle = "#64748b";
            ctx.font = "bold 11px system-ui";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            const labelR = radius + 20;
            ctx.fillText(text, cx + labelR * Math.cos(rad), cy + labelR * Math.sin(rad));
        }

        // 30-degree tick marks
        ctx.strokeStyle = "rgba(148, 163, 184, 0.1)";
        for (let deg = 0; deg < 360; deg += 30) {
            const rad = ((deg - 90) * Math.PI) / 180;
            ctx.beginPath();
            ctx.moveTo(cx + radius * 0.92 * Math.cos(rad), cy + radius * 0.92 * Math.sin(rad));
            ctx.lineTo(cx + radius * Math.cos(rad), cy + radius * Math.sin(rad));
            ctx.stroke();
        }

        // Detection trail (fading history)
        for (let i = 0; i < trail.length; i++) {
            const t = trail[i];
            const age = i / trail.length;
            const alpha = 0.1 + 0.4 * age;
            const r = radius * Math.min(t.confidence, 1);
            const rad = ((t.azimuth - 90) * Math.PI) / 180;
            const x = cx + r * Math.cos(rad);
            const y = cy + r * Math.sin(rad);

            ctx.beginPath();
            ctx.arc(x, y, 3, 0, Math.PI * 2);
            ctx.fillStyle = `rgba(6, 182, 212, ${alpha})`;
            ctx.fill();
        }

        // Current azimuth needle
        if (currentConfidence > 0.05) {
            const needleRad = ((currentAzimuth - 90) * Math.PI) / 180;
            const needleLen = radius * Math.min(currentConfidence, 1);

            // Glow
            ctx.shadowColor = "#22c55e";
            ctx.shadowBlur = 12;

            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(cx + needleLen * Math.cos(needleRad), cy + needleLen * Math.sin(needleRad));
            ctx.strokeStyle = "#22c55e";
            ctx.lineWidth = 3;
            ctx.stroke();

            // Arrowhead
            const tipX = cx + needleLen * Math.cos(needleRad);
            const tipY = cy + needleLen * Math.sin(needleRad);
            ctx.beginPath();
            ctx.arc(tipX, tipY, 6, 0, Math.PI * 2);
            ctx.fillStyle = "#22c55e";
            ctx.fill();

            ctx.shadowBlur = 0;
        }

        // Center dot
        ctx.beginPath();
        ctx.arc(cx, cy, 4, 0, Math.PI * 2);
        ctx.fillStyle = "#94a3b8";
        ctx.fill();
    }

    // --- WebSocket Connection ---

    function connect() {
        const protocol = location.protocol === "https:" ? "wss:" : "ws:";
        const url = `${protocol}//${location.host}/ws/telemetry`;

        ws = new WebSocket(url);

        ws.onopen = function () {
            connStatus.textContent = "CONNECTED";
            connStatus.className = "status-badge connected";
        };

        ws.onmessage = function (event) {
            const data = JSON.parse(event.data);
            if (data.heartbeat) return;
            handleTelemetry(data);
        };

        ws.onclose = function () {
            connStatus.textContent = "DISCONNECTED";
            connStatus.className = "status-badge disconnected";
            scheduleReconnect();
        };

        ws.onerror = function () {
            ws.close();
        };
    }

    function scheduleReconnect() {
        if (reconnectTimer) clearTimeout(reconnectTimer);
        reconnectTimer = setTimeout(connect, 2000);
    }

    // --- Telemetry Processing ---

    function handleTelemetry(pkt) {
        currentAzimuth = pkt.azimuth_deg || 0;
        currentConfidence = pkt.confidence || 0;

        azimuthEl.textContent = pkt.detection
            ? pkt.azimuth_deg.toFixed(1)
            : "---";

        confidenceBar.style.width = (pkt.confidence * 100) + "%";
        confidenceBar.style.background = pkt.confidence > 0.5 ? "#22c55e" : "#f59e0b";
        confidenceVal.textContent = pkt.confidence.toFixed(2);

        snrVal.textContent = pkt.snr_db.toFixed(1) + " dB";

        classVal.textContent = pkt.sound_class || "None";
        classConfVal.textContent = pkt.class_confidence.toFixed(2);

        detectionInd.className = "indicator " + (pkt.detection ? "on" : "off");

        frameVal.textContent = pkt.frame_id;

        // Update trail
        if (pkt.detection && pkt.confidence > 0.1) {
            trail.push({ azimuth: pkt.azimuth_deg, confidence: pkt.confidence });
            if (trail.length > MAX_TRAIL) trail.shift();
        }

        drawPolarPlot();

        // Log detections
        if (pkt.detection) {
            addLogEntry(pkt);
        }
    }

    function addLogEntry(pkt) {
        const row = document.createElement("tr");
        const time = new Date(pkt.timestamp_ms).toLocaleTimeString();
        row.innerHTML = `
            <td>${time}</td>
            <td>${pkt.azimuth_deg.toFixed(1)}&deg;</td>
            <td>${pkt.confidence.toFixed(2)}</td>
            <td>${pkt.sound_class}</td>
            <td>${pkt.snr_db.toFixed(1)}</td>
        `;

        const body = logBody;
        body.insertBefore(row, body.firstChild);

        // Keep last 100 entries
        while (body.children.length > 100) {
            body.removeChild(body.lastChild);
        }
    }

    // --- Init ---

    drawPolarPlot();
    connect();

})();
