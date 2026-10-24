const express = require('express');
const http = require('http');
const mqtt = require('mqtt');
const { WebSocketServer } = require('ws');
const path = require('path');

const app = express();
// Puerto HTTP configurable: node server.js [mqtt_url] [http_port]
const PORT = parseInt(process.argv[3] || process.env.HMI_PORT || '8080', 10);

// IP del broker por argumento: node server.js mqtt://<IP>:1883
// Orden de prioridad: argv[2] > MQTT_BROKER env > localhost
const BROKER_URL = process.argv[2] || process.env.MQTT_BROKER || 'mqtt://127.0.0.1:1883';

const server = http.createServer(app);
const wss = new WebSocketServer({ server });

app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));
app.use('/node_modules', express.static(path.join(__dirname, 'node_modules')));

app.get('/dictionary.json', (req, res) => {
    res.sendFile(path.join(__dirname, 'dictionary.json'));
});

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

let connectedClients = [];

wss.on('connection', (ws) => {
    connectedClients.push(ws);
    ws.on('close', () => {
        connectedClients = connectedClients.filter(client => client !== ws);
    });
});

// Helper: preserva el 0 real (?? en vez de ||) y sanea NaN/Infinity.
function num(v, fallback = 0.0) {
    if (v === null || v === undefined) return fallback;
    const n = Number(v);
    return Number.isFinite(n) ? n : fallback;
}

// Desenvuelve el doble anidado real de jetson/server.py:
// {"rover_sensors": {"sensores": {...}, "last_update": ...}}
function getSensorBlock(payload) {
    if (!payload || typeof payload !== 'object') return {};
    if (payload.rover_sensors && typeof payload.rover_sensors === 'object') {
        return payload.rover_sensors.sensores || payload.rover_sensors;
    }
    return payload.sensores || payload;
}

function getRoverState(payload) {
    if (!payload || typeof payload !== 'object') return {};
    return payload.rover_state || payload;
}

const mqttClient = mqtt.connect(BROKER_URL, {
    reconnectPeriod: 2000,
    connectTimeout: 5000,
    clientId: 'HMI_rover_mct_' + Math.random().toString(16).slice(2, 8)
});

console.log(`🔌 Conectando a broker MQTT: ${BROKER_URL}`);

const seenTopics = new Set();

mqttClient.on('connect', () => {
    console.log(`Conectado al Broker MQTT en ${BROKER_URL}.`);
    mqttClient.subscribe('rover/odometry', { qos: 0 });
    mqttClient.subscribe('rover/sensors', { qos: 0 });
    mqttClient.subscribe('rover/telemetry', { qos: 0 });
    mqttClient.subscribe('rover/status', { qos: 1 });
    mqttClient.subscribe('rover/commands/response', { qos: 1 });
});

mqttClient.on('reconnect', () => console.log(`[MQTT] Reintentando ${BROKER_URL}...`));
mqttClient.on('error', (err) => console.error(`[MQTT] Error: ${err.message}`));
mqttClient.on('offline', () => console.log('[MQTT] Offline.'));
mqttClient.on('close', () => console.log('[MQTT] Conexión cerrada.'));

function broadcast(telemetryData, timestamp) {
    Object.keys(telemetryData).forEach((key) => {
        const packetStr = JSON.stringify({ id: key, value: telemetryData[key], utc: timestamp });
        connectedClients.forEach(client => {
            if (client.readyState === WebSocket.OPEN) client.send(packetStr);
        });
    });
}

mqttClient.on('message', (topic, message) => {
    try {
        const raw = message.toString();

        if (topic === 'rover/status' || topic === 'rover/commands/response') {
            console.log(`[${topic}]: ${raw}`);
            return;
        }

        const payload = JSON.parse(raw);
        const timestamp = Date.now();
        let telemetryData = {};

        if (!seenTopics.has(topic)) {
            seenTopics.add(topic);
            console.log(`[MQTT] Primer paquete en ${topic}: ${raw.slice(0, 300)}`);
        }

        if (topic === 'rover/odometry') {
            telemetryData['pos_x'] = num(payload.x);
            telemetryData['pos_y'] = num(payload.y);
            telemetryData['pos_theta'] = num(payload.theta);
            telemetryData['vel_v'] = num(payload.v);
            telemetryData['vel_omega'] = num(payload.omega);
        } 
        else if (topic === 'rover/sensors') {
            // peso vive AQUI (kg), no en rover/telemetry
            const s = getSensorBlock(payload);
            telemetryData['pitch'] = num(s.pitch);
            telemetryData['heading'] = num(s.heading);
            telemetryData['peso'] = num(s.peso);
        } 
        else if (topic === 'rover/telemetry') {
            const state = getRoverState(payload);
            
            if (state.left_side && Array.isArray(state.left_side.motors)) {
                telemetryData['rpm_m_izq1'] = num(state.left_side.motors[0]?.rpm);
                telemetryData['rpm_m_izq2'] = num(state.left_side.motors[1]?.rpm);
                telemetryData['rpm_m_izq3'] = num(state.left_side.motors[2]?.rpm);
            }
            if (state.right_side && Array.isArray(state.right_side.motors)) {
                telemetryData['rpm_m_der1'] = num(state.right_side.motors[0]?.rpm);
                telemetryData['rpm_m_der2'] = num(state.right_side.motors[1]?.rpm);
                telemetryData['rpm_m_der3'] = num(state.right_side.motors[2]?.rpm);
            }
        }

        if (Object.keys(telemetryData).length === 0) return;
        broadcast(telemetryData, timestamp);
    } catch (err) {
        console.error(`[HMI] Paquete malformado en ${topic}: ${err.message}`);
    }
});

// Botón Reset Pose: publica lo que jetson/server.py:on_message ya escucha.
app.post('/api/reset-pose', (req, res) => {
    const cmd = JSON.stringify({ action: 'reset_pose' });
    mqttClient.publish('rover/commands', cmd, { qos: 1 }, (err) => {
        if (err) {
            console.error(`[HMI] Error publicando reset_pose: ${err.message}`);
            return res.status(500).json({ status: 'error', error: err.message });
        }
        console.log('[HMI] reset_pose publicado a rover/commands');
        return res.json({ status: 'published', topic: 'rover/commands', action: 'reset_pose' });
    });
});

app.get('/api/status', (req, res) => {
    res.json({
        broker_url: BROKER_URL,
        mqtt_connected: mqttClient.connected,
        ws_clients: connectedClients.length,
        topics_seen: [...seenTopics]
    });
});

server.listen(PORT, '0.0.0.0', () => {
    console.log(`Open MCT Mission Control listening on port ${PORT}`);
});