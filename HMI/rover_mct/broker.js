const net = require('net');

const PORT = process.env.MQTT_PORT ? parseInt(process.env.MQTT_PORT, 10) : 1883;

async function startBroker() {
    // aedes@1.x es ESM puro: import dinámico desde CommonJS.
    // Expone clase Aedes con static async createBroker(opts).
    const { Aedes } = await import('aedes');
    const aedes = await Aedes.createBroker();

    // Bind the connection handler to the TCP server
    const server = net.createServer(aedes.handle);

    server.listen(PORT, '0.0.0.0', function () {
        console.log(`📡 Local MQTT Broker active on port ${PORT}`);
    });

    aedes.on('client', function (client) {
        console.log(`[MQTT] Client connected: ${client ? client.id : 'unknown'}`);
    });

    aedes.on('clientDisconnect', function (client) {
        console.log(`[MQTT] Client disconnected: ${client ? client.id : 'unknown'}`);
    });

    aedes.on('clientError', function (client, err) {
        console.log(`[MQTT] Client error: ${client ? client.id : 'unknown'} - ${err.message}`);
    });

    aedes.on('publish', function (packet, client) {
        if (packet && packet.topic && !packet.topic.startsWith('$SYS')) {
            console.log(`[MQTT] ${client ? client.id : 'broker'} -> ${packet.topic} (${packet.payload ? packet.payload.length : 0} bytes)`);
        }
    });
}

startBroker().catch(err => {
    console.error("Failed to start MQTT broker:", err);
    process.exit(1);
});