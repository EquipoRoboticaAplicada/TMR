#!/bin/bash
# check_net.sh — verificación pre-vuelo HMI <-> Jetson (solo lectura, no modifica nada)
# Uso: ./check_net.sh <IP_BROKER> [PUERTO_MQTT=1883] [PUERTO_HMI=8080]
# Ej:  ./check_net.sh 172.32.149.2
set -u
BROKER_IP="${1:-}"
MQTT_PORT="${2:-1883}"
HMI_PORT="${3:-8080}"

if [ -z "$BROKER_IP" ]; then
  echo "Uso: $0 <IP_BROKER> [MQTT_PORT] [HMI_PORT]"
  exit 2
fi

echo "=== 1. ip a (tu IP local) ==="
ip -brief a || ip a | grep -E "inet |^[0-9]+:" | head -n 20

echo ""
echo "=== 2. ping a $BROKER_IP ==="
ping -c 3 -W 2 "$BROKER_IP" || echo "⚠️ ping falló (puede ser firewall, sigue con nc)."

echo ""
echo "=== 3. puerto MQTT $BROKER_IP:$MQTT_PORT ==="
if command -v nc >/dev/null 2>&1; then
  nc -zv -w 3 "$BROKER_IP" "$MQTT_PORT" || echo "⚠️ MQTT cerrado/inaccesible."
else
  timeout 3 bash -c "cat < /dev/null > /dev/tcp/$BROKER_IP/$MQTT_PORT" \
    && echo "MQTT abierto." || echo "⚠️ MQTT cerrado/inaccesible."
fi

echo ""
echo "=== 4. HMI local :$HMI_PORT ==="
curl -s -m 3 "http://localhost:$HMI_PORT/api/status" | head -c 500; echo ""
curl -s -m 3 "http://localhost:$HMI_PORT/dictionary.json" | head -c 200; echo ""

echo ""
echo "=== 5. escucha rover/# (5 mensajes) ==="
if command -v mosquitto_sub >/dev/null 2>&1; then
  timeout 10 mosquitto_sub -h "$BROKER_IP" -p "$MQTT_PORT" -t 'rover/#' -C 5 -v
elif node -e "require('mqtt')" 2>/dev/null; then
  node -e "
const mqtt=require('mqtt');
const c=mqtt.connect('mqtt://$BROKER_IP:$MQTT_PORT');
let n=0;
c.on('connect',()=>c.subscribe('rover/#'));
c.on('message',(t,m)=>{console.log('['+t+'] '+m.toString().slice(0,300)); if(++n>=5){c.end();process.exit(0);}});
setTimeout(()=>{console.log('timeout sin mensajes');process.exit(1);},10000);
"
else
  echo "(instala mosquitto-clients para esta prueba: sudo dnf install mosquitto)"
fi
