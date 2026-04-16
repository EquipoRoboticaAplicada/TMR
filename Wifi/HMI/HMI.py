import sys
import math
from PyQt5.QtWidgets import QApplication, QMainWindow, QTableWidgetItem, QOpenGLWidget
from PyQt5.QtGui import QPainter, QColor, QPen, QPolygonF, QLinearGradient
from PyQt5.QtCore import Qt, QTimer, QPointF

# Importamos diseño con pestañas
import qt_HMI_DISENO

# Importamos el Receiver 
from receiver import Receiver

# ===== FUNCIONES DE COLOR TOPOGRÁFICO =====
def limit_pitch(pitch):
    return max(-40, min(40, pitch))

def get_pitch_color(pitch):
    """Convierte el valor de pitch en un QColor para el heatmap"""
    pitch = limit_pitch(pitch)
    value = 50 + (pitch / 40) * 50
    value = max(0, min(100, int(value)))

    if value <= 50:
        ratio = value / 50
        r = 255
        g = int(255 * ratio)
    else:
        ratio = (value - 50) / 50
        r = int(255 * (1 - ratio))
        g = 255
    b = 0
    return QColor(r, g, b)


# ===== WIDGETS PERSONALIZADOS =====

class MapaNativoWidget(QOpenGLWidget):
    """Widget para dibujar el mapa de trayectoria (Pestaña 1)"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.escala = 50.0 # px por metro
        self.ruta = []
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_theta = 0.0

    def actualizar_pose(self, x, y, theta):
        self.rover_x = x
        self.rover_y = y
        self.rover_theta = theta
        # Solo agregamos a la ruta si nos movimos lo suficiente para no saturar memoria
        if not self.ruta or math.hypot(x - self.ruta[-1][0], y - self.ruta[-1][1]) > 0.05:
            self.ruta.append((x, y))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(30, 30, 30)) # Fondo oscuro
        
        origen_x = self.width() / 2
        origen_y = self.height() / 2

        # 1. Dibujar Cuadrícula
        pen_grid = QPen(QColor(60, 60, 60), 1)
        painter.setPen(pen_grid)
        paso = int(self.escala)
        for i in range(0, self.width(), paso):
            painter.drawLine(i, 0, i, self.height())
        for i in range(0, self.height(), paso):
            painter.drawLine(0, i, self.width(), i)

        # 2. Dibujar Trayectoria
        if len(self.ruta) > 1:
            pen_ruta = QPen(QColor(0, 255, 0), 2)
            painter.setPen(pen_ruta)
            for i in range(len(self.ruta) - 1):
                x1, y1 = self.ruta[i]
                x2, y2 = self.ruta[i+1]
                px1 = origen_x + (x1 * self.escala)
                py1 = origen_y - (y1 * self.escala)
                px2 = origen_x + (x2 * self.escala)
                py2 = origen_y - (y2 * self.escala)
                painter.drawLine(int(px1), int(py1), int(px2), int(py2))
 
        # 3. Dibujar Rover (Triángulo)
        rover_m = 0.3
        pts_locales = [(rover_m, 0.0), (-rover_m/2, rover_m/2), (-rover_m/2, -rover_m/2)]
        poligono = QPolygonF()
        cos_t = math.cos(self.rover_theta)
        sin_t = math.sin(self.rover_theta)
        
        for lx, ly in pts_locales:
            rx = lx * cos_t - ly * sin_t
            ry = lx * sin_t + ly * cos_t
            px = origen_x + ((self.rover_x + rx) * self.escala)
            py = origen_y - ((self.rover_y + ry) * self.escala)
            poligono.append(QPointF(px, py))
            
        painter.setBrush(QColor(255, 0, 0))
        painter.setPen(Qt.NoPen)
        painter.drawPolygon(poligono)

class MapaTopograficoWidget(QOpenGLWidget):
    """Widget para dibujar el heatmap topográfico y la barra de calor (Pestaña 2)"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.escala = 50.0 
        self.datos_heatmap = [] # Lista de (x, y, pitch)

    def actualizar_datos(self, x, y, pitch):
        if not self.datos_heatmap or math.hypot(x - self.datos_heatmap[-1][0], y - self.datos_heatmap[-1][1]) > 0.05:
            self.datos_heatmap.append((x, y, pitch))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(20, 20, 20)) # Fondo un poco más oscuro
        
        origen_x = self.width() / 2
        origen_y = self.height() / 2

        # 1. Dibujar segmentos de trayectoria con el color del Pitch
        if len(self.datos_heatmap) > 1:
            for i in range(len(self.datos_heatmap) - 1):
                x1, y1, p1 = self.datos_heatmap[i]
                x2, y2, p2 = self.datos_heatmap[i+1]
                
                px1 = origen_x + (x1 * self.escala)
                py1 = origen_y - (y1 * self.escala)
                px2 = origen_x + (x2 * self.escala)
                py2 = origen_y - (y2 * self.escala)
                
                color = get_pitch_color(p2)
                pen_heatmap = QPen(color, 8)
                pen_heatmap.setCapStyle(Qt.RoundCap)
                painter.setPen(pen_heatmap)
                painter.drawLine(int(px1), int(py1), int(px2), int(py2))

        # 2. Dibujar la Barra de Referencia de Calor (Lado Derecho)
        bar_width = 20
        bar_height = self.height() - 100
        bar_x = self.width() - 80
        bar_y = 50

        # Crear un gradiente de arriba (Verde = Pitch 40) a abajo (Rojo = Pitch -40)
        gradient = QLinearGradient(bar_x, bar_y, bar_x, bar_y + bar_height)
        gradient.setColorAt(0.0, QColor(0, 255, 0))    # Pitch alto (Verde)
        gradient.setColorAt(0.5, QColor(255, 255, 0))  # Flat (Amarillo)
        gradient.setColorAt(1.0, QColor(255, 0, 0))    # Pitch bajo (Rojo)

        painter.setBrush(gradient)
        painter.setPen(Qt.NoPen)
        painter.drawRect(bar_x, bar_y, bar_width, bar_height)

        # Dibujar Textos de la Escala
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(bar_x + 30, bar_y + 10, "+40°")
        painter.drawText(bar_x + 30, bar_y + (bar_height // 2) + 5, " 0°")
        painter.drawText(bar_x + 30, bar_y + bar_height, "-40°")


# ===== VENTANA PRINCIPAL =====

class MiRoverGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = qt_HMI_DISENO.Ui_MainWindow()
        self.ui.setupUi(self)

        # 1. Configurar Mapa de Trayectoria (Pestaña 1)
        self.mapa_trayectoria = MapaNativoWidget(self.ui.tab)
        self.mapa_trayectoria.setGeometry(self.ui.openGLWidget.geometry())
        self.ui.openGLWidget.deleteLater()
        self.ui.openGLWidget = self.mapa_trayectoria

        # 2. Configurar Mapa Topográfico (Pestaña 2)
        self.mapa_topografico = MapaTopograficoWidget(self.ui.tab_2)
        self.mapa_topografico.setGeometry(self.ui.openGLWidget_2.geometry())
        self.ui.openGLWidget_2.deleteLater()
        self.ui.openGLWidget_2 = self.mapa_topografico

        # 3. Configurar Conexión a la Jetson (Real Data)
        self.IP_JETSON = "172.32.237.112"  # <--- CAMBIA ESTO POR LA IP DE TU JETSON
        self.receiver = Receiver(PI_IP=self.IP_JETSON, poll_hz=10.0)
        self.receiver.start()
        print(f"📡 Intentando conectar con Jetson en http://{self.IP_JETSON}:5000 ...")

        # 4. Iniciar Timer de Actualización de HMI (10Hz / 100ms)
        self.timer = QTimer()
        self.timer.timeout.connect(self.loop_principal)
        self.timer.start(100)

    def actualizar_celda(self, tabla, fila, columna, texto):
        item = QTableWidgetItem(str(texto))
        item.setTextAlignment(Qt.AlignCenter)
        tabla.setItem(fila, columna, item)

    def loop_principal(self):
        """Lee datos del Receiver en background y actualiza el HMI en tiempo real"""
        
        # --- ODOMETRÍA ---
        x, y, theta = self.receiver.pose
        v_lineal, v_angular = self.receiver.velocity
        theta_grados = math.degrees(theta) % 360

        # --- SENSORES TOPOGRÁFICOS ---
        pitch = self.receiver.pitch
        heading = self.receiver.heading
        
        # Accedemos a variables extra usando el lock interno del receiver para seguridad
        with self.receiver._lock:
            terrain = getattr(self.receiver, '_terrain_text', "NONE")
            peso = getattr(self.receiver, '_peso', 0.0)

        # --- ACTUALIZAR MAPAS ---
        self.mapa_trayectoria.actualizar_pose(x, y, theta)
        self.mapa_topografico.actualizar_datos(x, y, pitch)

        # --- ACTUALIZAR TABLAS (PESTAÑA 3) ---
        estado_conexion = "⚠️ DESCONECTADO" if self.receiver.is_stale else "✅ CONECTADO"
        self.setWindowTitle(f"HMI Rover - {estado_conexion}")

        # TABLA 1: Odometría (Velocidad, RPM'S, Orientación)
        self.actualizar_celda(self.ui.tableWidget, 0, 0, f"{v_lineal:.2f} m/s")
        self.actualizar_celda(self.ui.tableWidget, 1, 0, f"w: {v_angular:.2f} rad/s") # Mostramos vel angular en lugar de RPMs sueltas
        self.actualizar_celda(self.ui.tableWidget, 2, 0, f"{theta_grados:.1f} °")

        # TABLA 2: Topografía (Orientación, Nivel de Inclinación, Valle/Surco)
        self.actualizar_celda(self.ui.tableWidget_2, 0, 0, f"{heading:.1f} °")
        self.actualizar_celda(self.ui.tableWidget_2, 1, 0, f"{pitch:.2f} °")
        self.actualizar_celda(self.ui.tableWidget_2, 2, 0, terrain)

        # TABLA 3: Piedras (Color, Peso, Tamaño)
        self.actualizar_celda(self.ui.tableWidget_3, 0, 0, "N/A") # Color de Visión pendiente
        self.actualizar_celda(self.ui.tableWidget_3, 1, 0, f"{peso:.1f} g")
        self.actualizar_celda(self.ui.tableWidget_3, 2, 0, "N/A") # Tamaño de Visión pendiente

    def closeEvent(self, event):
        """Garantiza que el hilo de conexión de red se cierre con la ventana"""
        print("🛑 Cerrando HMI y deteniendo receiver...")
        self.receiver.stop()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ventana = MiRoverGUI()
    ventana.show()
    sys.exit(app.exec_())
