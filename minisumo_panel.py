"""
MINISUMO 2026 - Panel de calibracion y sensores por serial / Bluetooth
Autor: Alcides Ramos

Requiere: Python 3 + pyserial      (pip install pyserial)
Ejecutar: python minisumo_panel.py
          python minisumo_panel.py --demo     (robot simulado, sin hardware)

Que hace:
  - Cambia KGIRO, VGIRO y VMAX (tope de velocidad) en vivo, y los guarda o lee de la flash del robot.
  - Prueba si el robot va recto (TRECTO) o como gira (TGIRO), sin radio.
  - Muestra los 8 lidars, pitch, roll y yaw, y una grafica de la aceleracion.
  - Muestra la bateria y los 4 sensores de piso (led y valor analogo).
  - Pasa el robot a modo reposo o automatico.

Los datos que llegan del robot quedan en variables para usarlos despues:
  self.datos.lidar[0..7]     distancia de cada lidar en cm
  self.datos.lidar_ok[0..7]  True = sensor funcionando
  self.datos.pitch, .roll, .yaw      grados
  self.datos.ax, .ay, .az            m/s2
  self.datos.radio[0..3]     canales del radio en us (0 = sin senal)
  self.datos.piso[0..3]      codigo ADC de los sensores S1 a S4 (0 a 4095)
  self.datos.en_blanco(i)    True = blanco, False = negro, None = sin dato o sensor descartado
  self.datos.bateria_v       voltaje de la bateria en V
  self.datos.kgiro, .vgiro, .vmax, .estado, .guardado, .imu_ok
  self.datos.hist_t, .hist_ax, .hist_ay, .hist_az    historial (ultimos 20 s)
"""

import math
import queue
import re
import sys
import threading
import time
import tkinter as tk
from collections import deque
from dataclasses import dataclass, field
from tkinter import ttk

import serial
import serial.tools.list_ports

# ----------------------------------------------------------------- estilo
FONDO = "#b4b4b4"
PANEL = "#c4c4c4"
CAJA = "#d6d6d6"
ROSA = "#e000e0"
AZUL = "#0000c0"
ROJO = "#e00000"
VERDE = "#008a00"
NEGRO = "#000000"

F_TITULO = ("Segoe UI", 12, "bold")
F_ETIQ = ("Segoe UI", 9, "bold")
F_VALOR = ("Segoe UI", 12, "bold")
F_PEQ = ("Segoe UI", 8)

# ----------------------------------------------------------------- protocolo
BAUDIOS = 115200
TX_ESPACIO_S = 0.05     # tiempo entre comandos enviados (que no lleguen pegados en una trama)
NUM_LIDAR = 8
NUM_PISO = 4

# bateria: el robot manda el codigo del ADC (12 bits, referencia 3.3 V)
ADC_VREF, ADC_MAX = 3.3, 4095
BAT_R1, BAT_R2 = 10000.0, 3300.0     # divisor de la bateria (lee hasta 13.3 V)
BAT_DIVISOR = (BAT_R1 + BAT_R2) / BAT_R2
BAT_CELDAS = 3           # LiPo 3S: define cuando el gadget marca vacia y llena
BAT_V_VACIA = 3.3 * BAT_CELDAS
BAT_V_LLENA = 4.2 * BAT_CELDAS

# donde va cada sensor en el cuadro del panel: primera fila = frente del robot
PISO_CUADRO = ((1, 2),
               (3, 4))
# lecturas medidas en el robot (negro, blanco). El umbral es el punto medio de cada sensor.
# None = sensor descartado. S3 no lee: da 79 en negro y 53 en blanco
PISO_CALIBRACION = ((3669, 256), (3328, 273), None, (2035, 248))

# limites: LOS MISMOS de parametros.h y de main.c (TEST_...)
KGIRO_MIN, KGIRO_MAX = 0.5, 2.0
VGIRO_MIN, VGIRO_MAX = 0, 500
VMAX_MIN, VMAX_MAX = 0, 500
PUERTO_DEF = "COM53"     # puerto que aparece elegido al abrir el panel, si existe
TEST_VEL_MAX = 500
TEST_MS_MIN, TEST_MS_MAX = 100, 10000

# MISMO orden que enum estados de main.c (*EST n*)
ESTADOS = ("REPOSO", "CONTROL", "TEST", "AUTOMATICO")

LIDAR_MAX_CM = 100.0     # lidar_max_mm de lidarvl53.h: sin objetivo o mas lejos
HISTORIAL_S = 20.0
GRAF_MUESTRAS = 200      # muestras visibles en la grafica (10 s a 20 Hz)
SIN_DATOS_S = 1.0        # sin telemetria este tiempo: el panel avisa
# lineas del robot que avisan que sus valores cambiaron (PARAMETROS: = se reinicio)
RESPUESTAS_SINCRONIZAN = ("FLASH: parametros leidos", "DEFECTO:", "PARAMETROS:")

# dato suelto entre asteriscos:  *KGIRO0.950*  *EST1*
DATO_AST = re.compile(r"\*([A-Za-z]+)\s*=?\s*(-?\d+(?:\.\d+)?)\*")
# dato tipo Teleplot:  >AX:-1.25
DATO_TP = re.compile(r"^>([A-Za-z0-9]+):\s*(-?\d+(?:\.\d+)?)\s*$")


def separa(buf):
    """saca del texto recibido los datos y las lineas completas.
    devuelve (eventos, resto_sin_procesar). Cada evento es ("dato", nombre, valor)
    o ("linea", texto)"""
    eventos = []
    while True:
        m = DATO_AST.search(buf)
        if not m:
            break
        eventos.append(("dato", m.group(1), float(m.group(2))))
        buf = buf[:m.start()] + buf[m.end():]
    while "\n" in buf:
        linea, buf = buf.split("\n", 1)
        linea = linea.strip("\r").strip()
        if not linea:
            continue
        m = DATO_TP.match(linea)
        if m:
            eventos.append(("dato", m.group(1), float(m.group(2))))
        elif not linea.startswith(">"):      # un >... cortado por ruido no se muestra
            eventos.append(("linea", linea))
    if len(buf) > 2000:                      # basura sin fin de linea: no dejar crecer el buffer
        buf = buf[-200:]
    return eventos, buf


def factores(k):
    """cuanto se aplica a cada motor (izquierdo, derecho). IGUAL que Motores_mueve de main.c"""
    return (k, 1.0) if k < 1.0 else (1.0, 1.0 / k)


def voltaje_bateria(codigo):
    """pasa el codigo del ADC a voltios, aplicando el divisor de la bateria"""
    return codigo * ADC_VREF / ADC_MAX * BAT_DIVISOR


def piso_en_blanco(i, codigo):
    """True = blanco, False = negro, None = sin dato o sensor descartado"""
    calibracion = PISO_CALIBRACION[i]
    if calibracion is None or codigo is None:
        return None
    return codigo < sum(calibracion) / 2


def carga_bateria(voltios):
    """de 0.0 (vacia) a 1.0 (llena). Es lineal: una aproximacion, sobre todo con los motores andando"""
    return min(1.0, max(0.0, (voltios - BAT_V_VACIA) / (BAT_V_LLENA - BAT_V_VACIA)))


@dataclass
class Datos:
    """ultimos valores recibidos del robot: aqui se leen para usarlos despues"""
    kgiro: float = None
    vgiro: int = None
    vmax: int = None
    estado: int = None
    guardado: bool = False
    imu_ok: bool = False
    lidar: list = field(default_factory=lambda: [0.0] * NUM_LIDAR)
    lidar_ok: list = field(default_factory=lambda: [False] * NUM_LIDAR)
    radio: list = field(default_factory=lambda: [0.0] * 4)
    piso: list = field(default_factory=lambda: [None] * NUM_PISO)
    bateria_v: float = None
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    ultimo: float = 0.0       # time.monotonic() del ultimo dato
    hist_t: deque = field(default_factory=lambda: deque(maxlen=int(HISTORIAL_S * 20)))
    hist_ax: deque = field(default_factory=lambda: deque(maxlen=int(HISTORIAL_S * 20)))
    hist_ay: deque = field(default_factory=lambda: deque(maxlen=int(HISTORIAL_S * 20)))
    hist_az: deque = field(default_factory=lambda: deque(maxlen=int(HISTORIAL_S * 20)))

    def en_blanco(self, i):
        return piso_en_blanco(i, self.piso[i])

    def actualiza(self, nombre, valor):
        """guarda un dato recibido. Devuelve False si el nombre no se conoce"""
        self.ultimo = time.monotonic()
        m = re.fullmatch(r"L([1-8])", nombre)
        if m:
            self.lidar[int(m.group(1)) - 1] = valor
            return True
        m = re.fullmatch(r"C([1-4])", nombre)
        if m:
            self.radio[int(m.group(1)) - 1] = valor
            return True
        m = re.fullmatch(r"S([1-4])", nombre)
        if m:
            self.piso[int(m.group(1)) - 1] = int(valor)
            return True
        if nombre == "BAT":
            self.bateria_v = voltaje_bateria(valor)
        elif nombre == "KGIRO":
            self.kgiro = valor
        elif nombre == "VGIRO":
            self.vgiro = int(valor)
        elif nombre == "VMAX":
            self.vmax = int(valor)
        elif nombre == "EST":
            self.estado = int(valor)
        elif nombre == "GUARD":
            self.guardado = valor != 0
        elif nombre == "IMU":
            self.imu_ok = valor != 0
        elif nombre == "LIDOK":
            mascara = int(valor)
            self.lidar_ok = [bool(mascara & (1 << i)) for i in range(NUM_LIDAR)]
        elif nombre == "PITCH":
            self.pitch = valor
        elif nombre == "ROLL":
            self.roll = valor
        elif nombre == "YAW":
            self.yaw = valor
        elif nombre == "AX":
            self.ax = valor
        elif nombre == "AY":
            self.ay = valor
        elif nombre == "AZ":          # llega de ultimo: completa una muestra del historial
            self.az = valor
            self.hist_t.append(self.ultimo)
            self.hist_ax.append(self.ax)
            self.hist_ay.append(self.ay)
            self.hist_az.append(self.az)
        else:
            return False
        return True


# ======================================================================
#                            ROBOT SIMULADO (--demo)
# ======================================================================
class RobotSimulado:
    """se comporta como el firmware en el serial: misma telemetria y mismos comandos.
    Tiene la misma interfaz que serial.Serial que usa Enlace: read, write, close"""

    def __init__(self):
        self.kgiro = 1.0
        self.vgiro = 500
        self.vmax = 500
        self.flash = None           # (kgiro, vgiro, vmax) grabados
        self.estado = 0
        self.fin_test = 0.0
        self.salida = []
        self.t_rapido = 0.0
        self.t_lento = 0.0
        self.t0 = time.monotonic()
        self.lock = threading.Lock()

    def _manda(self, texto):
        self.salida.append(texto)

    def _procesa(self, trama):
        self._manda(trama + "\r\n")
        if "STOP" in trama:
            self.estado = 0
        elif m := re.search(r"KGIRO=(-?[\d.]+)", trama):
            v = float(m.group(1))
            if KGIRO_MIN <= v <= KGIRO_MAX:
                self.kgiro = v
            else:
                self._manda("KGIRO: fuera de rango\r\n")
        elif m := re.search(r"VGIRO=(-?[\d.]+)", trama):
            v = int(float(m.group(1)))
            if VGIRO_MIN <= v <= VGIRO_MAX:
                self.vgiro = v
            else:
                self._manda("VGIRO: fuera de rango\r\n")
        elif m := re.search(r"VMAX=(-?[\d.]+)", trama):
            v = int(float(m.group(1)))
            if VMAX_MIN <= v <= VMAX_MAX:
                self.vmax = v
            else:
                self._manda("VMAX: fuera de rango\r\n")
        elif m := re.search(r"T(RECTO|GIRO)=(-?\d+),(-?\d+)", trama):
            tipo, a, ms = m.group(1), int(m.group(2)), int(m.group(3))
            valido = (abs(a) <= TEST_VEL_MAX) if tipo == "RECTO" else (a in (1, -1))
            if self.estado == 1:
                self._manda("TEST: manda el radio, suba el canal 3\r\n")
            elif not valido or not TEST_MS_MIN <= ms <= TEST_MS_MAX:
                self._manda("TEST: valores invalidos\r\n")
            else:
                self.estado = 2
                self.fin_test = time.monotonic() + ms / 1000.0
                self._manda("TEST: recto\r\n" if tipo == "RECTO" else "TEST: giro\r\n")
        elif "AUTO" in trama:
            if self.estado == 1:
                self._manda("AUTO: manda el radio, suba el canal 3\r\n")
            else:
                self.estado = 3
                self._manda("AUTO: activado\r\n")
        elif "GUARDAFLASH" in trama:
            if self.estado != 0:
                self._manda("FLASH: detener el robot\r\n")
            else:
                self.flash = (self.kgiro, self.vgiro, self.vmax)
                self._manda("FLASH: guardado\r\n")
        elif "LEEFLASH" in trama:
            if self.flash:
                self.kgiro, self.vgiro, self.vmax = self.flash
                self._manda("FLASH: parametros leidos\r\n")
            else:
                self._manda("FLASH: no hay parametros guardados\r\n")
        elif "DEFECTO" in trama:
            self.kgiro, self.vgiro, self.vmax = 1.0, 500, 500
            self._manda("DEFECTO: aplicado, sin guardar\r\n")

    def _telemetria(self):
        ahora = time.monotonic()
        t = ahora - self.t0
        if self.estado == 2 and ahora >= self.fin_test:
            self.estado = 0
            self._manda("TEST: fin\r\n")
        if ahora - self.t_rapido >= 0.05:
            self.t_rapido = ahora
            agita = 1.5 if self.estado == 2 else 0.1
            self._manda(f">PITCH:{3 * math.sin(t / 2):.1f}\n>ROLL:{2 * math.cos(t / 3):.1f}\n"
                        f">YAW:{(t * 10) % 360:.1f}\n"
                        f">AX:{agita * math.sin(t * 7):.2f}\n>AY:{agita * math.cos(t * 5):.2f}\n"
                        f">AZ:{9.8 + agita * math.sin(t * 3) / 2:.2f}\n")
        if ahora - self.t_lento >= 0.2:
            self.t_lento = ahora
            lineas = "".join(f">L{c + 1}:{50 + 45 * math.sin(t + c):.1f}\n" for c in range(7))
            guardado = self.flash == (self.kgiro, self.vgiro, self.vmax)
            lineas += "".join(f">S{c + 1}:{2000 + 1500 * math.sin(t + c):.0f}\n" for c in range(NUM_PISO))
            lineas += f">BAT:{3450 + 450 * math.sin(t / 5):.0f}\n"
            self._manda(lineas + ">L8:100.0\n>C1:0\n>C2:0\n>C3:0\n>C4:0\n"
                        f"*KGIRO{self.kgiro:.3f}**VMAX{self.vmax}**VGIRO{self.vgiro}**EST{self.estado}*"
                        f"*GUARD{int(guardado)}**IMU1**LIDOK127*\n")

    def write(self, datos):
        with self.lock:
            self._procesa(datos.decode())

    def read(self, _n):
        time.sleep(0.02)
        with self.lock:
            self._telemetria()
            texto = "".join(self.salida)
            self.salida.clear()
        return texto.encode()

    def close(self):
        pass


# ======================================================================
#                            ENLACE SERIAL
# ======================================================================
class Enlace(threading.Thread):
    """hilo que lee y escribe el puerto. Lo recibido llega por la cola 'entrada'.
    Los comandos salen separados TX_ESPACIO_S para que no lleguen pegados"""

    def __init__(self, puerto):
        super().__init__(daemon=True)
        self.puerto = puerto
        self.entrada = queue.Queue()
        self.salida = deque()
        self.lock = threading.Lock()
        self.activo = True
        self.t_tx = 0.0
        self.start()

    def enviar(self, texto):
        with self.lock:
            self.salida.append(texto)

    def enviar_urgente(self, texto, veces=3):
        """borra lo pendiente y lo manda de primero. Varias veces: un comando que llega
        justo mientras el robot rearma la recepcion se pierde"""
        with self.lock:
            self.salida.clear()
            self.salida.extend([texto] * veces)
            self.t_tx = 0.0

    def run(self):
        while self.activo:
            try:
                datos = self.puerto.read(512)
                if datos:
                    self.entrada.put(("rx", datos.decode("latin-1")))
                self._saca_comando()
            except (serial.SerialException, OSError) as e:
                self.entrada.put(("error", str(e)))
                self.activo = False

    def _saca_comando(self):
        with self.lock:
            if not self.salida or (time.monotonic() - self.t_tx) < TX_ESPACIO_S:
                return
            texto = self.salida.popleft()
            self.t_tx = time.monotonic()
        self.puerto.write(texto.encode())

    def cerrar(self):
        """manda lo que falte (el STOP al desconectar) y cierra el puerto"""
        limite = time.monotonic() + 0.5
        while self.salida and self.activo and time.monotonic() < limite:
            time.sleep(0.01)
        self.activo = False
        self.join(timeout=0.5)
        try:
            self.puerto.close()
        except (serial.SerialException, OSError):
            pass


# ======================================================================
#                                 PANEL
# ======================================================================
class GraficaAceleracion(tk.Canvas):
    """grafica pequena de ax, ay, az. Pinta las ultimas GRAF_MUESTRAS muestras"""
    SERIES = (("X", "#ff5050"), ("Y", "#40e040"), ("Z", "#50a0ff"))

    def __init__(self, master, ancho=410, alto=135):
        super().__init__(master, width=ancho, height=alto, bg="#101820", highlightthickness=0)
        self.ancho, self.alto = ancho, alto
        self.lineas = [self.create_line(0, 0, 0, 0, fill=c, width=1) for _, c in self.SERIES]
        self.cero = self.create_line(0, 0, ancho, 0, fill="#506070", dash=(2, 3))
        self.txt_max = self.create_text(4, 3, anchor="nw", fill="#a0b0c0", font=F_PEQ)
        self.txt_min = self.create_text(4, alto - 3, anchor="sw", fill="#a0b0c0", font=F_PEQ)
        x = ancho - 70
        for nombre, c in self.SERIES:
            self.create_text(x, 3, anchor="nw", fill=c, font=F_ETIQ, text=nombre)
            x += 22

    def redibuja(self, series):
        """series = (ax, ay, az), listas del mismo largo"""
        n = len(series[0])
        if n < 2:
            return
        maximo = max(12.0, max(abs(v) for s in series for v in s))   # escala simetrica en m/s2
        centro = self.alto / 2.0
        escala = (self.alto / 2.0 - 4) / maximo
        paso = self.ancho / float(GRAF_MUESTRAS - 1)
        x0 = self.ancho - (n - 1) * paso
        for linea, s in zip(self.lineas, series):
            puntos = []
            for i, v in enumerate(s):
                puntos += [x0 + i * paso, centro - v * escala]
            self.coords(linea, *puntos)
        self.coords(self.cero, 0, centro, self.ancho, centro)
        self.itemconfigure(self.txt_max, text=f"+{maximo:.0f} m/s2")
        self.itemconfigure(self.txt_min, text=f"-{maximo:.0f}")


class BarraLidar(tk.Canvas):
    """una fila por lidar: nombre, barra de 0 a LIDAR_MAX_CM y valor"""

    def __init__(self, master, ancho=410, alto_fila=21):
        super().__init__(master, width=ancho, height=alto_fila * NUM_LIDAR, bg=CAJA,
                         highlightthickness=0)
        self.x_barra, self.x_valor = 28, ancho - 62
        self.barras = []
        self.textos = []
        for i in range(NUM_LIDAR):
            y = i * alto_fila
            self.create_text(4, y + alto_fila / 2, anchor="w", text=f"L{i + 1}", font=F_ETIQ)
            self.create_rectangle(self.x_barra, y + 4, self.x_valor, y + alto_fila - 4,
                                  outline="#808080", fill="#eeeeee")
            self.barras.append(self.create_rectangle(self.x_barra, y + 4, self.x_barra,
                                                     y + alto_fila - 4, outline="", fill=VERDE))
            self.textos.append(self.create_text(ancho - 4, y + alto_fila / 2, anchor="e",
                                                text="--", font=F_ETIQ))
        self.alto_fila = alto_fila

    def set(self, i, cm, ok):
        y = i * self.alto_fila
        if not ok:
            self.coords(self.barras[i], self.x_barra, y + 4, self.x_barra, y + self.alto_fila - 4)
            self.itemconfigure(self.textos[i], text="sin sensor", fill="#707070")
            return
        fraccion = min(max(cm, 0.0), LIDAR_MAX_CM) / LIDAR_MAX_CM
        x = self.x_barra + fraccion * (self.x_valor - self.x_barra)
        self.coords(self.barras[i], self.x_barra, y + 4, x, y + self.alto_fila - 4)
        # cerca = rojo (hay algo enfrente), lejos = verde
        color = ROJO if cm < 20 else ("#d09000" if cm < 50 else VERDE)
        self.itemconfigure(self.barras[i], fill=color)
        texto = ">100 cm" if cm >= LIDAR_MAX_CM else f"{cm:.1f} cm"
        self.itemconfigure(self.textos[i], text=texto, fill=NEGRO)


class GadgetBateria(tk.Canvas):
    """bateria dibujada: se llena segun el voltaje, cambia de color y parpadea si esta baja.
    El valor recibido se suaviza para que el nivel se mueva sin saltos"""
    #        desde  color    brillo
    NIVELES = ((0.5, VERDE, "#30b030"), (0.2, "#d09000", "#e8b030"), (0.0, ROJO, "#ff5050"))
    VACIO = "#eeeeee"
    SUAVIZADO = 0.12         # fraccion de la diferencia que recorre en cada cuadro
    ANCHO_BRILLO = 14
    CUADROS_BARRIDO = 50     # el brillo cruza la bateria en 2 s
    CUADROS_PARPADEO = 10    # con 40 ms por cuadro: cambia cada 400 ms

    def __init__(self, master, ancho=230, alto=96):
        super().__init__(master, width=ancho, height=alto, bg=CAJA, highlightthickness=0)
        self.x0, self.x1, self.y0, self.y1 = 7, ancho - 16, 44, alto - 7
        medio = (self.y0 + self.y1) / 2
        self.txt_v = self.create_text(6, 22, anchor="w", text="--", font=("Segoe UI", 20, "bold"))
        self.txt_carga = self.create_text(ancho - 6, 24, anchor="e", text="", font=F_TITULO)
        self.create_rectangle(self.x0 - 3, self.y0 - 3, self.x1 + 3, self.y1 + 3, outline=NEGRO,
                              width=2, fill=self.VACIO)
        self.create_rectangle(self.x1 + 4, medio - 9, self.x1 + 9, medio + 9, outline=NEGRO, fill=NEGRO)
        self.relleno = self.create_rectangle(self.x0, self.y0, self.x0, self.y1, outline="")
        self.brillo = self.create_rectangle(self.x0, self.y0, self.x0, self.y1, outline="")
        for n in (1, 2, 3):      # divisiones cada 25 %
            x = self.x0 + (self.x1 - self.x0) * n / 4
            self.create_line(x, self.y0 - 2, x, self.y1 + 2, fill=CAJA, width=2)
        self.objetivo = None     # voltaje recibido; None = sin datos
        self.v = None            # voltaje que se esta dibujando
        self.cuadro = 0

    def anima(self):
        """dibuja el siguiente cuadro"""
        self.cuadro += 1
        if self.objetivo is None:
            self.v = None
            self.itemconfigure(self.txt_v, text="--", fill="#707070")
            self.itemconfigure(self.txt_carga, text="")
            self.coords(self.relleno, self.x0, self.y0, self.x0, self.y1)
            self.coords(self.brillo, self.x0, self.y0, self.x0, self.y1)
            return
        self.v = self.objetivo if self.v is None else self.v + (self.objetivo - self.v) * self.SUAVIZADO

        carga = carga_bateria(self.v)
        _, color, claro = next(n for n in self.NIVELES if carga >= n[0])
        baja = carga < self.NIVELES[1][0]
        apagado = baja and (self.cuadro // self.CUADROS_PARPADEO) % 2 == 1
        x = self.x0 + carga * (self.x1 - self.x0)
        self.coords(self.relleno, self.x0, self.y0, x, self.y1)
        self.itemconfigure(self.relleno, fill=self.VACIO if apagado else color)

        # brillo que recorre la parte llena; recortado para no salirse de ella
        avance = (self.cuadro % self.CUADROS_BARRIDO) / self.CUADROS_BARRIDO
        izq = self.x0 - self.ANCHO_BRILLO + avance * (x - self.x0 + self.ANCHO_BRILLO)
        self.coords(self.brillo, max(self.x0, izq), self.y0, min(x, izq + self.ANCHO_BRILLO), self.y1)
        self.itemconfigure(self.brillo, fill=self.VACIO if apagado else claro)

        self.itemconfigure(self.txt_v, text=f"{self.v:.2f} V", fill=color if baja else NEGRO)
        self.itemconfigure(self.txt_carga, text=f"{carga * 100:.0f} %", fill=color if baja else NEGRO)


class CuadroPiso(tk.Canvas):
    """los 4 sensores de piso en un cuadro: un led por sensor y debajo su valor analogo.
    Led encendido = blanco, apagado = negro. Un sensor descartado se dibuja hueco y en gris,
    y sigue mostrando su valor para poder revisarlo"""
    LED_APAGADO = "#505050"
    LED_ENCENDIDO = "#ffd000"
    GRIS = "#808080"
    RADIO_LED = 14

    def __init__(self, master, ancho=230, alto=230):
        super().__init__(master, width=ancho, height=alto, bg=CAJA, highlightthickness=0)
        self.create_text(ancho / 2, 2, anchor="n", text="FRENTE", font=F_PEQ)
        self.create_rectangle(4, 18, ancho - 4, alto - 4, outline=self.GRIS)
        self.create_text(ancho / 2, alto - 8, anchor="s", text="encendido = blanco", font=F_PEQ)
        self.leds = [None] * NUM_PISO
        self.etiquetas = [None] * NUM_PISO
        self.textos = [None] * NUM_PISO
        filas = len(PISO_CUADRO)
        alto_fila = (alto - 22) / filas
        for f, fila in enumerate(PISO_CUADRO):
            for c, sensor in enumerate(fila):
                x = ancho * (2 * c + 1) / (2 * len(fila))
                r = self.RADIO_LED
                y = 20 + alto_fila * f + (alto_fila - 22) / 2    # led y valor centrados en su celda
                i = sensor - 1
                descartado = PISO_CALIBRACION[i] is None
                self.leds[i] = self.create_oval(x - r, y - r, x + r, y + r,
                                                fill=CAJA if descartado else self.LED_APAGADO,
                                                outline=self.GRIS if descartado else NEGRO)
                self.etiquetas[i] = self.create_text(x, y, text=f"S{sensor}", font=F_PEQ,
                                                     fill=self.GRIS if descartado else "white")
                self.textos[i] = self.create_text(x, y + r + 11, text="--", font=F_ETIQ,
                                                  fill=self.GRIS if descartado else ROSA)

    def set(self, i, codigo):
        self.itemconfigure(self.textos[i], text="--" if codigo is None else str(codigo))
        if PISO_CALIBRACION[i] is None:
            return
        blanco = piso_en_blanco(i, codigo)
        self.itemconfigure(self.leds[i], fill=self.LED_ENCENDIDO if blanco else self.LED_APAGADO)
        self.itemconfigure(self.etiquetas[i], fill=NEGRO if blanco else "white")


class Panel(tk.Tk):
    def __init__(self, demo=False):
        super().__init__()
        self.demo = demo
        self.title("MINISUMO 2026 - Calibracion y sensores" + ("  (DEMO: robot simulado)" if demo else ""))
        self.configure(bg=FONDO)
        self.resizable(False, False)

        self.datos = Datos()          # <- aqui quedan todos los datos del robot
        self.enlace = None
        self.buf = ""
        self.sincronizar = True       # el proximo dato del robot llena las casillas de KGIRO, VGIRO y VMAX
        self.k_enviado = None
        self.v_enviado = None
        self.vmax_enviado = None

        self._crea_vars()
        self._crea_controles()
        self.bind("<Escape>", lambda e: self.stop())
        self.bind("<space>", lambda e: self.stop())
        self.protocol("WM_DELETE_WINDOW", self.salir)
        self.refresca_puertos()
        if demo:
            self.conectar()
        self.id_datos = self.after(40, self._tick_datos)
        self.id_pantalla = self.after(100, self._tick_pantalla)
        self.id_anima = self.after(40, self._tick_anima)

    # ------------------------------------------------------------ construccion
    def _crea_vars(self):
        self.v_puerto = tk.StringVar()
        self.v_kgiro = tk.StringVar(value="1.000")
        self.v_vgiro = tk.StringVar(value="500")
        self.v_vmax = tk.StringVar(value=str(VMAX_MAX))
        self.v_paso = tk.StringVar(value="0.01")
        self.v_test_vel = tk.StringVar(value="150")
        self.v_test_ms = tk.StringVar(value="2000")
        self.v_giro_ms = tk.StringVar(value="500")
        self.i_conexion = tk.StringVar(value="Desconectado")
        self.i_estado = tk.StringVar(value="--")
        self.i_kgiro = tk.StringVar(value="--")
        self.i_vgiro = tk.StringVar(value="--")
        self.i_vmax = tk.StringVar(value="--")
        self.i_flash = tk.StringVar(value="--")
        self.i_factores = tk.StringVar(value="--")
        self.i_radio = tk.StringVar(value="--")
        self.i_pitch = tk.StringVar(value="--")
        self.i_roll = tk.StringVar(value="--")
        self.i_yaw = tk.StringVar(value="--")
        self.i_acel = tk.StringVar(value="--")
        self.i_imu = tk.StringVar(value="")

    def caja(self, padre, titulo, fila, columna=0, **grid):
        marco = tk.LabelFrame(padre, text=titulo, font=F_TITULO, fg=AZUL, bg=PANEL, padx=8, pady=6)
        marco.grid(row=fila, column=columna, sticky="nsew", padx=5, pady=4, **grid)
        return marco

    def etiqueta(self, padre, texto, fila, col, **kw):
        kw.setdefault("font", F_ETIQ)
        kw.setdefault("bg", PANEL)
        w = tk.Label(padre, text=texto, **kw)
        w.grid(row=fila, column=col, sticky="w", padx=2, pady=1)
        return w

    def valor(self, padre, var, fila, col, ancho=9, **kw):
        kw.setdefault("fg", ROSA)
        w = tk.Label(padre, textvariable=var, font=F_VALOR, bg=CAJA, width=ancho, relief="sunken", **kw)
        w.grid(row=fila, column=col, sticky="w", padx=2, pady=1)
        return w

    def boton(self, padre, texto, accion, fila, col, ancho=14):
        # takefocus=0: la barra espaciadora es STOP, no debe activar el boton con foco
        w = tk.Button(padre, text=texto, command=accion, width=ancho, font=F_ETIQ, bg="#d0d0d0", takefocus=0)
        w.grid(row=fila, column=col, padx=2, pady=2, sticky="w")
        return w

    def _crea_controles(self):
        raiz = tk.Frame(self, bg=FONDO)
        raiz.pack(padx=6, pady=6)
        izq = tk.Frame(raiz, bg=FONDO)
        izq.grid(row=0, column=0, sticky="n")
        der = tk.Frame(raiz, bg=FONDO)
        der.grid(row=0, column=1, sticky="n")
        ter = tk.Frame(raiz, bg=FONDO)
        ter.grid(row=0, column=2, sticky="n")

        self._crea_conexion(izq, 0)
        self._crea_stop(izq, 1)
        self._crea_ajuste(izq, 2)
        self._crea_pruebas(izq, 3)
        self._crea_sensores(der, 0)
        self._crea_bateria(ter, 0)
        self._crea_piso(ter, 1)
        self._crea_modo(ter, 2)
        self._crea_flash(ter, 3)      # aqui y no a la izquierda: en una pantalla de 1080p quedaba bajo la barra de tareas

        marco = self.caja(raiz, "TRAMA", 1, 0, columnspan=3)
        self.log = tk.Text(marco, height=5, width=96, font=("Consolas", 9), bg=CAJA, state="disabled")
        self.log.pack(fill="both")

    def _crea_conexion(self, padre, fila):
        m = self.caja(padre, "CONEXION", fila)
        self.combo = ttk.Combobox(m, textvariable=self.v_puerto, width=9, state="readonly")
        self.combo.grid(row=0, column=0, padx=2)
        self.boton(m, "Puertos", self.refresca_puertos, 0, 1, ancho=8)
        self.btn_conectar = self.boton(m, "Conectar", self.conectar, 0, 2, ancho=11)
        self.lbl_conexion = tk.Label(m, textvariable=self.i_conexion, font=F_ETIQ, bg=PANEL, fg=ROJO)
        self.lbl_conexion.grid(row=1, column=0, columnspan=3, sticky="w", padx=2)

    def _crea_stop(self, padre, fila):
        tk.Button(padre, text="STOP  (Esc o espacio)", command=self.stop, font=("Segoe UI", 14, "bold"),
                  bg=ROJO, fg="white", activebackground="#ff4040", takefocus=0,
                  height=1).grid(row=fila, column=0, sticky="ew", padx=5, pady=4)

    def _crea_ajuste(self, padre, fila):
        m = self.caja(padre, "AJUSTE EN VIVO", fila)

        self.etiqueta(m, "KGIRO", 0, 0)
        self.spin_k = tk.Spinbox(m, textvariable=self.v_kgiro, from_=KGIRO_MIN, to=KGIRO_MAX, increment=0.01,
                                 format="%.3f", width=8, font=F_VALOR, command=self.aplica_k)
        self.spin_k.grid(row=0, column=1, sticky="w")
        self.valor(m, self.i_kgiro, 0, 2, ancho=7)

        self.etiqueta(m, "Paso", 1, 0)
        ttk.Combobox(m, textvariable=self.v_paso, width=6, state="readonly",
                     values=("0.005", "0.01", "0.02", "0.05")).grid(row=1, column=1, sticky="w")
        tk.Label(m, textvariable=self.i_factores, font=F_PEQ, bg=PANEL, fg=AZUL).grid(
            row=2, column=0, columnspan=3, sticky="w", padx=2)

        tk.Label(m, text="Avanzando, el robot se va hacia:", font=F_ETIQ, bg=PANEL).grid(
            row=3, column=0, columnspan=3, sticky="w", pady=(4, 0))
        fila_btn = tk.Frame(m, bg=PANEL)
        fila_btn.grid(row=4, column=0, columnspan=3, sticky="ew")
        tk.Button(fila_btn, text="<< IZQUIERDA", font=F_ETIQ, width=15, bg="#d0d0d0", takefocus=0,
                  command=lambda: self.corrige(+1)).pack(side="left", padx=2, pady=2)
        tk.Button(fila_btn, text="DERECHA >>", font=F_ETIQ, width=15, bg="#d0d0d0", takefocus=0,
                  command=lambda: self.corrige(-1)).pack(side="left", padx=2, pady=2)

        self.etiqueta(m, "VGIRO", 5, 0)
        self.spin_v = tk.Spinbox(m, textvariable=self.v_vgiro, from_=VGIRO_MIN, to=VGIRO_MAX, increment=10,
                                 width=8, font=F_VALOR, command=self.aplica_v)
        self.spin_v.grid(row=5, column=1, sticky="w")
        self.valor(m, self.i_vgiro, 5, 2, ancho=7)

        self.etiqueta(m, "VMAX (tope)", 6, 0)
        self.spin_vmax = tk.Spinbox(m, textvariable=self.v_vmax, from_=VMAX_MIN, to=VMAX_MAX, increment=10,
                                    width=8, font=F_VALOR, command=self.aplica_vmax)
        self.spin_vmax.grid(row=6, column=1, sticky="w")
        self.valor(m, self.i_vmax, 6, 2, ancho=7)

        # el slider no comparte variable con la casilla: un slider corrige lo que se esta escribiendo.
        # takefocus=0: la barra espaciadora es STOP. Se manda al soltar, no en cada paso del arrastre
        self.escala_vmax = tk.Scale(m, from_=VMAX_MIN, to=VMAX_MAX, resolution=1, orient="horizontal", showvalue=False,
                                    bg=PANEL, highlightthickness=0, takefocus=0,
                                    command=lambda v: self.v_vmax.set(str(int(float(v)))))
        self.escala_vmax.set(VMAX_MAX)
        self.escala_vmax.grid(row=7, column=0, columnspan=3, sticky="ew", padx=2)
        self.escala_vmax.bind("<ButtonRelease-1>", self._suelta_vmax)

        for spin, accion in ((self.spin_k, self.aplica_k), (self.spin_v, self.aplica_v), (self.spin_vmax, self.aplica_vmax)):
            spin.bind("<Return>", lambda e, a=accion: a())
            spin.bind("<FocusOut>", lambda e, a=accion: a())

    def _crea_pruebas(self, padre, fila):
        m = self.caja(padre, "PRUEBAS (canal 3 arriba)", fila)

        self.etiqueta(m, "Velocidad (+-500)", 0, 0)
        tk.Spinbox(m, textvariable=self.v_test_vel, from_=-TEST_VEL_MAX, to=TEST_VEL_MAX, increment=10,
                   width=7, font=F_VALOR).grid(row=0, column=1, sticky="w")
        self.etiqueta(m, "Tiempo (ms)", 1, 0)
        tk.Spinbox(m, textvariable=self.v_test_ms, from_=TEST_MS_MIN, to=TEST_MS_MAX, increment=100,
                   width=7, font=F_VALOR).grid(row=1, column=1, sticky="w")
        tk.Button(m, text="PROBAR RECTO", font=F_ETIQ, bg="#60d060", takefocus=0,
                  command=self.prueba_recto).grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=3)

        self.etiqueta(m, "Giro (ms, usa VGIRO)", 3, 0)
        tk.Spinbox(m, textvariable=self.v_giro_ms, from_=TEST_MS_MIN, to=TEST_MS_MAX, increment=100,
                   width=7, font=F_VALOR).grid(row=3, column=1, sticky="w")
        fila_btn = tk.Frame(m, bg=PANEL)
        fila_btn.grid(row=4, column=0, columnspan=2, sticky="ew")
        tk.Button(fila_btn, text="<< GIRAR IZQ", font=F_ETIQ, width=15, bg="#60d060", takefocus=0,
                  command=lambda: self.prueba_giro(-1)).pack(side="left", padx=2, pady=2)
        tk.Button(fila_btn, text="GIRAR DER >>", font=F_ETIQ, width=15, bg="#60d060", takefocus=0,
                  command=lambda: self.prueba_giro(+1)).pack(side="left", padx=2, pady=2)

    def _crea_flash(self, padre, fila):
        m = self.caja(padre, "FLASH DEL ROBOT", fila)
        self.boton(m, "Guardar", lambda: self.enviar("GUARDAFLASH/"), 0, 0, ancho=9)
        self.boton(m, "Leer", lambda: self.enviar("LEEFLASH/"), 0, 1, ancho=9)
        self.boton(m, "Por defecto", lambda: self.enviar("DEFECTO/"), 0, 2, ancho=11)
        self.lbl_flash = tk.Label(m, textvariable=self.i_flash, font=F_ETIQ, bg=PANEL)
        self.lbl_flash.grid(row=1, column=0, columnspan=3, sticky="w", padx=2)

    def _crea_piso(self, padre, fila):
        m = self.caja(padre, "SENSORES DE PISO", fila)
        self.piso = CuadroPiso(m)
        self.piso.pack()

    def _crea_modo(self, padre, fila):
        m = self.caja(padre, "MODO", fila)
        self.boton(m, "REPOSO", self.stop, 0, 0, ancho=11)
        self.boton(m, "AUTOMATICO", self.modo_automatico, 0, 1, ancho=12)

    def _crea_bateria(self, padre, fila):
        m = self.caja(padre, "BATERIA", fila)
        self.gadget_bat = GadgetBateria(m)
        self.gadget_bat.pack()

    def _crea_sensores(self, padre, fila):
        m = self.caja(padre, "ROBOT", fila)
        self.etiqueta(m, "Estado", 0, 0)
        self.valor(m, self.i_estado, 0, 1, ancho=11)
        self.etiqueta(m, "Radio (C1/C2/C3)", 0, 2)
        self.valor(m, self.i_radio, 0, 3, ancho=16, fg=NEGRO)

        m = self.caja(padre, "LIDARS (cm)", fila + 1)
        self.barras = BarraLidar(m)
        self.barras.pack()

        m = self.caja(padre, "IMU BNO055", fila + 2)
        for col, (nombre, var) in enumerate((("PITCH", self.i_pitch), ("ROLL", self.i_roll), ("YAW", self.i_yaw))):
            self.etiqueta(m, nombre, 0, col)
            self.valor(m, var, 1, col, ancho=8)
        tk.Label(m, textvariable=self.i_imu, font=F_PEQ, bg=PANEL, fg=ROJO, wraplength=130).grid(
            row=0, column=3, rowspan=2, sticky="w", padx=4)

        m = self.caja(padre, "ACELERACION", fila + 3)
        self.grafica = GraficaAceleracion(m)
        self.grafica.pack()
        tk.Label(m, textvariable=self.i_acel, font=F_ETIQ, bg=PANEL).pack(anchor="w")

    # ------------------------------------------------------------ conexion
    def refresca_puertos(self):
        puertos = [p.device for p in serial.tools.list_ports.comports()]
        self.combo["values"] = puertos
        if puertos and self.v_puerto.get() not in puertos:
            self.v_puerto.set(PUERTO_DEF if PUERTO_DEF in puertos else puertos[0])

    def conectar(self):
        if self.enlace:
            self.desconectar()
            return
        try:
            if self.demo:
                puerto = RobotSimulado()
                nombre = "robot simulado"
            else:
                nombre = self.v_puerto.get()
                if not nombre:
                    self.escribe_log("Elige un puerto")
                    return
                puerto = serial.Serial(nombre, BAUDIOS, timeout=0.02, write_timeout=1)
        except (serial.SerialException, OSError) as e:
            self.escribe_log(f"No se pudo abrir el puerto: {e}")
            return
        self.enlace = Enlace(puerto)
        self.datos.ultimo = time.monotonic()
        self.sincronizar = True
        self.buf = ""
        self.i_conexion.set(f"Conectado a {nombre}")
        self.lbl_conexion.configure(fg=VERDE)
        self.btn_conectar.configure(text="Desconectar")

    def desconectar(self):
        if not self.enlace:
            return
        self.enlace.enviar_urgente("STOP/")   # sin panel nadie puede detener el test
        self.enlace.cerrar()
        self.enlace = None
        self.i_conexion.set("Desconectado")
        self.lbl_conexion.configure(fg=ROJO)
        self.btn_conectar.configure(text="Conectar")

    def salir(self):
        self.desconectar()
        self.after_cancel(self.id_datos)
        self.after_cancel(self.id_pantalla)
        self.after_cancel(self.id_anima)
        self.destroy()

    def enviar(self, texto):
        if not self.enlace:
            self.escribe_log("Sin conexion")
            return False
        self.enlace.enviar(texto)
        return True

    # ------------------------------------------------------------ acciones
    def stop(self):
        if self.enlace:
            self.enlace.enviar_urgente("STOP/")
        else:
            self.escribe_log("Sin conexion: no se pudo mandar STOP")

    def modo_automatico(self):
        self.enviar("AUTO/")

    def _lee_k(self):
        try:
            k = float(self.v_kgiro.get().replace(",", "."))
        except ValueError:
            return None
        return k if KGIRO_MIN <= k <= KGIRO_MAX else None

    def _lee_v(self):
        try:
            v = int(float(self.v_vgiro.get()))
        except ValueError:
            return None
        return v if VGIRO_MIN <= v <= VGIRO_MAX else None

    def aplica_k(self):
        """manda KGIRO si es distinto de lo ultimo enviado o de lo que reporta el robot"""
        k = self._lee_k()
        if k is None:
            self.escribe_log(f"KGIRO debe estar entre {KGIRO_MIN} y {KGIRO_MAX}")
            return False
        robot = self.datos.kgiro
        if (k != self.k_enviado or robot is None or abs(k - robot) >= 0.0005) and self.enviar(f"KGIRO={k:.3f}/"):
            self.k_enviado = k
        return True

    def aplica_v(self):
        """manda VGIRO si es distinto de lo ultimo enviado o de lo que reporta el robot"""
        v = self._lee_v()
        if v is None:
            self.escribe_log(f"VGIRO debe estar entre {VGIRO_MIN} y {VGIRO_MAX}")
            return False
        if (v != self.v_enviado or v != self.datos.vgiro) and self.enviar(f"VGIRO={v}/"):
            self.v_enviado = v
        return True

    def aplica_vmax(self):
        """manda VMAX si es distinto de lo ultimo enviado o de lo que reporta el robot"""
        v = self._lee_vmax()
        if v is None:
            self.escribe_log(f"VMAX debe estar entre {VMAX_MIN} y {VMAX_MAX}")
            return False
        self.escala_vmax.set(v)
        if (v != self.vmax_enviado or v != self.datos.vmax) and self.enviar(f"VMAX={v}/"):
            self.vmax_enviado = v
        return True

    def _lee_vmax(self):
        try:
            v = int(float(self.v_vmax.get()))
        except ValueError:
            return None
        return v if VMAX_MIN <= v <= VMAX_MAX else None

    def _suelta_vmax(self, _evento):
        """al soltar el slider: la casilla toma su valor y se manda al robot"""
        self.v_vmax.set(str(self.escala_vmax.get()))
        self.aplica_vmax()

    def corrige(self, signo):
        """+1: se va a la izquierda (el derecho corre mas): sube k, frena el derecho.
        -1: se va a la derecha (el izquierdo corre mas): baja k, frena el izquierdo"""
        k = self._lee_k()
        if k is None:
            k = 1.0
        k = min(KGIRO_MAX, max(KGIRO_MIN, k + signo * float(self.v_paso.get())))
        self.v_kgiro.set(f"{k:.3f}")
        self.aplica_k()

    def _entero(self, var, minimo, maximo, nombre):
        try:
            v = int(float(var.get()))
        except ValueError:
            v = None
        if v is None or not minimo <= v <= maximo:
            self.escribe_log(f"{nombre} debe estar entre {minimo} y {maximo}")
            return None
        return v

    def prueba_recto(self):
        vel = self._entero(self.v_test_vel, -TEST_VEL_MAX, TEST_VEL_MAX, "Velocidad")
        ms = self._entero(self.v_test_ms, TEST_MS_MIN, TEST_MS_MAX, "Tiempo")
        if vel is None or ms is None or not self.aplica_k() or not self.aplica_v():
            return
        self.enviar(f"TRECTO={vel},{ms}/")

    def prueba_giro(self, direccion):
        ms = self._entero(self.v_giro_ms, TEST_MS_MIN, TEST_MS_MAX, "Tiempo de giro")
        if ms is None or not self.aplica_k() or not self.aplica_v():
            return
        self.enviar(f"TGIRO={direccion},{ms}/")

    # ------------------------------------------------------------ recepcion
    def _tick_datos(self):
        """procesa lo recibido. Corre seguido para no atrasar los datos"""
        if self.enlace:
            try:
                while True:
                    tipo, contenido = self.enlace.entrada.get_nowait()
                    if tipo == "error":
                        self.escribe_log(f"Error del puerto: {contenido}")
                        self.desconectar()
                        break
                    self.buf += contenido
            except queue.Empty:
                pass
            if self.buf:
                eventos, self.buf = separa(self.buf)
                for ev in eventos:
                    if ev[0] == "dato":
                        self.datos.actualiza(ev[1], ev[2])
                        if ev[1] == "VGIRO":     # KGIRO llega justo antes
                            self._sincroniza_casillas()
                    else:
                        self.escribe_log(ev[1])
                        # el robot cambio sus valores: el dato que llegue DESPUES de esta
                        # linea ya es el nuevo (uno anterior aun puede venir en camino)
                        if ev[1].startswith(RESPUESTAS_SINCRONIZAN):
                            self.sincronizar = True
        self.id_datos = self.after(40, self._tick_datos)

    def _sincroniza_casillas(self):
        """al conectar, al reiniciarse el robot o tras leer la flash, las casillas toman
        lo que tiene el robot"""
        d = self.datos
        if self.sincronizar and d.kgiro is not None:
            self.sincronizar = False
            self.v_kgiro.set(f"{d.kgiro:.3f}")
            self.v_vgiro.set(str(d.vgiro))
            if d.vmax is not None:
                self.v_vmax.set(str(d.vmax))
                self.escala_vmax.set(d.vmax)

    def _tick_pantalla(self):
        """redibuja 10 veces por segundo"""
        d = self.datos
        vivo = self.enlace is not None and (time.monotonic() - d.ultimo) < SIN_DATOS_S
        self.gadget_bat.objetivo = d.bateria_v if vivo else None   # sin enlace no mostrar un voltaje viejo

        if self.enlace and not vivo:
            self.i_conexion.set("Conectado, pero el robot no manda datos")
            self.lbl_conexion.configure(fg=ROJO)
        elif vivo and "no manda" in self.i_conexion.get():
            self.i_conexion.set("Conectado")
            self.lbl_conexion.configure(fg=VERDE)

        if vivo and d.kgiro is not None:
            self.i_kgiro.set(f"{d.kgiro:.3f}")
            self.i_vgiro.set(str(d.vgiro))
            self.i_vmax.set("--" if d.vmax is None else str(d.vmax))
            izq, der = factores(d.kgiro)
            self.i_factores.set(f"izquierdo {izq * 100:.1f} %   derecho {der * 100:.1f} %")
            self.i_estado.set(ESTADOS[d.estado] if d.estado in range(len(ESTADOS)) else "--")
            self.i_radio.set(" / ".join(f"{c:.0f}" for c in d.radio[:3]))
            self.i_flash.set("Guardado en flash" if d.guardado else "Sin guardar en flash")
            self.lbl_flash.configure(fg=VERDE if d.guardado else ROJO)
            for i in range(NUM_LIDAR):
                self.barras.set(i, d.lidar[i], d.lidar_ok[i])
            for i in range(NUM_PISO):
                self.piso.set(i, d.piso[i])

            if d.imu_ok:
                self.i_imu.set("")
                self.i_pitch.set(f"{d.pitch:.1f}")
                self.i_roll.set(f"{d.roll:.1f}")
                self.i_yaw.set(f"{d.yaw:.1f}")
                self.i_acel.set(f"X {d.ax:+.2f}    Y {d.ay:+.2f}    Z {d.az:+.2f}  m/s2")
                n = GRAF_MUESTRAS
                self.grafica.redibuja((list(d.hist_ax)[-n:], list(d.hist_ay)[-n:], list(d.hist_az)[-n:]))
            else:
                self.i_imu.set("El BNO055 no respondio al arrancar")
        if not vivo:                  # sin enlace no dejar un led encendido con un dato viejo
            for i in range(NUM_PISO):
                self.piso.set(i, None)
        self.id_pantalla = self.after(100, self._tick_pantalla)

    def _tick_anima(self):
        """cuadros de la animacion de la bateria, 25 por segundo"""
        self.gadget_bat.anima()
        self.id_anima = self.after(40, self._tick_anima)

    def escribe_log(self, texto):
        self.log.configure(state="normal")
        self.log.insert("end", texto + "\n")
        if int(self.log.index("end-1c").split(".")[0]) > 300:
            self.log.delete("1.0", "100.0")
        self.log.see("end")
        self.log.configure(state="disabled")


if __name__ == "__main__":
    Panel(demo="--demo" in sys.argv).mainloop()
