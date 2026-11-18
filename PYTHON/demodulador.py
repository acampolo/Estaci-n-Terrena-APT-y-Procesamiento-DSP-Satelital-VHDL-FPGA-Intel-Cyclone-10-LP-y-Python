#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ---------------------------------------------------------
# Proyecto: Estación Terrena Stratosky
# Descripción: GUI en PyQt6 para recepción de telemetría y
#              video desde FPGA vía puerto serie (UART), y para
#              demodular una imagen desde un archivo de audio.
#              (Incluye demodulador robusto y filtros OpenCV).
# ---------------------------------------------------------

import sys
import bisect
import math
import time
import struct
import argparse
import datetime

import numpy as np
import pyqtgraph as pg
import matplotlib.pyplot as plt
from PyQt6 import QtCore, QtGui, QtWidgets

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

try:
    from scipy import fft as sfft, signal as sps
    from scipy.io import wavfile
except ImportError:
    sfft = sps = wavfile = None

try:
    import cv2
except ImportError:
    cv2 = None

Qt = QtCore.Qt
AA = QtGui.QPainter.RenderHint.Antialiasing

BAUD = 921600
AVG_N = 10
ACCEL_SIGN = (1, 1, 1)

HDR = b"\xA5\x5A"
T_LINE, T_SHT, T_ADXL, T_MS, T_CAL = 0xA1, 0xB1, 0xB2, 0xB3, 0xB4
LINE_PIXELS = 256
BODY_LEN = {T_SHT: 7, T_ADXL: 7, T_MS: 7, T_CAL: 13}

# Estructura de la señal de audio que genera el emisor
FS_AUDIO = 48000          # Hz
MUESTRAS_PIXEL = 20
MUESTRAS_SYNC = 400
MUESTRAS_PORCHE = 240
MUESTRAS_LINEA = MUESTRAS_SYNC + MUESTRAS_PORCHE + LINE_PIXELS * MUESTRAS_PIXEL + MUESTRAS_PORCHE   # 6000
SYNC_MIN, SYNC_MAX = 280, 650
HEADER_MIN = 30000        # la cabecera dura 1 s = 48000 muestras

# Parámetros del demodulador tolerante a ruido
VENTANA = 150            # +- muestras en que se busca el sincronismo de cada linea
CONTRASTE_MIN = 0.15     # por debajo de esto no se considera un sincronismo
GANANCIA_PERIODO = 0.10  # velocidad con que el lazo aprende la deriva del reloj

# --- Funciones matemáticas y de sensores ---
def crc8_sht(data):
    crc = 0xFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80: crc = ((crc << 1) ^ 0x31) & 0xFF
            else: crc = (crc << 1) & 0xFF
    return crc

def ms5611_compensate(C, D1, D2):
    dT = D2 - (C[5] << 8)
    temp = 2000 + ((dT * C[6]) >> 23)
    off = (C[2] << 16) + ((C[4] * dT) >> 7)
    sens = (C[1] << 15) + ((C[3] * dT) >> 8)
    t2 = off2 = sens2 = 0
    if temp < 2000:
        t2 = (dT * dT) >> 31
        off2 = (5 * (temp - 2000) ** 2) >> 1
        sens2 = (5 * (temp - 2000) ** 2) >> 2
        if temp < -1500:
            off2 += 7 * (temp + 1500) ** 2
            sens2 += (11 * (temp + 1500) ** 2) >> 1
    temp -= t2
    off -= off2
    sens -= sens2
    pres = (((D1 * sens) >> 21) - off) >> 15
    return temp / 100.0, pres / 100.0

def altitude_isa(p_mbar):
    if p_mbar <= 0: return float("nan")
    if p_mbar >= 226.32: return 44330.77 * (1.0 - (p_mbar / 1013.25) ** 0.190263)
    if p_mbar >= 54.749: return 11000.0 + 6341.62 * math.log(226.32 / p_mbar)
    return 20000.0 + 216650.0 * ((p_mbar / 54.749) ** -0.029271 - 1.0)

# --- Parser de tramas ---
class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.C = None
        self.stats = dict(bytes=0, lines=0, sensors=0, bad=0, skipped=0)

    def _bad(self):
        self.stats["bad"] += 1; self.stats["skipped"] += 1
        del self.buf[:1]

    def feed(self, data):
        self.stats["bytes"] += len(data)
        buf = self.buf
        buf += data
        events = []
        while True:
            i = buf.find(HDR)
            if i < 0:
                keep = 1 if buf and buf[-1] == 0xA5 else 0
                self.stats["skipped"] += len(buf) - keep
                del buf[:len(buf) - keep]
                break
            if i:
                self.stats["skipped"] += i
                del buf[:i]
            if len(buf) < 3: break
            t = buf[2]
            if t == T_LINE:
                total = 4 + LINE_PIXELS
                if len(buf) < total + 2: break
                if buf[total:total + 2] != HDR:
                    self._bad(); continue
                events.append(("line", buf[3], bytes(buf[4:total])))
                self.stats["lines"] += 1
                del buf[:total]
            elif t in BODY_LEN:
                total = 3 + BODY_LEN[t]
                if len(buf) < total: break
                chk = t
                for b in buf[3:total - 1]: chk ^= b
                if chk != buf[total - 1]:
                    self._bad(); continue
                ev = self._sensor(t, bytes(buf[3:total - 1]))
                if ev:
                    events.append(ev); self.stats["sensors"] += 1
                del buf[:total]
            else:
                self._bad()
        return events

    def _sensor(self, t, body):
        if t == T_SHT:
            if crc8_sht(body[0:2]) != body[2] or crc8_sht(body[3:5]) != body[5]:
                self.stats["bad"] += 1; return None
            t_raw = (body[0] << 8) | body[1]
            h_raw = (body[3] << 8) | body[4]
            return ("sht", -45.0 + 175.0 * t_raw / 65535.0, min(100.0, 100.0 * h_raw / 65535.0))
        if t == T_ADXL:
            x, y, z = struct.unpack("<hhh", body)
            return ("adxl", x / 256.0, y / 256.0, z / 256.0)
        if t == T_CAL:
            c = [(body[2 * i] << 8) | body[2 * i + 1] for i in range(6)]
            if any(v in (0, 0xFFFF) for v in c):
                self.stats["bad"] += 1; return None
            self.C = [0] + c
            return ("calib", c)
        if t == T_MS:
            if self.C is None: return None
            d1 = (body[0] << 16) | (body[1] << 8) | body[2]
            d2 = (body[3] << 16) | (body[4] << 8) | body[5]
            if d1 == 0 or d2 == 0: return None
            temp, pres = ms5611_compensate(self.C, d1, d2)
            if not (-60.0 <= temp <= 85.0 and 5.0 <= pres <= 1300.0):
                self.stats["bad"] += 1; return None
            return ("ms", temp, pres)
        return None

# ----------------------------------------------------------------------------- 
# DEMODULACIÓN DE AUDIO (Tolerante a ruido SDR)
# ----------------------------------------------------------------------------- 
def leer_audio(ruta):
    if wavfile is None:
        raise RuntimeError("Falta scipy: pip install scipy")
    fs, x = wavfile.read(ruta)
    x = np.asarray(x, dtype=np.float64)
    if x.ndim > 1:
        x = x.mean(axis=1)
    return int(fs), x

def estimar_portadora(x):
    f, p = sps.welch(x, fs=FS_AUDIO, nperseg=16384)
    p = np.convolve(p, np.ones(5) / 5.0, mode="same")
    m = (f >= 1000) & (f <= 2800)
    f0 = float(f[m][np.argmax(p[m])])
    total = p[f >= 100].sum()
    banda = p[np.abs(f - f0) <= 100].sum() / max(total, 1e-30)
    return f0, float(banda)

def envolvente(x, f0):
    n = len(x)
    nf = sfft.next_fast_len(n)
    X = sfft.fft(x, nf)
    f = np.abs(sfft.fftfreq(nf, 1.0 / FS_AUDIO))
    d = np.abs(f - f0)
    m = np.zeros(nf)
    m[d <= 1100] = 1.0
    r = (d > 1100) & (d < 1400)
    m[r] = 0.5 + 0.5 * np.cos(np.pi * (d[r] - 1100) / 300.0)
    h = np.zeros(nf)
    h[0] = 1.0
    if nf % 2 == 0:
        h[nf // 2] = 1.0
        h[1:nf // 2] = 2.0
    else:
        h[1:(nf + 1) // 2] = 2.0
    return np.abs(sfft.ifft(X * h * m))[:n]

def contraste(cs, pos):
    pos = np.asarray(pos)
    n = len(cs) - 1
    ok = (pos >= MUESTRAS_PORCHE) & (pos + MUESTRAS_SYNC + MUESTRAS_PORCHE <= n)
    p = np.where(ok, pos, MUESTRAS_PORCHE)
    a = (cs[p + MUESTRAS_SYNC] - cs[p]) / MUESTRAS_SYNC
    b1 = (cs[p] - cs[p - MUESTRAS_PORCHE]) / MUESTRAS_PORCHE
    b2 = (cs[p + MUESTRAS_SYNC + MUESTRAS_PORCHE] - cs[p + MUESTRAS_SYNC]) / MUESTRAS_PORCHE
    b = 0.5 * (b1 + b2)
    return np.where(ok, (a - b) / (a + b + 1e-12), -1.0)

def buscar_ancla(cs, n):
    paso = 8
    offs = np.arange(0, MUESTRAS_LINEA, paso)
    nl = n // MUESTRAS_LINEA
    mejor = (-1.0, 0)
    for c0 in range(0, max(1, nl - 20), 10):
        k = np.arange(c0, min(nl, c0 + 20))
        pos = offs[:, None] + k[None, :] * MUESTRAS_LINEA
        sc = np.clip(contraste(cs, pos), 0, None).mean(axis=1)
        i = int(np.argmax(sc))
        if sc[i] > mejor[0]:
            mejor = (float(sc[i]), int(offs[i] + (c0 + 10) * MUESTRAS_LINEA))
    return mejor[1], mejor[0]

def seguir(cs, n, e0, sentido, umbral, periodo):
    pos_l, con_l, val_l = [], [], []
    e = float(e0)
    per = float(periodo)
    desp = np.arange(-VENTANA, VENTANA + 1, 2)
    while (MUESTRAS_PORCHE + VENTANA < e < n - MUESTRAS_LINEA + VENTANA):
        cand = np.round(e).astype(int) + desp
        c = contraste(cs, cand)
        i = int(np.argmax(c))
        b, cb = float(cand[i]), float(c[i])
        if cb > umbral:
            pos_l.append(b); con_l.append(cb); val_l.append(True)
            per += GANANCIA_PERIODO * sentido * (b - e)
            per = min(max(per, MUESTRAS_LINEA * 0.98), MUESTRAS_LINEA * 1.02)
            e = b + sentido * per
        else:
            pos_l.append(e); con_l.append(cb); val_l.append(False)
            e = e + sentido * per
    return pos_l, con_l, val_l, per

def demodular_audio(ruta):
    fs, x = leer_audio(ruta)
    dur = len(x) / fs
    x = x - x.mean()
    if fs != FS_AUDIO:
        g = math.gcd(FS_AUDIO, fs)
        x = sps.resample_poly(x, FS_AUDIO // g, fs // g)
    n = len(x)
    if n < 3 * MUESTRAS_LINEA or np.max(np.abs(x)) <= 0:
        raise ValueError("El audio esta vacio o es demasiado corto")

    f0, banda = estimar_portadora(x)
    env = envolvente(x, f0)
    cs = np.concatenate(([0.0], np.cumsum(env)))

    e0, sc_ancla = buscar_ancla(cs, n)
    k_anc = np.arange(-5, 6) * MUESTRAS_LINEA + e0
    c_anc = contraste(cs, k_anc)
    umbral = max(CONTRASTE_MIN, 0.4 * float(np.median(c_anc[c_anc > 0])) if np.any(c_anc > 0) else CONTRASTE_MIN)

    pf, cf, vf, per_f = seguir(cs, n, e0, +1, umbral, MUESTRAS_LINEA)
    pb, cb, vb, per_b = seguir(cs, n, e0 - MUESTRAS_LINEA, -1, umbral, MUESTRAS_LINEA)
    pos = np.array(pb[::-1] + pf)
    con = np.array(cb[::-1] + cf)
    val = np.array(vb[::-1] + vf)
    if len(pos) < 20:
        return [], (f"Audio: {dur:.1f} s | portadora {f0:.0f} Hz ({banda * 100:.0f} % de la energia) | "
                    f"el audio es demasiado corto para una imagen")

    k0 = None
    for k in range(len(val) - 4):
        if val[k:k + 4].sum() >= 3:
            k0 = k
            break
    forzado = k0 is None
    if forzado:
        validas = np.where(val)[0]
        k0 = int(validas[0]) if len(validas) else int(np.argmax(con))
    pos_img = pos[k0:k0 + LINE_PIXELS]
    val_img = val[k0:k0 + LINE_PIXELS]
    con_img = con[k0:k0 + LINE_PIXELS]
    n_lin = len(pos_img)

    blanco = np.zeros(n_lin)
    negro = np.zeros(n_lin)
    ruido = np.zeros(n_lin)
    for i, s in enumerate(pos_img.astype(int)):
        blanco[i] = np.median(env[s + 60:s + MUESTRAS_SYNC - 60])
        porche = env[s + MUESTRAS_SYNC + 60:s + MUESTRAS_SYNC + 180]
        negro[i] = np.median(porche)
        ruido[i] = np.std(porche)
    ventana = min(9, n_lin | 1)
    blanco_s = sps.medfilt(blanco, ventana)
    negro_s = sps.medfilt(negro, ventana)

    lineas = []
    for i, s in enumerate(pos_img.astype(int)):
        v0 = s + MUESTRAS_SYNC + MUESTRAS_PORCHE
        if v0 + LINE_PIXELS * MUESTRAS_PIXEL > n:
            break
        seg = env[v0:v0 + LINE_PIXELS * MUESTRAS_PIXEL].reshape(LINE_PIXELS, MUESTRAS_PIXEL)[:, 2:18].mean(axis=1)
        rango = blanco_s[i] - negro_s[i]
        if rango > 1e-12:
            pix = np.clip((seg - negro_s[i]) / rango * 255.0, 0, 255).astype(np.uint8)
        else:
            pix = np.zeros(LINE_PIXELS, dtype=np.uint8)
        lineas.append((i, pix.tobytes()))

    bloqueadas = int(val_img.sum())
    idx = np.where(val_img)[0]
    ppm = None
    if len(idx) > 20 and not forzado:
        ppm = ((pos_img[idx[-1]] - pos_img[idx[0]]) / ((idx[-1] - idx[0]) * MUESTRAS_LINEA) - 1.0) * 1e6
    antes = env[max(0, int(pos_img[0]) - 40000):max(0, int(pos_img[0]) - 2000)]
    cabecera = (len(antes) > 20000 and float(np.median(con_img)) >= 0.25
                and np.median(antes) > 0.6 * np.median(blanco_s))
    snr = 20.0 * math.log10(max(float(np.median(blanco)), 1e-30) / max(float(np.median(ruido)), 1e-30))
    txt = (f"Audio: {dur:.1f} s a {fs} Hz | portadora: {f0:.0f} Hz ({banda * 100:.0f} % de la energia) | "
           f"cabecera: {'si' if cabecera else 'no'} | sincronismos: {bloqueadas}/{n_lin} lineas | "
           f"contraste: {float(np.median(con_img)):.2f} | "
           f"deriva de reloj: {'--' if ppm is None else f'{ppm:+.0f} ppm'} | "
           f"relacion senal/ruido: {'mas de 60 dB' if snr > 60 else f'{snr:.0f} dB'}")
    if float(np.median(con_img)) < 0.25 and not forzado:
        txt += " | SENAL DEBIL: contraste bajo, la imagen saldra con mucho ruido"
    if forzado:
        txt += (" | MODO FORZADO: no hubo sincronismos claros, la imagen puede salir desplazada "
                f"o con ruido (mejor contraste visto: {sc_ancla:.2f})")
    return lineas, txt


# ----------------------------------------------------------------------------- 
# HILOS DE FUENTE Y COMPONENTES VISUALES
# ----------------------------------------------------------------------------- 
class AudioSource(QtCore.QThread):
    events = QtCore.pyqtSignal(list)
    status = QtCore.pyqtSignal(str)

    def __init__(self, ruta):
        super().__init__()
        self.ruta, self.parser, self._run = ruta, Parser(), True

    def run(self):
        self.status.emit("Demodulando audio (procesamiento avanzado)...")
        try:
            lineas, info = demodular_audio(self.ruta)
        except Exception as e:
            self.status.emit(f"No se pudo demodular el audio: {e}")
            return
        self.status.emit(info)
        for idx, px in lineas:
            if not self._run:
                return
            self.events.emit([("line", idx, px)])
            time.sleep(0.008)

    def stop(self): self._run = False; self.wait(3000)

class SerialSource(QtCore.QThread):
    events = QtCore.pyqtSignal(list)
    status = QtCore.pyqtSignal(str)
    def __init__(self, port, baud=BAUD):
        super().__init__()
        self.port, self.baud, self.parser, self._run = port, baud, Parser(), True
    def run(self):
        try: ser = serial.Serial(self.port, self.baud, timeout=0.05, rtscts=False, dsrdtr=False)
        except Exception as e: self.status.emit(f"Error puerto {self.port}: {e}"); return
        self.status.emit(f"UART Abierto: {self.port}")
        try:
            while self._run:
                n = ser.in_waiting
                data = ser.read(n if n else 1)
                if data:
                    ev = self.parser.feed(data)
                    if ev: self.events.emit(ev)
        except Exception as e: self.status.emit(f"Fallo conexión: {e}")
        finally: ser.close()
    def stop(self): self._run = False; self.wait(1500)

class Averager:
    def __init__(self, n): self.n, self.acc, self.cnt = n, None, 0
    def add(self, values):
        v = np.asarray(values, dtype=float)
        self.acc = v if self.acc is None else self.acc + v
        self.cnt += 1
        if self.cnt >= self.n:
            out = (self.acc / self.cnt).tolist()
            self.acc, self.cnt = None, 0
            return out
        return None

# --- Panel de Diagnóstico ---
class DataFusionPanel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)

        self.lbl_density = QtWidgets.QLabel("-- kg/m³")
        self.lbl_vib = QtWidgets.QLabel("NORMAL")
        self.lbl_link = QtWidgets.QLabel("ESPERANDO")

        style_val = "color: #d4d4d4; font-size: 11pt; font-weight: bold;"
        style_lbl = "color: #858585; font-size: 10pt;"

        grid = QtWidgets.QGridLayout()
        grid.addWidget(self._make_label("Densidad Aire (ρ):", style_lbl), 0, 0)
        grid.addWidget(self.lbl_density, 0, 1)
        self.lbl_density.setStyleSheet(style_val)

        grid.addWidget(self._make_label("Nivel Vibración:", style_lbl), 1, 0)
        grid.addWidget(self.lbl_vib, 1, 1)
        self.lbl_vib.setStyleSheet(style_val)

        grid.addWidget(self._make_label("Salud Enlace UART:", style_lbl), 2, 0)
        grid.addWidget(self.lbl_link, 2, 1)
        self.lbl_link.setStyleSheet(style_val)

        layout.addLayout(grid)
        self.g_hist = []

    def _make_label(self, text, style):
        lbl = QtWidgets.QLabel(text)
        lbl.setStyleSheet(style)
        return lbl

    def update_diagnostics(self, temp, pres, g_total, link_quality):
        if temp is not None and pres is not None:
            p_pascal = pres * 100
            t_kelvin = temp + 273.15
            rho = p_pascal / (287.05 * t_kelvin)
            self.lbl_density.setText(f"{rho:.3f} kg/m³")

        if g_total is not None:
            self.g_hist.append(g_total)
            if len(self.g_hist) > 10: self.g_hist.pop(0)
            ruido = max(self.g_hist) - min(self.g_hist)
            if ruido > 0.15:
                self.lbl_vib.setText("ALTA VIBRACIÓN")
                self.lbl_vib.setStyleSheet("color: #d16969; font-size: 11pt; font-weight: bold;")
            else:
                self.lbl_vib.setText("ESTABLE")
                self.lbl_vib.setStyleSheet("color: #4ec9b0; font-size: 11pt; font-weight: bold;")

        if link_quality > 98:
            self.lbl_link.setText(f"ÓPTIMO ({link_quality}%)")
            self.lbl_link.setStyleSheet("color: #4ec9b0; font-size: 11pt; font-weight: bold;")
        else:
            self.lbl_link.setText(f"CON RUIDO ({link_quality}%)")
            self.lbl_link.setStyleSheet("color: #d16969; font-size: 11pt; font-weight: bold;")

# --- Indicadores visuales ---
class Thermometer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.value = None
        self.setMinimumSize(120, 180)

    def setValue(self, v):
        self.value = v; self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(AA)
        w, h = self.width(), self.height()

        bulb_r = min(w * 0.15, h * 0.08)
        tube_w = bulb_r * 0.8
        cx = w * 0.4
        top = 20
        bottom = h - bulb_r - 10
        y_max = top + tube_w / 2

        p.setPen(QtGui.QPen(QtGui.QColor("#3f3f46"), 1))
        p.setBrush(QtGui.QColor("#2d2d2d"))
        p.drawRoundedRect(QtCore.QRectF(cx - tube_w / 2, top, tube_w, bottom - top), tube_w / 2, tube_w / 2)
        p.drawEllipse(QtCore.QPointF(cx, bottom), bulb_r, bulb_r)

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#e53935"))
        p.drawEllipse(QtCore.QPointF(cx, bottom), bulb_r * 0.8, bulb_r * 0.8)

        if self.value is not None:
            frac = (self.value - (-40)) / (60 - (-40))
            fc = min(1.0, max(0.0, frac))
            yv = bottom - fc * (bottom - y_max)
            p.drawRoundedRect(QtCore.QRectF(cx - tube_w * 0.4, yv, tube_w * 0.8, bottom - yv), tube_w * 0.2, tube_w * 0.2)

        f = p.font()
        f.setPointSize(12)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QtGui.QColor("#d4d4d4"))
        txt = "--" if self.value is None else f"{self.value:.1f} °C"
        p.drawText(QtCore.QRectF(cx + bulb_r + 10, bottom - 15, w - cx - bulb_r, 30), Qt.AlignmentFlag.AlignLeft, txt)

class PressureGauge(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.value = None
        self.setMinimumSize(180, 180)

    def setValue(self, v):
        self.value = v; self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(AA)
        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy, r = w / 2.0, h / 2.0 + side * 0.04, side * 0.42

        rect = QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r)

        p.setPen(QtGui.QPen(QtGui.QColor("#3f3f46"), side * 0.04, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
        p.drawArc(rect, 225 * 16, -270 * 16)

        frac = 0.0
        if self.value is not None:
            frac = min(1.0, max(0.0, (self.value - 0) / 1100.0))
            p.setPen(QtGui.QPen(QtGui.QColor("#007acc"), side * 0.04, Qt.PenStyle.SolidLine, Qt.PenCapStyle.FlatCap))
            p.drawArc(rect, 225 * 16, int(-270 * frac * 16))

            a = math.radians(225 - 270 * frac)
            c, s = math.cos(a), -math.sin(a)
            p.setPen(QtGui.QPen(QtGui.QColor("#e53935"), 3))
            p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx + c * r * 0.85, cy + s * r * 0.85))

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor("#d4d4d4"))
        p.drawEllipse(QtCore.QPointF(cx, cy), side * 0.05, side * 0.05)

        f = p.font()
        f.setPointSize(12)
        f.setBold(True)
        p.setFont(f)
        p.setPen(QtGui.QColor("#d4d4d4"))
        txt = "--" if self.value is None else f"{self.value:.1f}"
        p.drawText(QtCore.QRectF(cx - r, cy + r * 0.5, 2 * r, side * 0.15), Qt.AlignmentFlag.AlignCenter, txt)

        f.setPointSize(8)
        f.setBold(False)
        p.setFont(f)
        p.setPen(QtGui.QColor("#858585"))
        p.drawText(QtCore.QRectF(cx - r, cy + r * 0.7, 2 * r, side * 0.1), Qt.AlignmentFlag.AlignCenter, "mbar")

def rot_x(a): c = math.cos(a); s = math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def rot_y(a): c = math.cos(a); s = math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rot_z(a): c = math.cos(a); s = math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

class AccelPlate(QtWidgets.QWidget):
    HALF = np.array([48.0, 35.0, 2.5])
    FACES = [
        ([1, 5, 7, 3], (0, 0, 1), (32, 34, 38)), ([0, 4, 5, 1], (0, -1, 0), (22, 24, 28)),
        ([4, 6, 7, 5], (1, 0, 0), (22, 24, 28)), ([0, 1, 3, 2], (-1, 0, 0), (22, 24, 28)),
        ([2, 3, 7, 6], (0, 1, 0), (22, 24, 28)), ([0, 2, 6, 4], (0, 0, -1), (15, 17, 20))
    ]
    def __init__(self):
        super().__init__()
        self.setMinimumSize(360, 190)
        self.roll = self.pitch = self.t_roll = self.t_pitch = 0.0
        self.g = (0.0, 0.0, 1.0)
        self.view = rot_x(math.radians(58)) @ rot_z(math.radians(-32))
        self.verts = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float) * self.HALF
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._animate)
        self.timer.start(33)

    def setAccel(self, x, y, z):
        self.g = (x, y, z)
        self.t_roll = math.atan2(y, z)
        self.t_pitch = math.atan2(-x, math.hypot(y, z))

    def _animate(self):
        def step(cur, tgt):
            d = (tgt - cur + math.pi) % (2 * math.pi) - math.pi
            return cur + d * 0.25
        r, p = step(self.roll, self.t_roll), step(self.pitch, self.t_pitch)
        if abs(r - self.roll) > 1e-4 or abs(p - self.pitch) > 1e-4:
            self.roll, self.pitch = r, p
            self.update()

    def _proj(self, pts, rot, scale, cx, cy):
        v = (self.view @ (rot @ np.asarray(pts, dtype=float).T)).T
        k = 1.0 / (1.0 + v[:, 1] / 520.0)
        return np.column_stack((cx + v[:, 0] * k * scale, cy - v[:, 2] * k * scale)), v

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(AA)
        w, h = self.width(), self.height()
        cx, cy, scale = w * 0.5, h * 0.55, min(w / 150.0, h / 105.0)
        R = rot_y(self.pitch) @ rot_x(self.roll)

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(0, 0, 0, 40))
        p.drawEllipse(QtCore.QPointF(cx, cy + h * 0.25), w * 0.30, h * 0.05)

        pts, _ = self._proj(self.verts, R, scale, cx, cy)
        light = np.array([-0.35, -0.55, 0.75])
        light /= np.linalg.norm(light)
        order = []
        for idx, n, base in self.FACES:
            nv = self.view @ (R @ np.array(n, dtype=float))
            if nv[1] >= 0: continue
            depth = float(np.mean((self.view @ (R @ self.verts[idx].T)).T[:, 1]))
            order.append((depth, idx, nv, base))
        order.sort(key=lambda t: -t[0])

        for _, idx, nv, base in order:
            shade = 0.55 + 0.45 * max(0.0, float(np.dot(nv, light)))
            col = QtGui.QColor(*[min(255, int(c * shade * 1.35)) for c in base])
            p.setPen(QtGui.QPen(QtGui.QColor(20, 20, 25), 1))
            p.setBrush(col)
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*pts[i]) for i in idx]))

        zt = self.HALF[2] + 0.2
        for (x0, x1, y0, y1, c) in ((-14, 14, -14, 14, (20, 20, 20)), (-48, -36, -8, 8, (150, 150, 150))):
            quad = np.array([[x0, y0, zt], [x1, y0, zt], [x1, y1, zt], [x0, y1, zt]])
            q, _ = self._proj(quad, R, scale, cx, cy)
            p.setPen(QtGui.QPen(QtGui.QColor(20, 20, 25), 1)); p.setBrush(QtGui.QColor(*c))
            p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*pt) for pt in q]))

        hole_pts = []
        for y in [-30, 0, 30]: hole_pts.extend([[45, y, zt], [-45, y, zt]])
        for x in [-25, 25]: hole_pts.extend([[x, 32, zt], [x, -32, zt]])
        hq, _ = self._proj(hole_pts, R, scale, cx, cy)
        for pt in hq:
            p.setPen(Qt.PenStyle.NoPen); p.setBrush(QtGui.QColor(180, 150, 50))
            p.drawEllipse(QtCore.QPointF(*pt), 3 * scale/3.5, 3 * scale/3.5)
            p.setBrush(QtGui.QColor(20, 20, 20))
            p.drawEllipse(QtCore.QPointF(*pt), 1.5 * scale/3.5, 1.5 * scale/3.5)

        for vec, color, name in (((70, 0, 0), "#d16969", "X"), ((0, 55, 0), "#4ec9b0", "Y"), ((0, 0, 45), "#569cd6", "Z")):
            q, _ = self._proj(np.array([[0, 0, 0], vec], dtype=float), R, scale, cx, cy)
            p.setPen(QtGui.QPen(QtGui.QColor(color), 2)); p.drawLine(QtCore.QPointF(*q[0]), QtCore.QPointF(*q[1]))

class ImageView(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.image = None
        self.line = 0
        self.setMinimumSize(340, 340)

    def setImage(self, img, line):
        self.image = img
        self.line = line
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.SmoothPixmapTransform)
        w, h = self.width(), self.height()
        side = min(w, h - 2)
        x0, y0 = (w - side) / 2.0, 1.0
        rect = QtCore.QRectF(x0, y0, side, side)

        p.fillRect(rect, QtGui.QColor("#2d2d2d"))
        if self.image is not None:
            p.drawImage(rect, self.image)
            y = y0 + (self.line + 1) / 256.0 * side
            p.setPen(QtGui.QPen(QtGui.QColor(0, 122, 204, 180), 1.5))
            p.drawLine(QtCore.QPointF(x0, y), QtCore.QPointF(x0 + side, y))

        p.setPen(QtGui.QPen(QtGui.QColor("#3f3f46"), 1))
        p.drawRect(rect)

# --- ESTILO "IDE DE INGENIERÍA" ---
STYLE = """
QMainWindow, QWidget {
    background-color: #1e1e1e;
    color: #d4d4d4;
    font-family: 'Segoe UI', Arial, sans-serif;
    font-size: 10pt;
}

QGroupBox {
    border: 1px solid #3f3f46;
    border-radius: 4px;
    margin-top: 2ex;
    padding-top: 10px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 5px;
    color: #4ec9b0;
    font-weight: bold;
}

QPushButton {
    background-color: #333337;
    border: 1px solid #3f3f46;
    border-radius: 3px;
    padding: 6px 12px;
}
QPushButton:hover { background-color: #3e3e42; }
QPushButton:pressed { background-color: #007acc; border-color: #007acc; }
QPushButton:checked { background-color: #007acc; border-color: #007acc; color: white; }

QComboBox {
    background-color: #2d2d2d;
    border: 1px solid #3f3f46;
    border-radius: 3px;
    padding: 4px;
}
QComboBox:hover { border-color: #007acc; }

QProgressBar {
    border: 1px solid #3f3f46;
    border-radius: 3px;
    text-align: center;
    background-color: #2d2d2d;
}
QProgressBar::chunk { background-color: #007acc; }

QScrollArea { border: none; }
QStatusBar { background-color: #007acc; color: white; font-weight: bold;}
"""

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.setWindowTitle("Stratosky - Interfaz de Recepción FPGA (OpenCV Enhanced)")

        scr = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1400, scr.width() - 40), min(850, scr.height() - 60))
        self.source = None

        self.frame = np.zeros((256, 256), dtype=np.uint8)
        self.last_line = 0
        self.dirty = False
        self.last_median = 0.0
        self.filtered = None
        self.cmap = plt.get_cmap('gray')

        self.av_sht, self.av_ms, self.av_adxl = Averager(AVG_N), Averager(AVG_N), Averager(AVG_N)
        self.prev_stats = (time.time(), 0)

        self.hist_len = 150
        self.data_alt = np.full(self.hist_len, np.nan)
        self.data_temp = np.full(self.hist_len, np.nan)
        self.data_x = np.full(self.hist_len, np.nan)
        self.data_y = np.full(self.hist_len, np.nan)
        self.data_z = np.full(self.hist_len, np.nan)

        self._build_ui()

        self.timer_img = QtCore.QTimer(self)
        self.timer_img.timeout.connect(self.refresh_image)
        self.timer_img.start(50)

        self.current_t = None
        self.current_p = None

        if args.audio: self.load_audio(args.audio)
        elif args.port:
            self.set_port(args.port)
            self.connect_serial()

    def _build_ui(self):
        central, scroll = QtWidgets.QWidget(), QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True); scroll.setWidget(central)
        self.setCentralWidget(scroll)
        root = QtWidgets.QVBoxLayout(central)

        bar = QtWidgets.QHBoxLayout()
        bar.addWidget(QtWidgets.QLabel("Dispositivo UART:"))
        self.cmb_port = QtWidgets.QComboBox(); self.cmb_port.setEditable(True); self.cmb_port.setMinimumWidth(200)
        bar.addWidget(self.cmb_port)

        btn_ref = QtWidgets.QPushButton("Escanear"); btn_ref.clicked.connect(self.refresh_ports); bar.addWidget(btn_ref)
        self.btn_conn = QtWidgets.QPushButton("Conectar FPGA"); self.btn_conn.clicked.connect(self.toggle_connection); bar.addWidget(self.btn_conn)
        btn_audio = QtWidgets.QPushButton("Subir audio"); btn_audio.clicked.connect(lambda: self.load_audio()); bar.addWidget(btn_audio)
        bar.addStretch(1)
        root.addLayout(bar)
        self.refresh_ports()

        body = QtWidgets.QHBoxLayout()
        root.addLayout(body, 1)

        left = QtWidgets.QVBoxLayout()
        body.addLayout(left, 3)

        g_acc = QtWidgets.QGroupBox("Acelerómetro (ADXL345)")
        l = QtWidgets.QVBoxLayout(g_acc)
        self.plate = AccelPlate()
        l.addWidget(self.plate)
        left.addWidget(g_acc, 3)

        g_t = QtWidgets.QGroupBox("Humedad / Temp (SHT31)")
        l = QtWidgets.QHBoxLayout(g_t)
        self.therm = Thermometer()
        l.addWidget(self.therm, 1)

        vbox = QtWidgets.QVBoxLayout()
        self.bar_h = QtWidgets.QProgressBar()
        self.bar_h.setRange(0, 100)
        self.bar_h.setFormat("%v % RH")
        vbox.addStretch(1)
        vbox.addWidget(QtWidgets.QLabel("Humedad Relativa:"))
        vbox.addWidget(self.bar_h)
        vbox.addStretch(1)
        l.addLayout(vbox, 2)
        left.addWidget(g_t, 2)

        g_p = QtWidgets.QGroupBox("Barómetro (MS5611)")
        l = QtWidgets.QHBoxLayout(g_p)
        self.gauge = PressureGauge()
        l.addWidget(self.gauge, 1)
        self.lbl_alt = QtWidgets.QLabel("Altitud:\n-- m")
        self.lbl_alt.setAlignment(Qt.AlignmentFlag.AlignCenter)
        f = self.lbl_alt.font(); f.setPointSize(12); self.lbl_alt.setFont(f)
        l.addWidget(self.lbl_alt, 1)
        left.addWidget(g_p, 2)

        pg.setConfigOption('background', '#1e1e1e')
        pg.setConfigOption('foreground', '#d4d4d4')

        self.plot_widget = pg.GraphicsLayoutWidget()
        body.addWidget(self.plot_widget, 4)

        col_blue = (86, 156, 214)
        col_orange = (206, 145, 120)
        col_green = (78, 201, 176)

        p_alt = self.plot_widget.addPlot(title="Altitud (m)")
        p_alt.showGrid(x=True, y=True, alpha=0.3)
        self.curve_alt = p_alt.plot(pen=pg.mkPen(col_blue, width=2), fillLevel=0, fillBrush=(*col_blue, 50))
        self.plot_widget.nextRow()

        p_temp = self.plot_widget.addPlot(title="Temperatura (°C)")
        p_temp.showGrid(x=True, y=True, alpha=0.3)
        self.curve_temp = p_temp.plot(pen=pg.mkPen(col_orange, width=2), fillLevel=0, fillBrush=(*col_orange, 50))
        self.plot_widget.nextRow()

        p_acc = self.plot_widget.addPlot(title="Aceleración (g)")
        p_acc.showGrid(x=True, y=True, alpha=0.3)
        p_acc.addLegend(offset=(5, 5))
        self.curve_x = p_acc.plot(pen=pg.mkPen(col_orange, width=1.5), name="X")
        self.curve_y = p_acc.plot(pen=pg.mkPen(col_green, width=1.5), name="Y")
        self.curve_z = p_acc.plot(pen=pg.mkPen(col_blue, width=1.5), name="Z")

        right = QtWidgets.QVBoxLayout()
        body.addLayout(right, 3)

        self.data_panel = DataFusionPanel()
        right.addWidget(self.data_panel, 1)

        g_img = QtWidgets.QGroupBox("Buffer de Imagen")
        l = QtWidgets.QVBoxLayout(g_img)
        self.view = ImageView()
        l.addWidget(self.view, 1)

        ctl = QtWidgets.QHBoxLayout()
        self.chk_median = QtWidgets.QCheckBox("Filtros HD OpenCV")
        self.chk_median.setChecked(True)
        self.chk_median.stateChanged.connect(lambda _: self._mark_dirty())
        ctl.addWidget(self.chk_median)

        self.btn_vis = QtWidgets.QPushButton("Gris"); self.btn_vis.setCheckable(True); self.btn_vis.setChecked(True)
        self.btn_ir = QtWidgets.QPushButton("Termal"); self.btn_ir.setCheckable(True)
        self.btn_topo = QtWidgets.QPushButton("Topo"); self.btn_topo.setCheckable(True)

        self.btn_group = QtWidgets.QButtonGroup()
        self.btn_group.addButton(self.btn_vis, 1)
        self.btn_group.addButton(self.btn_ir, 2)
        self.btn_group.addButton(self.btn_topo, 3)
        self.btn_group.buttonClicked.connect(self._change_filter)

        ctl.addWidget(self.btn_vis)
        ctl.addWidget(self.btn_ir)
        ctl.addWidget(self.btn_topo)

        l.addLayout(ctl)

        lbl_info = QtWidgets.QLabel("Formato RX: 256x256 px")
        lbl_info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lbl_info.setStyleSheet("color: #858585; font-size: 9pt;")
        l.addWidget(lbl_info)

        right.addWidget(g_img, 3)

        self.status = self.statusBar()
        self.lbl_stat = QtWidgets.QLabel("Desconectado.")
        self.status.addWidget(self.lbl_stat, 1)

    def _change_filter(self, btn):
        if btn == self.btn_vis: self.cmap = plt.get_cmap('gray')
        elif btn == self.btn_ir: self.cmap = plt.get_cmap('inferno')
        elif btn == self.btn_topo: self.cmap = plt.get_cmap('terrain')
        self._mark_dirty()

    def refresh_ports(self):
        self.cmb_port.clear()
        if list_ports:
            for pt in list_ports.comports():
                self.cmb_port.addItem(f"{pt.device} - {pt.description}", pt.device)

    def set_port(self, port): self.cmb_port.setEditText(port)

    def current_port(self):
        data = self.cmb_port.currentData()
        text = self.cmb_port.currentText().strip()
        return data if (data and text.startswith(str(data))) else text.split()[0] if text else ""

    def toggle_connection(self):
        if self.source:
            self.stop_source()
            self.lbl_stat.setText("Puerto cerrado.")
        else: self.connect_serial()

    def connect_serial(self):
        if serial is None:
            self.lbl_stat.setText("Falta pyserial: pip install pyserial")
            return
        port = self.current_port()
        if port: self.start_source(SerialSource(port, BAUD))

    def load_audio(self, ruta=None):
        if wavfile is None:
            self.lbl_stat.setText("Falta scipy: pip install scipy")
            return
        if not ruta:
            ruta, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, "Subir audio", "", "Audio WAV (*.wav);;Todos los archivos (*.*)")
        if ruta: self.start_source(AudioSource(ruta))

    def start_source(self, src):
        self.stop_source()
        self.frame[:] = 0; self.last_line = 0; self._mark_dirty()

        self.data_alt.fill(np.nan); self.data_temp.fill(np.nan)
        self.data_x.fill(np.nan); self.data_y.fill(np.nan); self.data_z.fill(np.nan)
        self.curve_alt.setData(self.data_alt); self.curve_temp.setData(self.data_temp)
        self.curve_x.setData(self.data_x); self.curve_y.setData(self.data_y); self.curve_z.setData(self.data_z)

        self.source = src
        src.events.connect(self.on_events)
        src.status.connect(self.lbl_stat.setText)
        src.finished.connect(lambda s=src: self._source_finished(s))
        src.start()
        self.btn_conn.setText("Detener" if isinstance(src, AudioSource) else "Desconectar")
        self.btn_conn.setChecked(True)

    def _source_finished(self, src):
        if self.source is src:
            self.source = None
            self.btn_conn.setText("Conectar FPGA")
            self.btn_conn.setChecked(False)

    def stop_source(self):
        if self.source: self.source.stop(); self.source = None
        self.btn_conn.setText("Conectar FPGA")
        self.btn_conn.setChecked(False)

    def closeEvent(self, ev):
        self.stop_source()
        super().closeEvent(ev)

    def _mark_dirty(self):
        self.dirty = True; self.last_median = 0.0

    def on_events(self, events):
        for e in events:
            k = e[0]
            if k == "line":
                self.frame[e[1], :] = np.frombuffer(e[2], dtype=np.uint8)
                self.last_line = e[1]
                self.dirty = True
            elif k == "sht":
                _, t, h = e
                self.current_t = t
                self.therm.setValue(t)
                self.bar_h.setValue(int(round(h)))
                a = self.av_sht.add((t, h))
                if a:
                    self.data_temp[:-1] = self.data_temp[1:]; self.data_temp[-1] = a[0]; self.curve_temp.setData(self.data_temp)
            elif k == "ms":
                _, t, p = e
                self.current_p = p
                self.gauge.setValue(p)
                alt = altitude_isa(p)
                self.lbl_alt.setText(f"Altitud:\n{alt:,.0f} m")
                a = self.av_ms.add((t, p))
                if a:
                    self.data_alt[:-1] = self.data_alt[1:]; self.data_alt[-1] = altitude_isa(a[1]); self.curve_alt.setData(self.data_alt)
            elif k == "adxl":
                _, x, y, z = e
                x = x * ACCEL_SIGN[0]; y = y * ACCEL_SIGN[1]; z = z * ACCEL_SIGN[2]
                self.plate.setAccel(x, y, z)

                if self.source:
                    st = self.source.parser.stats
                    tot = st['sensors'] + st['lines'] + st['bad']
                    link = int(((tot - st['bad']) / tot) * 100) if tot > 0 else 100
                    self.data_panel.update_diagnostics(self.current_t, self.current_p, math.sqrt(x**2 + y**2 + z**2), link)

                a = self.av_adxl.add((x, y, z))
                if a:
                    self.data_x[:-1] = self.data_x[1:]; self.data_x[-1] = a[0]; self.curve_x.setData(self.data_x)
                    self.data_y[:-1] = self.data_y[1:]; self.data_y[-1] = a[1]; self.curve_y.setData(self.data_y)
                    self.data_z[:-1] = self.data_z[1:]; self.data_z[-1] = a[2]; self.curve_z.setData(self.data_z)

    # -------------------------------------------------------------------------
    # NUEVA PIPELINE DE FILTROS OPENCV PARA EXTRACCIÓN DE TEXTO/MEMES
    # -------------------------------------------------------------------------
    def processed_frame(self):
        img = self.frame.copy()
        
        if self.chk_median.isChecked():
            if cv2 is None:
                raise RuntimeError("Falta OpenCV. Ejecuta en tu terminal: pip install opencv-python")

            now = time.time()
            if self.filtered is None or now - self.last_median > 0.25:
                # 1. Filtro Bilateral: "Derrite" la lluvia y estática del fondo, pero protege 
                # agresivamente los bordes finos (letras, pistas de la PCB, fronteras).
                suavizado = cv2.bilateralFilter(img, d=5, sigmaColor=50, sigmaSpace=50)
                
                # 2. CLAHE (Contrast Limited Adaptive Histogram Equalization): Ecualiza el 
                # contraste por cuadrículas. Revela detalles ocultos en las sombras o nubes.
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
                self.filtered = clahe.apply(suavizado)
                
                self.last_median = now
            img = self.filtered
        else:
            # Contraste global estándar si los filtros HD están apagados
            lo, hi = np.percentile(img, (2, 98))
            if hi > lo: 
                img = np.clip((img.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)

        colored = self.cmap(img / 255.0)
        return np.ascontiguousarray((colored[:, :, :3] * 255).astype(np.uint8))

    def refresh_image(self):
        if not self.dirty: return
        self.dirty = False
        arr = self.processed_frame()
        qimg = QtGui.QImage(arr.data, 256, 256, 256 * 3, QtGui.QImage.Format.Format_RGB888).copy()
        self.view.setImage(qimg, self.last_line)
        if self.chk_median.isChecked(): self.dirty = True

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--audio", help="archivo de audio WAV del que demodular la imagen")
    args = ap.parse_args()

    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)

    win = MainWindow(args)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()