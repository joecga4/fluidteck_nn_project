#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
daq.py — Capa de E/S sobre el cDAQ-9184 con `nidaqmx`.

############################################################################
#  ADVERTENCIA DE SEGURIDAD — LEER ANTES DE USAR                           #
#                                                                          #
#  Este programa puede MOVER un cilindro hidraulico capaz de 200 kN a       #
#  100 bar, sobre una planta con amortiguamiento practicamente nulo         #
#  (delta_h = 1.4e-3). Un comando equivocado en el AO no da un error de     #
#  software: rompe algo.                                                    #
#                                                                          #
#  Por eso:                                                                 #
#   * NINGUN subcomando escribe en el AO salvo que se pase --armar.         #
#   * --diag y --sensores son de SOLO LECTURA. Empezar siempre por ahi.     #
#   * La captura supervisa la posicion en vivo y aborta a cero.             #
#   * Los limites de posicion y fuerza son obligatorios, no opcionales.     #
#   * Esto NO sustituye a los enclavamientos fisicos (seta de emergencia,   #
#     limitadora de presion, finales de carrera). Son la ultima linea de    #
#     defensa y deben estar operativos.                                     #
############################################################################

============================ GUIA DE LECTURA ============================
Por que Python y no LabVIEW para capturar
------------------------------------------
Porque el cDAQ sabe temporizar por HARDWARE y eso es exactamente lo que un
modelo NARX necesita (CLAUDE.md §2.5.1):

  * La secuencia de excitacion se emite con el reloj del chasis, no con un lazo
    de software. El Ts es EXACTO, no "nominal con jitter".
  * El AI (NI 9222, muestreo simultaneo) arranca con el MISMO trigger que el AO,
    asi que comando y medida quedan alineados por hardware. Sin ese alineamiento
    el modelo aprende un retardo falso.

Un lazo de software en Windows -en LabVIEW o en Python, da igual- no puede
garantizar ninguna de las dos cosas. Para el LAZO CERRADO no queda mas remedio
que software (hay que leer, calcular y escribir dentro del periodo), y ahi si
entran la latencia de Ethernet y el jitter del SO: eso es lo que mide
`--latencia`, y su resultado decide el Ts de control alcanzable.

Estructura de la captura con supervision
-----------------------------------------
El AO NO se precarga entero y se lanza a ciegas: eso impediria reaccionar si el
vastago se acerca a un tope (y la ganancia real se conoce con un factor de 6.5x
de incertidumbre — CLAUDE.md §8). En su lugar:

    AO continuo, temporizado por hardware, alimentado por TROZOS
    AI continuo, temporizado por hardware, con el mismo trigger
    lazo supervisor (software, lento): lee AI -> comprueba limites ->
                                       escribe el siguiente trozo de AO

El Ts sigue siendo exacto (lo marca el reloj del chasis); el software solo tiene
que ir por delante rellenando el buffer. La latencia de reaccion ante un limite
es la profundidad del buffer (~0.3 s por defecto), que a la velocidad maxima de
la planta son decimas de milimetro.

MAPA DEL FICHERO
----------------
    1. Configuracion de la cadena de medida   que canal es que, y su escala
    2. Descubrimiento del hardware            localizar los modulos del chasis
    3. Diagnostico                            SOLO LECTURA
    4. Estado seguro y latencia del lazo      AO a 0 V, medida del lazo
    5. Digitales                              enclavamientos y arranque de la HPU
    6. Calibracion de los transductores       recta de presion por dos puntos
    7. Maniobras que mueven el actuador       sentido, curva u->v, saturacion, jog
    8. Captura                                excitacion + registro, por hardware

Los NUMEROS de la planta (areas, ley comando->velocidad, null, caudal maximo)
NO estan aqui: viven en `tools/planta.py`, que es la fuente unica del proyecto.

ORDEN DE TRABAJO en una sesion (protocolo completo en docs/protocolo_fase0.md)
------------------------------------------------------------------------------
    Nada de esto toca la planta:
      python tools/daq.py --diag                  # modulos + los 4 AI + fuerza
      python tools/daq.py --di                    # enclavamientos
      python tools/daq.py --sensores --secs 10    # sensores en vivo

    Escriben en el AO o en el DO, y por eso EXIGEN --armar:
      python tools/daq.py --cero --armar          # AO a 0 V (estado seguro)
      python tools/daq.py --hpu on --armar        # arranca la UPH y la para al salir
      python tools/daq.py --latencia --armar      # mide el lazo (el AO no se mueve)

    Maniobras que MUEVEN el actuador. Con --con-hpu arrancan y paran la bomba
    dentro del MISMO proceso, que es lo unico que garantiza que el permisivo
    siga afirmado durante toda la maniobra:
      python tools/daq.py --caracteriza --con-hpu --armar        # curva u -> v
      python tools/daq.py --jog 75 --con-hpu --armar             # recolocar
      python tools/daq.py --satura --signo 1 --con-hpu --armar   # techo de caudal
      python tools/daq.py --calibra-presion --man-a 30 --man-b 50 --armar
      python tools/daq.py --captura results/excitacion_train.csv                           --etiqueta train --con-hpu --armar

DOS COSAS QUE HAN COSTADO TIEMPO Y CONVIENE SABER DE ANTEMANO
--------------------------------------------------------------
  * ESPERAR 3 MINUTOS tras arrancar la HPU antes de medir nada. El transitorio
    de arranque dura eso, y contamina cualquier medida corta: causo una
    conclusion que hubo que retirar y varios abortos de prueba.
  * RESERVA DEL CHASIS: LabVIEW y NI MAX reservan los modulos y bloquean a
    Python (y al reves). Cerrar el VI y cualquier panel de prueba de MAX antes
    de empezar. Si algo falla con "resource already reserved", es esto.
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import time

import numpy as np

# Los numeros de la planta viven en UN solo sitio (tools/planta.py):
# geometria, ley comando->velocidad medida, null, caudal maximo. Estaban
# duplicados aqui y en gen_excitacion.py, y las areas del cilindro ademas
# escritas a mano en dos funciones. Si se re-mide la planta, se cambia alli.
from planta import A_A, A_B, PESO, U_NULL, V_MAX_EXT, fuerza_hidraulica

try:
    import nidaqmx
    from nidaqmx.constants import (AcquisitionType, Edge, LineGrouping,
                                   RegenerationMode, TaskMode, TerminalConfiguration)
    from nidaqmx.system import System
except ImportError:  # pragma: no cover
    print("ERROR: falta el paquete `nidaqmx`.  ->  python -m pip install nidaqmx")
    print("       (y el driver NI-DAQmx instalado en el sistema)")
    sys.exit(1)


# ============================================================================
# 1. CONFIGURACION DE LA CADENA DE MEDIDA
# ============================================================================
# Reparto CONFIRMADO por el laboratorio (2026-08-12):
#   AI0 = sensor de posicion del actuador   AI1 = celda de carga
#   AI2 = presion camara A                  AI3 = presion camara B
#
# Que AI2 y AI3 sean las DOS CAMARAS del actuador es mas util de lo que parece:
# P_A y P_B son VARIABLES DE ESTADO del modelo fisico (CLAUDE.md §3), asi que se
# puede validar el modelo contra estados internos y no solo contra la posicion.
#
# OJO con la formula de la fuerza. Como el cilindro es ASIMETRICO, la fuerza NO
# es A_p*(P_A - P_B) sino:
#       F = P_A*A_A - P_B*A_B      con A_A = 201.06 cm2 , A_B = 122.52 cm2
# Esto da una medida de fuerza INDEPENDIENTE de la celda, util para verificarla
# y para trabajar el lazo de fuerza sin probeta. Con la formula simetrica el
# error seria del 64% en el termino de A.
#
# P_A y P_B son ademas candidatos naturales a entrar en el regresor del NARX:
# llevan la informacion de la dinamica hidraulica que la posicion sola no
# muestra (la posicion es su integral y la filtra).
# RANGOS aportados por el laboratorio (2026-08-12) — sustituyen a los de la
# memoria, que estan desactualizados:
#   posicion  0-150 mm   (la memoria citaba un sensor MTS de 400 mm)
#   celda     0-200 kN
#   presion A (camara SIN vastago)  0-100 bar
#   presion B (camara CON vastago)  0-400 bar
#
# Por que A y B tienen rangos tan distintos: el cilindro es ASIMETRICO
# (A_A/A_B = 1.641), asi que al retener o frenar hay INTENSIFICACION de presion
# en la camara anular: P_B = P_A*(A_A/A_B). Con 100 bar en A se llega a 164 bar
# en B. El rango de 400 bar no es un capricho, es esa ecuacion con margen.
#
# PRESIONES: escala CALIBRADA POR DOS PUNTOS en el equipo (2026-08-12).
#
#   punto 0 : UPH parada, ambos manometros a 0 bar
#             V0_A = 1.9697 V     V0_B = 1.9801 V
#   punto 1 : UPH en marcha, AO sostenido en -0.370 V, media de 180 s
#             V1_A = 2.8987 V (manometro 30 bar)
#             V1_B = 4.7329 V (manometro 50 bar)
#
#   =>  ai2:  32.293 bar/V   (fondo de escala a 10 V: 259.3 bar)
#       ai3:  18.163 bar/V   (fondo de escala a 10 V: 145.7 bar)
#
# Que los dos puntos den ~2.0 V a presion nula confirma el span 4-20 mA sobre
# 500 ohm (2-10 V). Se usa el V0 MEDIDO y no 2.000 V exactos porque asi la recta
# absorbe tambien el offset del propio canal de entrada.
#
# VALIDACION INDEPENDIENTE (no circular): con el piston en equilibrio y sin carga
# externa debe cumplirse  P_A*A_A = P_B*A_B , es decir  P_B/P_A = A_A/A_B.
#       relacion de areas     A_A/A_B = 1.6410
#       relacion de presiones   50/30 = 1.6667      -> discrepancia 1.6 %
# Las presiones se leyeron a ojo en dos manometros y las areas salen de la
# geometria: que coincidan al 1.6 % confirma a la vez las areas asimetricas, el
# reparto ai2=camara grande / ai3=anular, y las propias lecturas.
#
# El balance de fuerzas paso de -141.3 kN (escala supuesta) a +0.80 kN una vez
# sumado el peso del conjunto — residuo del orden de la friccion de sellos.
#
# ⚠ PENDIENTE: los fondos de escala que salen (259 y 146 bar) NO coinciden con
# los 0-100 y 0-400 bar que reporto el laboratorio. La calibracion empirica cierra
# la fisica, asi que se usa esa; pero conviene mirar la placa de los transductores.
CANALES_AI = {
    "ai0": dict(nombre="posicion", unidad="mm", v_lo=0.0, v_hi=10.0, e_lo=0.0, e_hi=150.0),
    "ai1": dict(nombre="fuerza",   unidad="kN", v_lo=0.0, v_hi=10.0, e_lo=0.0, e_hi=200.0),
    "ai2": dict(nombre="presion_A", unidad="bar", v_lo=1.9697, v_hi=10.0,
                e_lo=0.0, e_hi=259.3),
    "ai3": dict(nombre="presion_B", unidad="bar", v_lo=1.9801, v_hi=10.0,
                e_lo=0.0, e_hi=145.7),
}


CANAL_AO = "ao0"          # comando a la servovalvula (via amplificador)

# --- SALIDAS DIGITALES (NI 9472) -------------------------------------------
# Reparto OBSERVADO en el sistema de control de LabVIEW (2026-08-12), leyendo
# los indicadores de salida digital del VI:
#
#     linea:        0     1     2     3     4     5     6     7
#     UPH apagada:  F     F     F     F     F     T     F     F
#     UPH encendida:T     F     F     F     F     T     F     F
#
# De ahi:
#   * line0 = ARRANQUE DE LA UPH. Es la unica que cambia.
#   * line5 = PERMISIVO, energizado SIEMPRE, incluso con la UPH parada.
#     La memoria (§2.5) dice que el NI 9472 "envia senial de parada de emergencia
#     desde la pantalla del programa": una salida que se mantiene a TRUE en
#     reposo y en marcha es justo eso, un permisivo en logica negada (mientras
#     este alto, no hay emergencia). NO SE CONFIRMO cual es su funcion exacta,
#     pero el tratamiento correcto es el mismo en cualquier caso: SOSTENERLA.
#
# CONSECUENCIA DE DISENIO — por eso se escribe el PUERTO ENTERO y no una linea
# suelta. Al liberar LabVIEW su tarea de DO, las salidas del modulo pueden volver
# a su estado por defecto (0) y tirar el permisivo. Python toma el puerto
# completo y afirma el patron conocido; asi el estado es siempre explicito y
# nunca queda a merced de lo que dejara la aplicacion anterior.
LINEA_HPU = 0
LINEA_PERMISIVO = 5

def _patron_do(hpu: bool) -> list:
    """Patron de las 8 salidas digitales. El permisivo va SIEMPRE alto."""
    p = [False] * 8
    p[LINEA_PERMISIVO] = True
    p[LINEA_HPU] = bool(hpu)
    return p

DO_SEGURO = _patron_do(False)     # permisivo alto, UPH parada
DO_MARCHA = _patron_do(True)      # permisivo alto, UPH en marcha

# --- ENTRADAS DIGITALES (NI 9421) ------------------------------------------
# Estado leido con la HPU apagada (2026-08-12): line0=1, line1=1, resto 0.
# Segun la memoria el 9421 recibe: motor encendido, saturacion de filtro,
# sobrepresion y respuesta de la seta. Que dos esten altas en reposo apunta a
# contactos NORMALMENTE CERRADOS ("seta no pulsada", "filtro no saturado"), pero
# es conjetura: la forma de confirmarlo es mirar cual cambia al arrancar la UPH,
# que es lo que hace `set_hpu`.
DI_ESPERADO_REPOSO = {0: 1, 1: 1, 2: 0, 3: 0, 4: 0, 5: 0, 6: 0, 7: 0}

# Limites de seguridad por software. Estrictamente interiores a los mecanicos.
# Dos ventanas distintas, y la diferencia importa:
#   * VENTANA DE TRABAJO [X_MIN_SEG, X_MAX_SEG]: la que se exige para EMPEZAR
#     una captura y la que se vigila durante la emision de la excitacion.
#   * LIMITES MECANICOS [X_MIN_JOG, X_MAX_JOG]: margen respecto a los topes
#     reales (0 y 150 mm). Son los que aplican durante una RECOLOCACION.
# Usar la ventana de trabajo tambien para el jog es un error: impide la unica
# maniobra que sirve para volver a entrar en ella. Se comprobo con el vastago
# parado en 5.2 mm — abortaba antes de moverse.
X_MIN_SEG = 20.0          # [mm] ventana de trabajo
X_MAX_SEG = 130.0         # [mm]
X_MIN_JOG = 4.0           # [mm] limites mecanicos con margen (carrera 0..150)
X_MAX_JOG = 146.0         # [mm]
F_MAX_SEG = 50.0          # [kN] muy por debajo de los 200 kN de la celda
# Limites de presion, revisados tras calibrar las escalas (§5.3c). El fondo de
# escala real de cada canal es 259.3 bar (A) y 145.7 bar (B), asi que un limite
# de 200 bar en B seria un chequeo MUERTO: el canal satura antes de alcanzarlo.
PA_MAX_SEG = 110.0        # [bar] camara A: por encima de la limitadora (100 bar)
PB_MAX_SEG = 130.0        # [bar] camara B: por debajo de su saturacion (145.7)
U_MAX_SEG = 10.0          # [V]  rango del NI 9263


def escala(v: np.ndarray, c: dict) -> np.ndarray:
    """Convierte voltios a unidades de ingenieria (lineal de dos puntos)."""
    return c["e_lo"] + (v - c["v_lo"]) * (c["e_hi"] - c["e_lo"]) / (c["v_hi"] - c["v_lo"])


# ============================================================================
# 2. DESCUBRIMIENTO DEL HARDWARE
# ============================================================================
def encuentra_modulos() -> dict:
    """Localiza el chasis y los modulos por TIPO DE PRODUCTO, no por alias.

    Se busca por tipo a proposito: el alias depende del numero de serie del
    chasis (`cDAQ9184-1ADC24CMod2`) y cambiaria si se sustituye el equipo. El
    tipo de producto (`NI 9222`) no.
    """
    s = System.local()
    hall = {"chasis": None, "ai": None, "ao": None, "di": None, "do": None}
    for d in s.devices:
        try:
            t = d.product_type
        except Exception:
            continue
        if "cDAQ-9184" in t:
            hall["chasis"] = d.name
        elif "9222" in t:
            hall["ai"] = d.name
        elif "9263" in t:
            hall["ao"] = d.name
        elif "9421" in t:
            hall["di"] = d.name
        elif "9472" in t:
            hall["do"] = d.name
    return hall


def _ai_chan(mods: dict, ch: str) -> str:
    return f"{mods['ai']}/{ch}"


def _ao_chan(mods: dict) -> str:
    return f"{mods['ao']}/{CANAL_AO}"

# --- Atajos para abrir tareas -----------------------------------------------
# La misma media docena de lineas aparecia SIETE veces en el fichero. Ademas de
# ruido, era una trampa: cambiar el `terminal_config` o el rango habia que
# acordarse de hacerlo en los siete sitios.

def _abre_ai(tarea, mods: dict, canales=None):
    """Anade los canales AI a `tarea` y la devuelve, para poder encadenar."""
    for ch in (canales or CANALES_AI):
        tarea.ai_channels.add_ai_voltage_chan(
            _ai_chan(mods, ch), min_val=-10.0, max_val=10.0,
            terminal_config=TerminalConfiguration.DIFF)
    return tarea


def _abre_ao(tarea, mods: dict):
    """Anade el canal de comando a la servovalvula y devuelve la tarea."""
    tarea.ao_channels.add_ao_voltage_chan(_ao_chan(mods), min_val=-10.0,
                                          max_val=10.0)
    return tarea


def lee_bloque(mods: dict, n: int = 1000, fs: float = 1000.0, canales=None):
    """Una rafaga de `n` muestras de los AI a `fs` Hz. SOLO LECTURA.

    Devuelve el array crudo [n_canales, n] en VOLTIOS. Para pasarlo a unidades
    de ingenieria, `a_ingenieria`.
    """
    with nidaqmx.Task() as t:
        _abre_ai(t, mods, canales)
        t.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.FINITE,
                                     samps_per_chan=n)
        return np.array(t.read(number_of_samples_per_channel=n,
                               timeout=max(10.0, 3 * n / fs)))


def a_ingenieria(d, canales=None) -> dict:
    """Bloque de voltios -> dict {nombre: media en unidades de ingenieria}."""
    cs = [CANALES_AI[c] for c in (canales or CANALES_AI)]
    return {c["nombre"]: float(escala(d[i], c).mean()) for i, c in enumerate(cs)}


# ============================================================================
# 3. DIAGNOSTICO — SOLO LECTURA
# ============================================================================
def diag() -> dict:
    """Enumera el sistema y lee una rafaga corta de los 4 AI. NO escribe nada."""
    print("=" * 74)
    print("DIAGNOSTICO DEL cDAQ  —  SOLO LECTURA, no se escribe en el AO")
    print("=" * 74)

    s = System.local()
    try:
        dv = s.driver_version
        print(f"  driver NI-DAQmx : {dv.major_version}.{dv.minor_version}.{dv.update_version}")
    except Exception as e:
        print(f"  !! no se pudo leer la version del driver: {e}")

    mods = encuentra_modulos()
    print("\n  Modulos localizados por tipo de producto:")
    for k, v in mods.items():
        print(f"    {k:8s} : {v if v else '-- NO ENCONTRADO --'}")

    faltan = [k for k in ("chasis", "ai", "ao") if not mods[k]]
    if faltan:
        print(f"\n  !! Faltan modulos imprescindibles: {faltan}")
        print("     Revisar NI MAX y la conexion Ethernet del chasis.")
        return mods

    print("\n  Lectura de los 4 canales AI (1000 muestras a 1 kHz):")
    try:
        with nidaqmx.Task() as t:
            for ch, c in CANALES_AI.items():
                t.ai_channels.add_ai_voltage_chan(
                    _ai_chan(mods, ch), min_val=-10.0, max_val=10.0,
                    terminal_config=TerminalConfiguration.DIFF,
                )
            t.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.FINITE,
                                         samps_per_chan=1000)
            data = np.array(t.read(number_of_samples_per_channel=1000))
    except Exception as e:
        print(f"    !! fallo la lectura: {type(e).__name__}: {e}")
        print("       Causas tipicas: el chasis esta apagado o desconectado, o")
        print("       LabVIEW tiene el modulo RESERVADO (CLAUDE.md §2.5.1).")
        return mods

    print(f"  {'canal':10s} {'senial':10s} {'V medio':>9s} {'V ruido':>9s}"
          f" {'ing. medio':>12s} {'unid':>5s}")
    for i, (ch, c) in enumerate(CANALES_AI.items()):
        v = data[i]
        ing = escala(v, c)
        print(f"  {ch:10s} {c['nombre']:10s} {v.mean():9.4f} {v.std():9.5f}"
              f" {ing.mean():12.3f} {c['unidad']:>5s}")

    print("\n  Que mirar aqui:")
    print("   * el ruido (V ruido) fija la velocidad minima que se podra medir;")
    print("     con la planta parada deberia ser de pocos mV.")
    print("   * si un canal esta clavado a 0 V exactos o a fondo de escala, el")
    print("     cableado o el reparto de CANALES_AI no es el que se supone.")
    print("   * la posicion leida debe coincidir con la que se ve en el equipo:")
    print("     es la comprobacion de que la escala 0-10 V -> 0-150 mm es correcta.")

    # Comprobacion CRUZADA de fuerza: la celda y las dos presiones son cadenas
    # independientes, asi que compararlas detecta un sensor caido o una escala
    # mal puesta sin tener que mover nada. Ojo al interpretarla sin probeta: la
    # celda mide la reaccion EXTERNA (marca ~0) y las presiones dan la fuerza
    # INTERNA del cilindro, que equilibra el peso.
    e = a_ingenieria(data)
    F_hid = fuerza_hidraulica(e["presion_A"], e["presion_B"])
    print(f"{chr(10)}  Fuerza por dos caminos independientes:")
    print(f"    celda de carga            : {e['fuerza']:+7.3f} kN")
    print(f"    P_A*A_A - P_B*A_B         : {F_hid:+7.3f} kN")
    print(f"    peso del conjunto movil   : {PESO/1e3:+7.3f} kN (siempre hacia +)")
    return mods


def lee_sensores(mods: dict, secs: float = 10.0, fs: float = 100.0) -> None:
    """Muestra los sensores en vivo. SOLO LECTURA."""
    n = int(fs * secs)
    print(f"\nLectura en vivo {secs:.0f} s a {fs:.0f} Hz  (SOLO LECTURA)")
    with nidaqmx.Task() as t:
        _abre_ai(t, mods)
        t.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS)
        t.start()
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < secs:
            blq = np.array(t.read(number_of_samples_per_channel=int(fs / 5)))
            vals = [escala(blq[i], c).mean() for i, c in enumerate(CANALES_AI.values())]
            linea = "  ".join(
                f"{c['nombre']}={v:8.3f} {c['unidad']}"
                for v, c in zip(vals, CANALES_AI.values())
            )
            print(f"\r  t={time.perf_counter()-t0:5.1f}s  {linea}", end="", flush=True)
        t.stop()
    print()


# ============================================================================
# 4. ESTADO SEGURO Y LATENCIA DEL LAZO
# ============================================================================
def pon_cero(mods: dict) -> None:
    """Escribe 0 V en el AO: el estado seguro (carrete en null nominal)."""
    with nidaqmx.Task() as t:
        _abre_ao(t, mods)
        t.write(0.0, auto_start=True)
    print("[ok] AO a 0.000 V")


def mide_latencia(mods: dict, n: int = 2000, fs_obj: float = 100.0) -> None:
    """Mide el lazo software: escribir AO + leer AI, punto a punto.

    Escribe SIEMPRE 0 V — no mueve la planta — pero recorre el mismo camino que
    recorreria un controlador. El resultado es lo que decide el Ts de control
    alcanzable (CLAUDE.md §6.1).
    """
    print(f"\nMEDIDA DEL LAZO DE SOFTWARE ({n} iteraciones, objetivo {fs_obj:.0f} Hz)")
    print("  (el AO se mantiene a 0.000 V: la planta no se mueve)")
    Ts = 1.0 / fs_obj
    t_rd = np.empty(n)
    t_wr = np.empty(n)

    with nidaqmx.Task() as tao, nidaqmx.Task() as tai:
        _abre_ao(tao, mods)
        tai.ai_channels.add_ai_voltage_chan(
            _ai_chan(mods, "ai0"), min_val=-10.0, max_val=10.0,
            terminal_config=TerminalConfiguration.DIFF,
        )
        # CLAVE: COMMIT + START explicitos, y una sola vez.
        # Si la tarea se deja sin arrancar, DAQmx hace un start/stop implicito
        # EN CADA read(), y sobre un chasis Ethernet eso cuesta ~200 ms por
        # iteracion. Medido: 202 ms de mediana sin esto. Con COMMIT la tarea
        # queda preprogramada en el hardware y cada operacion es solo la
        # transferencia.
        tai.control(TaskMode.TASK_COMMIT)
        tao.control(TaskMode.TASK_COMMIT)
        tai.start()
        tao.start()
        tao.write(0.0)

        objetivo = time.perf_counter()
        for k in range(n):
            objetivo += Ts
            t0 = time.perf_counter()
            tai.read()                          # leer la medida
            t1 = time.perf_counter()
            tao.write(0.0)                      # escribir la accion
            t2 = time.perf_counter()
            t_rd[k], t_wr[k] = t1 - t0, t2 - t1
            espera = objetivo - time.perf_counter()
            if espera > 0:
                time.sleep(espera)
        tao.write(0.0)
        tao.stop()
        tai.stop()

    tot = (t_rd + t_wr) * 1e3
    print(f"\n  {'':14s} {'min':>9s} {'mediana':>9s} {'p95':>9s} {'max':>9s}")
    for nom, v in (("lectura AI", t_rd * 1e3), ("escritura AO", t_wr * 1e3),
                   ("TOTAL", tot)):
        print(f"  {nom:14s} {v.min():9.3f} {np.median(v):9.3f} "
              f"{np.percentile(v,95):9.3f} {v.max():9.3f}   ms")
    print(f"  desviacion del total: {tot.std():.3f} ms")

    Ts_min = np.percentile(tot, 99) * 2
    print(f"\n  -> Ts de control alcanzable: >= {Ts_min:.1f} ms "
          f"({1000/Ts_min:.0f} Hz), dejando la mitad del periodo para el calculo.")
    print("     Es tambien el RETARDO PURO que hay que meter en el modelo NARX:")
    print("     un retardo de transporte no modelado se le aparece a la red como")
    print("     dinamica falsa. Anotar en CLAUDE.md §6.1.")


# ============================================================================
# 5. DIGITALES: ENCLAVAMIENTOS Y ARRANQUE DE LA HPU
# ============================================================================
def lee_di(mods: dict) -> list:
    """Lee las 8 entradas digitales del NI 9421. SOLO LECTURA."""
    with nidaqmx.Task() as t:
        t.di_channels.add_di_chan(f"{mods['di']}/port0/line0:7")
        v = t.read()
    bits = [(v >> i) & 1 for i in range(8)] if isinstance(v, int) else [int(b) for b in v]
    print("  Entradas digitales (NI 9421):")
    for i, b in enumerate(bits):
        marca = "" if b == DI_ESPERADO_REPOSO.get(i) else "   <-- distinto del reposo"
        print(f"    line{i} = {b}{marca}")
    return bits


class PuertoDO:
    """Gestor del puerto digital completo del NI 9472.

    Se escribe el PUERTO ENTERO, nunca una linea suelta, para que el permisivo
    (line5) quede afirmado de forma explicita y no dependa de lo que dejara la
    aplicacion anterior al liberar el modulo (ver la nota junto a LINEA_HPU).

    Al salir del bloque `with` deja SIEMPRE el patron seguro: UPH parada y
    permisivo alto. Eso incluye el caso de que salte una excepcion.
    """

    def __init__(self, mods: dict):
        self.mods = mods
        self.t = None

    def __enter__(self):
        self.t = nidaqmx.Task()
        self.t.do_channels.add_do_chan(
            f"{self.mods['do']}/port0/line0:7",
            line_grouping=LineGrouping.CHAN_PER_LINE)
        self.t.write(DO_SEGURO, auto_start=True)
        print(f"  [DO] puerto tomado, patron seguro: "
              f"{''.join('T' if b else 'F' for b in DO_SEGURO)}"
              f"  (permisivo line{LINEA_PERMISIVO} alto, UPH parada)")
        return self

    def hpu(self, encender: bool) -> None:
        self.t.write(DO_MARCHA if encender else DO_SEGURO)
        print(f"  [DO] {''.join('T' if b else 'F' for b in (DO_MARCHA if encender else DO_SEGURO))}"
              f"  -> UPH {'EN MARCHA' if encender else 'PARADA'}")

    def __exit__(self, *exc):
        try:
            self.t.write(DO_SEGURO)
            print(f"  [DO] patron seguro restaurado (UPH parada, permisivo alto)")
        finally:
            self.t.close()
        return False


def set_hpu(mods: dict, encender: bool, espera: float = 3.0) -> None:
    """Arranca o para la UPH, dejando constancia del efecto en las DI.

    OJO: al terminar este comando se libera el puerto DO, y si el modulo no
    retiene el estado, la UPH se parara sola. Para una sesion de trabajo hay que
    usar `--con-hpu`, que mantiene el puerto tomado durante toda la maniobra.
    """
    print(f"\n{'ARRANCANDO' if encender else 'PARANDO'} la UPH")

    # PRIMERO el AO a 0 V, SIEMPRE, antes de energizar la bomba.
    # Aprendido a base de moverlo (2026-08-12): el NI 9263 CONSERVA la ultima
    # tension escrita, y la habia dejado un panel de prueba de NI MAX. Al
    # arrancar la UPH sin poner el AO a cero, la servovalvula ya tenia comando y
    # el vastago se desplazo 15 mm solo. Presurizar es dar energia a lo que ya
    # este pedido: el orden correcto es anular el mando y despues arrancar.
    pon_cero(mods)

    print("  DI antes:")
    antes = lee_di(mods)
    with PuertoDO(mods) as do:
        # Registrar la posicion durante toda la espera: con la bomba en marcha y
        # el comando a 0 V, lo que se mide es la DERIVA DE NULL del carrete, una
        # de las cifras que hay que anotar (CLAUDE.md §5.1). En 2017 era ~0.03 mm/s.
        ts, xs = [], []
        with nidaqmx.Task() as tv:
            tv.ai_channels.add_ai_voltage_chan(
                _ai_chan(mods, "ai0"), min_val=-10.0, max_val=10.0,
                terminal_config=TerminalConfiguration.DIFF)
            tv.timing.cfg_samp_clk_timing(200.0, sample_mode=AcquisitionType.CONTINUOUS)
            tv.start()
            do.hpu(encender)
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < espera:
                blq = np.array(tv.read(number_of_samples_per_channel=100))
                ts.append(time.perf_counter() - t0)
                xs.append(float(escala(blq, CANALES_AI["ai0"]).mean()))
            tv.stop()
        if len(ts) >= 3:
            deriva = float(np.polyfit(ts, xs, 1)[0])
            print(f"  posicion durante la espera: {xs[0]:.2f} -> {xs[-1]:.2f} mm "
                  f"en {ts[-1]:.1f} s")
            print(f"  DERIVA a comando 0 V: {deriva:+.4f} mm/s "
                  f"({deriva*60:+.3f} mm/min)")
            if encender and abs(deriva) > 0.01:
                print("    -> el carrete NO esta en null: es la no linealidad que")
                print("       justifica el proyecto. Anotar en CLAUDE.md §5.")

        print("  DI despues:")
        despues = lee_di(mods)
        cambios = [i for i in range(8) if antes[i] != despues[i]]
        if cambios:
            print(f"  -> DI que han cambiado: {cambios}"
                  "   (candidata a 'motor encendido')")
        else:
            print("  -> NINGUNA DI ha cambiado. O la UPH no ha arrancado, o el")
            print("     estado del motor no esta cableado a este modulo.")
        with nidaqmx.Task() as t:
            _abre_ai(t, mods)
            t.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.FINITE,
                                         samps_per_chan=1000)
            d = np.array(t.read(number_of_samples_per_channel=1000))
        print("  Sensores:")
        for i, c in enumerate(CANALES_AI.values()):
            print(f"    {c['nombre']:10s} = {escala(d[i], c).mean():9.3f} "
                  f"± {escala(d[i], c).std():.3f} {c['unidad']}")
        if encender:
            print("\n  Comparar 'presion_A' contra el MANOMETRO de la camara A:")
            print("  es lo que confirma el span 2-10 V deducido (CLAUDE.md §2.4).")


# ============================================================================
# 6. CALIBRACION DE LOS TRANSDUCTORES DE PRESION
# ============================================================================
# PUNTO CERO, medido el 2026-08-12 con la UPH apagada y los DOS MANOMETROS a 0
# (confirmado por el laboratorio). Es la mitad de una calibracion de dos puntos:
V0_PRESION = {"presion_A": 1.9697, "presion_B": 1.9801}   # [V] a 0 bar


def calibra_presion(mods: dict, man_A: float | None, man_B: float | None,
                    secs: float = 90.0, previa: float = 15.0,
                    armar: bool = False) -> None:
    """Calibra los dos canales de presion por DOS PUNTOS.

    Por que hace falta: la escala supuesta no cierra el balance de fuerzas — con
    ella salian 141 kN de desequilibrio con el vastago moviendose a velocidad
    constante (CLAUDE.md §5.3c). Y una sola lectura no basta para despejar
    offset y span a la vez.

    El punto cero ya se tiene (V0_PRESION, con los manometros a 0 y la bomba
    parada). Esta rutina toma el SEGUNDO punto: mantiene la UPH en marcha y el
    AO a 0 V mientras registra los canales, y el operador lee los manometros
    durante ese rato. Con los dos puntos la recta queda determinada:

        P(V) = (V - V0) * P_man / (V1 - V0)

    Uso:
        python tools/daq.py --calibra-presion --man-a 30 --man-b 50 --armar
    """
    print("=" * 74)
    print("CALIBRACION DE PRESION POR DOS PUNTOS")
    print("=" * 74)
    print(f"  punto 0 (ya medido): {V0_PRESION} V con ambos manometros a 0 bar")
    print(f"  ventana de lectura : {previa:.0f} s para acercarse + "
          f"{secs:.0f} s de registro")
    if man_A is None or man_B is None:
        print("  lecturas de manometro: se daran DESPUES (no hacen falta ahora)")
    else:
        print(f"  punto 1            : manometro A = {man_A} bar, B = {man_B} bar")
    if not armar:
        print("\n  MODO ENSAYO EN SECO (falta --armar).")
        return

    # El comando de MANTENIMIENTO no es 0 V. Con el actuador en vertical y el
    # peso colgando, a 0 V el vastago desciende a +0.14 mm/s: en los ~4 minutos
    # que dura esta medida serian mas de 30 mm de deriva, y podria salirse de la
    # ventana. Se sostiene en U_NULL, el comando de velocidad nula medido
    # (CLAUDE.md §5.3), y ademas se vigila la posicion.
    pon_cero(mods)
    with PuertoDO(mods) as do, nidaqmx.Task() as tao:
        _abre_ao(tao, mods)
        tao.write(U_NULL, auto_start=True)
        do.hpu(True)
        print(f"\n  UPH en marcha, AO a {U_NULL:+.3f} V (velocidad nula).")
        print("  La planta se queda QUIETA todo el rato; se vigila la posicion.")
        with nidaqmx.Task() as t:
            for ch in ("ai2", "ai3", "ai0"):
                t.ai_channels.add_ai_voltage_chan(
                    _ai_chan(mods, ch), min_val=-10.0, max_val=10.0,
                    terminal_config=TerminalConfiguration.DIFF)
            t.timing.cfg_samp_clk_timing(200.0, sample_mode=AcquisitionType.CONTINUOUS)
            t.start()

            def _pos(d):
                return float(escala(d[2], CANALES_AI["ai0"]).mean())

            # --- margen para llegar hasta los manometros ------------------
            print(f"\n  Estabilizando y dando tiempo para acercarse "
                  f"({previa:.0f} s)...")
            t0 = time.perf_counter()
            while time.perf_counter() - t0 < previa:
                d = np.array(t.read(number_of_samples_per_channel=100))
                r = previa - (time.perf_counter() - t0)
                x = _pos(d)
                if not (X_MIN_SEG <= x <= X_MAX_SEG):
                    tao.write(0.0)
                    raise SystemExit(f"\nABORTA: posicion {x:.1f} mm fuera de "
                                     f"[{X_MIN_SEG}, {X_MAX_SEG}] mm")
                print(f"\r    empieza el registro en {r:4.0f} s   x={x:6.2f} mm",
                      end="", flush=True)

            # --- ventana de registro --------------------------------------
            print(f"\n\n  {'='*60}")
            print(f"  >>> LEE LOS DOS MANOMETROS AHORA. Tienes {secs:.0f} segundos. <<<")
            print(f"  {'='*60}\n")
            vA, vB, xs = [], [], []
            t0 = time.perf_counter()
            ultimo = -1.0
            while (el := time.perf_counter() - t0) < secs:
                d = np.array(t.read(number_of_samples_per_channel=100))
                vA.append(float(d[0].mean())); vB.append(float(d[1].mean()))
                xs.append(_pos(d))
                if not (X_MIN_SEG <= xs[-1] <= X_MAX_SEG):
                    tao.write(0.0)
                    raise SystemExit(f"\nABORTA: posicion {xs[-1]:.1f} mm fuera de "
                                     f"[{X_MIN_SEG}, {X_MAX_SEG}] mm")
                if el - ultimo >= 5.0:          # una linea cada 5 s, sin \r
                    ultimo = el
                    print(f"    quedan {secs-el:4.0f} s   "
                          f"V_A={vA[-1]:7.4f} V   V_B={vB[-1]:7.4f} V   "
                          f"x={xs[-1]:6.2f} mm")
            t.stop()
        tao.write(0.0)
        do.hpu(False)

    if xs:
        deriva = (xs[-1] - xs[0]) / secs
        print(f"\n  POSICION durante la ventana: {xs[0]:.2f} -> {xs[-1]:.2f} mm "
              f"({deriva*60:+.3f} mm/min)")
        print(f"  (sostenida en {U_NULL:+.3f} V; si la deriva residual es")
        print("   pequenia, confirma que ese es el comando de velocidad nula)")

    V1A, V1B = float(np.mean(vA)), float(np.mean(vB))
    sA, sB = float(np.std(vA)), float(np.std(vB))
    print(f"\n  MEDIA DE LA VENTANA ({len(vA)} bloques, {secs:.0f} s):")
    print(f"    V_A = {V1A:.4f} ± {sA:.4f} V")
    print(f"    V_B = {V1B:.4f} ± {sB:.4f} V")
    if max(sA, sB) > 0.02:
        print("    !! la presion NO estaba estable: la recta saldra sucia.")
    else:
        print("    [ok] estable durante toda la ventana")

    if man_A is None or man_B is None:
        print("\n  Faltan las lecturas de manometro. Cuando las tengas:")
        print(f"    P = (V - V0) * P_man / (V1 - V0)")
        print(f"    canal A:  V0 = {V0_PRESION['presion_A']:.4f} V, "
              f"V1 = {V1A:.4f} V   ->  dV = {V1A-V0_PRESION['presion_A']:+.4f} V")
        print(f"    canal B:  V0 = {V0_PRESION['presion_B']:.4f} V, "
              f"V1 = {V1B:.4f} V   ->  dV = {V1B-V0_PRESION['presion_B']:+.4f} V")
        print("  Basta con decir los dos valores en bar y se resuelve la recta.")
        return

    print("\n  RECTA RESULTANTE (dos puntos):")
    for nom, V0, V1, Pman in (("presion_A", V0_PRESION["presion_A"], V1A, man_A),
                              ("presion_B", V0_PRESION["presion_B"], V1B, man_B)):
        dv = V1 - V0
        if abs(dv) < 1e-3:
            print(f"    {nom}: el canal no ha cambiado ({dv:+.4f} V). Sin resolver.")
            continue
        bar_por_V = Pman / dv
        fondo = bar_por_V * (10.0 - V0)     # bar a 10 V
        print(f"    {nom}: V0={V0:.4f} V  V1={V1:.4f} V  ->  "
              f"{bar_por_V:8.3f} bar/V   (fondo de escala a 10 V: {fondo:.1f} bar)")
        print(f"        para CANALES_AI: v_lo={V0:.4f}, v_hi=10.0, "
              f"e_lo=0.0, e_hi={fondo:.1f}")

    print("\n  Comprobar despues: con la nueva escala, ¿cierra el balance")
    print("  F = P_A*A_A - P_B*A_B contra la celda? (CLAUDE.md §5.3c)")


# ============================================================================
# 7. MANIOBRAS QUE MUEVEN EL ACTUADOR
# ============================================================================
def _lee_estado(tai) -> dict:
    """Lee un bloque corto y devuelve las medias en unidades de ingenieria."""
    d = np.array(tai.read(number_of_samples_per_channel=50))
    return {c["nombre"]: float(escala(d[i], c).mean())
            for i, c in enumerate(CANALES_AI.values())}


def _comprueba_limites(e: dict, x_lo: float = X_MIN_SEG,
                       x_hi: float = X_MAX_SEG) -> str | None:
    if not (x_lo <= e["posicion"] <= x_hi):
        return f"posicion {e['posicion']:.1f} mm fuera de [{x_lo}, {x_hi}]"
    if abs(e["fuerza"]) > F_MAX_SEG:
        return f"fuerza {e['fuerza']:.1f} kN > {F_MAX_SEG}"
    if e["presion_A"] > PA_MAX_SEG:
        return f"presion A {e['presion_A']:.1f} bar > {PA_MAX_SEG}"
    if e["presion_B"] > PB_MAX_SEG:
        return f"presion B {e['presion_B']:.1f} bar > {PB_MAX_SEG}"
    return None


def _descubre_sentido(tao, tai, u_tanteo: float = 1.0, t_tanteo: float = 3.0,
                      dx_min: float = 0.15) -> float:
    """Determina que signo de comando hace CRECER la posicion medida.

    No se puede dar por sabido: depende del cableado del amplificador, de como
    esten conectados los puertos A y B al manifold y de la polaridad con que se
    escale el sensor. Equivocarse de signo en un lazo P sobre una planta
    integradora no da un error: da una carrera contra el tope.

    Devuelve +1.0 si un comando positivo aumenta la posicion, -1.0 si la baja.
    """
    e0 = _lee_estado(tai)
    tao.write(u_tanteo)
    time.sleep(t_tanteo)
    e1 = _lee_estado(tai)
    tao.write(0.0)
    time.sleep(0.5)
    dx = e1["posicion"] - e0["posicion"]
    print(f"  tanteo {u_tanteo:+.2f} V durante {t_tanteo:.1f} s -> dx = {dx:+.3f} mm")
    if abs(dx) < dx_min:
        raise SystemExit(
            f"ABORTA: el vastago apenas se mueve ({dx:+.3f} mm en {t_tanteo:.1f} s "
            f"con {u_tanteo:+.2f} V). Comprobar: ¿UPH encendida? ¿amplificador "
            "alimentado? ¿presion de suministro? ¿seta liberada?")
    signo = 1.0 if dx > 0 else -1.0
    print(f"  -> un comando POSITIVO {'AUMENTA' if signo > 0 else 'DISMINUYE'}"
          " la posicion medida")
    return signo


# ZONA MUERTA del carrete, estimada con dos puntos medidos el 2026-08-12:
#     u = 0.30 V -> v = 0.0427 mm/s        u = 1.00 V -> v = 0.5450 mm/s
#     recta:  v = 0.718 * (u - 0.241)
# Por debajo de ~0.24 V el comando NO mueve el actuador: es el solape del centro
# cerrado de la servovalvula. Explica que el primer intento de recolocacion se
# quedara 180 s clavado a 0.55 mm del destino: con Kp=0.08 el lazo pedia 0.044 V,
# muy por debajo del umbral. Un lazo P puro NO PUEDE cerrar el ultimo tramo de
# posicion en esta planta.


def _recoloca(tao, tai, destino: float, signo: float, u_max_jog: float = 1.0,
              tol: float = 1.0, t_max: float = 60.0, verboso: bool = True) -> bool:
    """Lazo P lento que lleva el vastago a `destino` [mm], sobre tareas ya
    abiertas. Devuelve True si llego.

    Lleva COMPENSACION DEL NULL, que es un feedforward en toda regla: el comando
    de velocidad nula NO es 0 V sino U_NULL = -0.370 V (el actuador esta
    en vertical y a 0 V desciende solo). Sin ese termino, el lazo P pide
    tensiones cercanas a 0 que en realidad ordenan BAJAR a 0.14 mm/s, y el
    vastago se aleja del destino mientras el lazo cree estar corrigiendo.
    Se comprobo: tres barridos abortaron por esto.

        u = U_NULL + Kp * error * signo

    Es, en pequenio, la misma idea que el feedforward no lineal que tendra que
    aprender la red.

    La tolerancia por defecto es 1.0 mm y no menos: afinar mas en una planta
    integradora con deriva exige accion integral, no un lazo P.
    """
    Kp = 0.08                            # V por mm, suave a proposito
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < t_max:
        e = _lee_estado(tai)
        # Durante la recolocacion mandan los limites MECANICOS, no la ventana de
        # trabajo: el objetivo del jog es precisamente volver a entrar en ella.
        fallo = _comprueba_limites(e, X_MIN_JOG, X_MAX_JOG)
        if fallo:
            tao.write(0.0)
            print(f"\n  !! ABORTA recolocacion: {fallo}")
            return False
        err = destino - e["posicion"]
        if abs(err) <= tol:
            tao.write(0.0)
            if verboso:
                print(f"\r  [ok] en {e['posicion']:.2f} mm (error {err:+.2f} mm)"
                      + " " * 20)
            return True
        u = U_NULL + Kp * err * signo       # feedforward del null + P
        u = float(np.clip(u, -u_max_jog, u_max_jog))
        tao.write(u)
        if verboso:
            print(f"\r    recolocando: x={e['posicion']:7.2f} mm  err={err:+7.2f} mm"
                  f"  u={u:+5.2f} V", end="", flush=True)
    tao.write(0.0)
    print(f"\n  !! recolocacion sin converger en {t_max} s")
    return False


def caracteriza(mods: dict, u_lista=None, t_paso: float = 2.5,
                x_obj: float = 75.0, armar: bool = False) -> None:
    """Mide la curva comando -> velocidad en AMBOS sentidos.

    Es LA medida de la Fase 0: de aqui salen a la vez
      * el SENTIDO (que signo de comando hace crecer la posicion medida),
      * la GANANCIA real en cada sentido -> zanja la discrepancia de ~6x
        entre el modelo fisico y los datos de 2017 (CLAUDE.md §8),
      * la ASIMETRIA medida, que se contrasta con el 0.772 que predice el
        modelo de dos camaras (§5.4),
      * la DERIVA DE NULL (el paso con u = 0).

    Procedimiento: primero se descubre el SENTIDO con un tanteo pequenio y se
    recoloca el vastago en `x_obj`; luego, por cada amplitud, se aplica el
    comando `t_paso` segundos vigilando limites cada 50 ms, se vuelve a 0 V y se
    ajusta la velocidad por minimos cuadrados sobre la posicion registrada. Si
    el barrido deriva mas de 15 mm respecto a `x_obj`, se recoloca: cada tramo
    consume recorrido y un par +/- no se cancela por la asimetria (§5.4).
    """
    if u_lista is None:
        # De menor a mayor: si algo va mal, se descubre con el comando mas
        # pequenio, no con el mas grande.
        u_lista = [0.0, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 6.0, 8.0, 10.0]

    print("=" * 74)
    print("CARACTERIZACION comando -> velocidad")
    print("=" * 74)
    print(f"  amplitudes : {u_lista} V (y sus negativas)")
    print(f"  t por paso : {t_paso} s     recolocacion a {x_obj} mm entre pasos")
    print(f"  limites    : x en [{X_MIN_SEG}, {X_MAX_SEG}] mm · F < {F_MAX_SEG} kN · "
          f"P_A < {PA_MAX_SEG} bar · P_B < {PB_MAX_SEG} bar")
    if not armar:
        print("\n  MODO ENSAYO EN SECO (falta --armar): no se escribe en el AO.")
        return

    fs = 100.0
    res = []
    with nidaqmx.Task() as tao, nidaqmx.Task() as tai:
        _abre_ao(tao, mods)
        _abre_ai(tai, mods)
        tai.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS)
        tao.write(0.0, auto_start=True)
        tai.start()

        try:
            e = _lee_estado(tai)
            print(f"\n  posicion inicial: {e['posicion']:.2f} mm")
            if (fallo := _comprueba_limites(e)):
                raise SystemExit(f"ABORTA antes de empezar: {fallo}")
            signo = _descubre_sentido(tao, tai)
            # Solo se recoloca si hace falta DE VERDAD. Exigir llegar a x_obj
            # exacto bloqueo TRES intentos seguidos de barrido: en sentido
            # negativo la planta apenas se mueve (§5.3b4) y el lazo P se pasaba
            # los 180 s sin cerrar unos pocos milimetros. Para barrer basta con
            # estar holgadamente dentro de la ventana de trabajo.
            BANDA = 25.0
            if abs(e["posicion"] - x_obj) > BANDA:
                print(f"\n  Recolocando a {x_obj:.1f} mm antes del barrido...")
                if not _recoloca(tao, tai, x_obj, signo, u_max_jog=2.0, t_max=180.0):
                    raise SystemExit("ABORTA: no se pudo recolocar el vastago.")
            else:
                print(f"  a {abs(e['posicion']-x_obj):.1f} mm del centro "
                      f"(banda +-{BANDA:.0f} mm): se barre desde aqui")
            print()

            for u0 in u_lista:
                for sg in ((1.0,) if u0 == 0.0 else (1.0, -1.0)):
                    u = sg * u0
                    e = _lee_estado(tai)
                    # Recolocar si el barrido ha ido derivando: cada tramo
                    # consume recorrido (planta integradora) y la asimetria
                    # hace que un par +/- NO se cancele del todo.
                    if abs(e["posicion"] - x_obj) > 30.0:
                        _recoloca(tao, tai, x_obj, signo, u_max_jog=2.0,
                                  t_max=90.0, verboso=False)
                        e = _lee_estado(tai)
                    # Un limite rozado ANTES de un paso suele ser transitorio:
                    # tras parar el paso anterior las presiones tardan en
                    # asentarse. Antes esto abortaba el barrido ENTERO de golpe,
                    # tirando varios minutos de maquina por una condicion que se
                    # habria despejado sola. Ahora se reintenta antes de rendirse,
                    # y al rendirse se dice el estado COMPLETO, no solo la causa.
                    for intento in range(4):
                        fallo = _comprueba_limites(e)
                        if not fallo:
                            break
                        print(f"    (limite rozado antes de u={u:+.2f} V: {fallo}"
                              f" — esperando, intento {intento+1}/4)")
                        tao.write(0.0)
                        time.sleep(2.0)
                        e = _lee_estado(tai)
                    if fallo:
                        print(f"  !! ABORTA antes de u={u:+.2f} V: {fallo}")
                        print("     estado completo en el momento del corte:")
                        for k, val in e.items():
                            print(f"       {k:10s} = {val:9.3f}")
                        print(f"     limites: x en [{X_MIN_SEG}, {X_MAX_SEG}] mm · "
                              f"F < {F_MAX_SEG} kN · P_A < {PA_MAX_SEG} · "
                              f"P_B < {PB_MAX_SEG} bar")
                        raise KeyboardInterrupt

                    tao.write(u)
                    t0 = time.perf_counter()
                    ts, xs = [], []
                    while time.perf_counter() - t0 < t_paso:
                        e = _lee_estado(tai)
                        ts.append(time.perf_counter() - t0)
                        xs.append(e["posicion"])
                        fallo = _comprueba_limites(e)
                        if fallo:
                            tao.write(0.0)
                            print(f"  !! corte en u={u:+.2f} V: {fallo}")
                            break
                    tao.write(0.0)
                    time.sleep(0.5)

                    # Se descartan los primeros 0.3 s: son el transitorio del
                    # carrete, no regimen. `satura` ya lo hacia y aqui no, asi
                    # que las dos pruebas median cosas distintas — y pesa mas
                    # cuanto mayor es el comando, justo donde interesa.
                    ts_r = [a for a in ts if a >= 0.3]
                    xs_r = xs[len(ts) - len(ts_r):]
                    if len(ts_r) >= 3:
                        ts, xs = ts_r, xs_r
                    if len(ts) >= 3:
                        v = float(np.polyfit(ts, xs, 1)[0])   # mm/s
                        res.append((u, v))
                        print(f"    u={u:+6.2f} V -> v={v:+9.4f} mm/s   "
                              f"(x {xs[0]:6.1f} -> {xs[-1]:6.1f} mm, "
                              f"P_A={e['presion_A']:6.1f} P_B={e['presion_B']:6.1f} bar)")
        except KeyboardInterrupt:
            print("  interrumpido")
        finally:
            tao.write(0.0)
            tai.stop()

    if not res:
        return
    # AJUSTE SOLO EN LA ZONA LINEAL. Por encima de U_LINEAL la relacion deja de
    # ser recta: el caudal de la bomba satura al extender. Y con escalones de
    # pocos segundos ni siquiera se ve la saturacion, porque el acumulador
    # aporta caudal mientras dura el paso — MEDIDO: a 10 V un escalon de 2.5 s
    # da 3.92 mm/s cuando el regimen sostenido es 2.634, un factor 1.5.
    #
    # Meter esos puntos en el ajuste ESTROPEA la ganancia: con el barrido hasta
    # 10 V sale K+ = 0.389 frente a 0.438 usando solo hasta 2 V (13 % baja).
    # Por eso se ajusta la zona lineal y los altos se reportan aparte.
    U_LINEAL = 2.5
    print("\n  RESUMEN")
    cero = [v for u, v in res if u == 0]
    if cero:
        print(f"    deriva de null (u=0)      : {cero[0]:+.4f} mm/s")

    lineal = {"positivo": [(u, v) for u, v in res if 0 < u <= U_LINEAL],
              "negativo": [(u, v) for u, v in res if -U_LINEAL <= u < 0]}
    aj = {}
    for nom, lst in lineal.items():
        if len(lst) >= 2:
            K, b = np.polyfit([u for u, _ in lst], [v for _, v in lst], 1)
            aj[nom] = (K, b)
            print(f"    K comando {nom:12s}: {K:+.4f} mm/s por V   "
                  f"(v = {K:.4f}u {b:+.4f}, n={len(lst)})")
    if len(aj) == 2:
        Kp = aj["positivo"][0]
        Kn, bn = aj["negativo"]
        print(f"    asimetria |K-/K+|         : {abs(Kn/Kp):.3f}"
              "   (el modelo de 2 camaras predice 0.772)")
        print(f"    velocidad nula en u       : {-bn/Kn:+.3f} V")
        print(f"\n  -> ajustado SOLO con |u| <= {U_LINEAL} V. Si se quiere que todo")
        print("     el proyecto lo use, anotarlo en planta.py (K_POS / K_NEG).")

    altos = [(u, v) for u, v in res if abs(u) > U_LINEAL]
    if altos:
        print(f"\n  Puntos por encima de {U_LINEAL} V, FUERA del ajuste:")
        for u, v in sorted(altos):
            aviso = ""
            if u > 0 and v > V_MAX_EXT * 1.05:
                aviso = f"  <- por encima del techo de caudal ({V_MAX_EXT:.2f})"
            print(f"    u={u:+6.2f} V -> v={v:+8.4f} mm/s{aviso}")
        print("    Un escalon corto NO mide el regimen a comando alto: el")
        print("    acumulador aporta caudal durante los primeros segundos.")
        print("    Para el techo de caudal usar --satura, que recorre la carrera.")


def satura(mods: dict, u_test: float = 10.0, margen: float = 6.0,
           t_max: float = 120.0, fs: float = 200.0, signo_dado: float = 0.0,
           calienta: float = 180.0, armar: bool = False) -> None:
    """Mide el TOPE DE CAUDAL de la bomba recorriendo la carrera a comando alto.

    QUE PREGUNTA CONTESTA
    ---------------------
    La placa dice 1.7 L/min, que daria un tope de velocidad de 1.41 mm/s
    extendiendo. Pero en la captura se midieron 2.80 mm/s sostenidos 2.8 s
    (CLAUDE.md §5.9). La diferencia son 78 cm3 de aceite, que un acumulador de
    vejiga entrega sin dificultad — asi que ese tramo NO distingue entre
    "la bomba da mas de 1.7" y "el acumulador estaba aportando".

    Para distinguirlo hay que sostener el comando lo suficiente para VACIAR el
    acumulador. Si lo hay, la velocidad arranca alta y DECAE a una meseta; esa
    meseta es Q_bomba/A.

    EL DISCRIMINADOR, y es lo que hace la prueba concluyente
    --------------------------------------------------------
    Se recorre la carrera en los DOS sentidos, y se mira la RELACION de las
    mesetas:

      * limitada por CAUDAL   -> v = Q/A en cada sentido, luego la relacion
                                 es exactamente A_A/A_B = 1.641
      * limitada por ORIFICIO -> la relacion es la que ya se midio: 0.847

    Son numeros que no se parecen en nada, asi que no hay ambiguedad posible.
    Y de paso: 1.641 y 0.847 estan a lados opuestos de 1, o sea que hasta el
    SENTIDO de la asimetria cambia.
    """
    print("=" * 74)
    print(f"PRUEBA DE SATURACION DE CAUDAL  (|u| = {u_test:.1f} V sostenido)")
    print("=" * 74)
    print(f"  recorrido: entre {X_MIN_SEG+margen:.0f} y {X_MAX_SEG-margen:.0f} mm, "
          f"en los dos sentidos")
    print(f"  si la bomba da 1.7 L/min, las mesetas serian "
          f"{1.7/60000/A_A*1e3:.3f} y {1.7/60000/A_B*1e3:.3f} mm/s")
    print(f"  relacion esperada si SATURA POR CAUDAL : {A_A/A_B:.3f}  (= A_A/A_B)")
    print(f"  relacion medida en regimen no saturado : 0.847")
    if not armar:
        print("\n  MODO ENSAYO EN SECO (falta --armar): no se escribe en el AO.")
        return

    x_lo, x_hi = X_MIN_SEG + margen, X_MAX_SEG - margen
    reg = {}
    with nidaqmx.Task() as tao, nidaqmx.Task() as tai:
        _abre_ao(tao, mods)
        _abre_ai(tai, mods)
        tai.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS)
        tao.write(0.0, auto_start=True)
        tai.start()
        try:
            if signo_dado:
                # El tanteo de sentido es una medida CORTA, y las medidas cortas
                # tras arrancar la bomba estan contaminadas por un transitorio
                # que decae despacio (0.55 -> 0.13 mm/s en 14 s, §5.9). Con el
                # sentido ya conocido de otras sesiones se salta el tanteo: esta
                # prueba no lo necesita, porque sus tramos duran 40-70 s.
                signo = float(np.sign(signo_dado))
                print(f"  sentido IMPUESTO: comando positivo "
                      f"{'aumenta' if signo > 0 else 'disminuye'} la posicion")
            else:
                signo = _descubre_sentido(tao, tai)
            # ---- CALENTAMIENTO, y de paso se mide el transitorio ---------
            # En la primera version se esperaba 25 s y NINGUNO de los dos
            # recorridos llego a regimen: las presiones seguian moviendose al
            # final de los dos. Aqui se sostiene el comando de velocidad nula
            # varios minutos y se registra la deriva, que ademas contesta
            # cuanto dura de verdad el transitorio tras arrancar la bomba.
            if calienta > 0:
                print(f"\n  --- calentamiento: {calienta:.0f} s sosteniendo "
                      f"{U_NULL:+.3f} V ---")
                centro = 0.5 * (x_lo + x_hi)
                tao.write(U_NULL)
                tw, xw, pw = [], [], []
                t0 = time.perf_counter()
                ult = -30.0
                while (el := time.perf_counter() - t0) < calienta:
                    d = np.array(tai.read(number_of_samples_per_channel=int(fs / 5)))
                    e = {c["nombre"]: float(escala(d[i], c).mean())
                         for i, c in enumerate(CANALES_AI.values())}
                    tw.append(el); xw.append(e["posicion"]); pw.append(e["presion_A"])
                    if abs(e["posicion"] - centro) > 35.0:
                        tao.write(0.0)
                        _recoloca(tao, tai, centro, signo, u_max_jog=2.0,
                                  t_max=120.0, verboso=False)
                        tao.write(U_NULL)
                    if el - ult >= 30.0:
                        ult = el
                        print(f"    t={el:5.0f}s  x={e['posicion']:7.2f} mm  "
                              f"P_A={e['presion_A']:5.1f} bar")
                tao.write(0.0); time.sleep(1.0)
                tw = np.array(tw); xw = np.array(xw); pw = np.array(pw)
                w = max(5, int(round(10.0 / np.median(np.diff(tw)))))
                if len(tw) - w > 10:
                    vw = np.array([np.polyfit(tw[i:i+w], xw[i:i+w], 1)[0]
                                   for i in range(len(tw) - w)])
                    print(f"    deriva sostenida en {U_NULL:+.3f} V: "
                          f"{vw[0]:+.4f} -> {vw[-1]:+.4f} mm/s")
                    print(f"    presion A: {pw[0]:.1f} -> {pw[-1]:.1f} bar "
                          f"({'ASENTADA' if abs(pw[-1]-pw[-len(pw)//4]) < 1.0 else 'AUN CAMBIANDO'})")
                # NO entra en `reg`: si entrara, el bucle de analisis lo
                # trataria como un recorrido mas y la comparacion de
                # mesetas entre sentidos dejaria de imprimirse.
                calent = (tw, xw, pw)

            for nombre, destino, arranque in (("extendiendo", x_hi, x_lo),
                                              ("retrayendo", x_lo, x_hi)):
                print(f"\n  --- {nombre}: colocando en {arranque:.0f} mm ---")
                if not _recoloca(tao, tai, arranque, signo, u_max_jog=2.0,
                                 t_max=200.0, verboso=False):
                    print("  !! no se pudo colocar; se salta este sentido")
                    continue
                u = u_test * signo * (1 if destino > arranque else -1)
                print(f"  aplicando u = {u:+.2f} V hasta {destino:.0f} mm...")
                ts, xs, pas, pbs = [], [], [], []
                tao.write(u)
                t0 = time.perf_counter()
                while (el := time.perf_counter() - t0) < t_max:
                    d = np.array(tai.read(number_of_samples_per_channel=int(fs / 10)))
                    e = {c["nombre"]: float(escala(d[i], c).mean())
                         for i, c in enumerate(CANALES_AI.values())}
                    ts.append(el); xs.append(e["posicion"])
                    pas.append(e["presion_A"]); pbs.append(e["presion_B"])
                    if (destino > arranque and e["posicion"] >= destino) or \
                       (destino < arranque and e["posicion"] <= destino):
                        break
                    if _comprueba_limites(e, X_MIN_JOG, X_MAX_JOG):
                        break
                tao.write(0.0)
                time.sleep(1.0)
                reg[nombre] = (np.array(ts), np.array(xs), np.array(pas), np.array(pbs))
                print(f"  {xs[0]:.1f} -> {xs[-1]:.1f} mm en {ts[-1]:.1f} s")
        finally:
            tao.write(0.0)
            tai.stop()

    # ---- analisis --------------------------------------------------------
    print("\n" + "=" * 74)
    print("RESULTADO")
    print("=" * 74)
    mesetas = {}
    for nombre, (ts, xs, pa, pb) in reg.items():
        if len(ts) < 20:
            print(f"  {nombre}: muy pocos datos"); continue
        # Velocidad instantanea por ajuste sobre ventana deslizante de 3 s.
        # El paso se toma como MEDIANA de los intervalos, no como ts[1]-ts[0]:
        # la primera lectura puede volver casi instantanea y ese unico intervalo
        # corto hacia w enorme, la lista de velocidades salia vacia y el informe
        # imprimia NaN sin avisar de nada.
        w = max(5, int(round(3.0 / np.median(np.diff(ts)))))
        if len(ts) - w < 10:
            print(f"  {nombre}: tramo demasiado corto para estimar velocidad")
            continue
        vt = np.array([np.polyfit(ts[i:i+w], xs[i:i+w], 1)[0]
                       for i in range(len(ts) - w)])
        tv = ts[:len(vt)]
        v_ini = np.median(vt[:max(3, len(vt)//10)])
        v_fin = np.median(vt[-max(3, len(vt)//5):])
        mesetas[nombre] = abs(v_fin)
        print(f"  {nombre:12s}: v inicial {abs(v_ini):6.3f}  ->  meseta "
              f"{abs(v_fin):6.3f} mm/s   (caida {(1-abs(v_fin/v_ini))*100:+5.1f} %)")
        A = A_A if nombre == "extendiendo" else A_B
        print(f"                caudal implicito = {abs(v_fin)*1e-3*A*60000:.2f} L/min"
              f"   P_A {pa.mean():5.1f} bar  P_B {pb.mean():5.1f} bar")
    if len(mesetas) == 2:
        rel = mesetas["retrayendo"] / mesetas["extendiendo"]
        print(f"\n  RELACION retraer/extender = {rel:.3f}")
        print(f"    si estuviera limitada por CAUDAL   : {A_A/A_B:.3f}")
        print(f"    si estuviera limitada por ORIFICIO : 0.847")
        d_q = abs(rel - A_A / A_B); d_o = abs(rel - 0.847)
        print(f"  -> compatible con limitacion por "
              f"{'CAUDAL (la bomba)' if d_q < d_o else 'ORIFICIO (la servovalvula)'}")

    os.makedirs("results", exist_ok=True)
    with open("results/saturacion.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f); w.writerow(["sentido", "t_s", "posicion_mm",
                                       "presion_A_bar", "presion_B_bar"])
        for nombre, (ts, xs, pa, pb) in reg.items():
            for i in range(len(ts)):
                w.writerow([nombre, f"{ts[i]:.4f}", f"{xs[i]:.4f}",
                            f"{pa[i]:.4f}", f"{pb[i]:.4f}"])
    print("\n[ok] results/saturacion.csv")


def jog(mods: dict, destino: float, u_max_jog: float = 2.0, tol: float = 1.0,
        t_max: float = 240.0, armar: bool = False) -> None:
    """Lleva el vastago a `destino` [mm] con un lazo P lento y acotado.

    Sirve para RECOLOCAR antes de una captura (ahora mismo el vastago esta
    practicamente retraido, fuera de la ventana de trabajo). No pretende ser un
    controlador: es una maniobra de posicionamiento con el comando muy limitado.

    Requiere conocer el SENTIDO. Se deduce sobre la marcha: aplica un comando
    pequenio, mira si la posicion se acerca o se aleja, y fija el signo.
    """
    print("=" * 74)
    print(f"RECOLOCACION a {destino:.1f} mm  (|u| <= {u_max_jog} V, tol {tol} mm)")
    print("=" * 74)
    if not (X_MIN_SEG <= destino <= X_MAX_SEG):
        raise SystemExit(f"ABORTA: destino fuera de [{X_MIN_SEG}, {X_MAX_SEG}] mm")
    if not armar:
        print("  MODO ENSAYO EN SECO (falta --armar): no se escribe en el AO.")
        return

    fs = 100.0
    signo = 0.0          # 0 = aun por determinar
    with nidaqmx.Task() as tao, nidaqmx.Task() as tai:
        _abre_ao(tao, mods)
        _abre_ai(tai, mods)
        tai.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS)
        tao.write(0.0, auto_start=True)
        tai.start()
        try:
            e0 = _lee_estado(tai)
            print(f"  posicion inicial: {e0['posicion']:.2f} mm")
            signo = _descubre_sentido(tao, tai)
            _recoloca(tao, tai, destino, signo, u_max_jog=u_max_jog,
                      tol=tol, t_max=t_max)
        finally:
            print()
            tao.write(0.0)
            tai.stop()
    print("  AO a 0 V.")


# ============================================================================
# 8. CAPTURA CON TEMPORIZACION POR HARDWARE Y SUPERVISION
# ============================================================================
def captura(mods: dict, ruta_exc: str, etiqueta: str, fs: float = 1000.0,
            buffer_s: float = 0.3, armar: bool = False) -> None:
    """Emite la secuencia de excitacion y registra los 4 AI.

    AO y AI comparten el trigger de arranque del AI, asi que comando y medidas
    quedan alineados POR HARDWARE. El AO se alimenta por trozos para poder
    abortar si la posicion se acerca a un limite.
    """
    # ---- cargar la secuencia -------------------------------------------
    t_exc, u_exc = [], []
    with open(ruta_exc, encoding="utf-8") as f:
        for fila in csv.DictReader(f):
            t_exc.append(float(fila["t_s"]))
            u_exc.append(float(fila["u_V"]))
    u_exc = np.asarray(u_exc)
    Ts_exc = t_exc[1] - t_exc[0]

    if abs(Ts_exc - 1.0 / fs) > 1e-9:
        print(f"  aviso: la secuencia esta a {1/Ts_exc:.1f} Hz y se pide {fs:.1f} Hz."
              f"  Se usara {1/Ts_exc:.1f} Hz.")
        fs = 1.0 / Ts_exc

    if np.abs(u_exc).max() > U_MAX_SEG:
        raise SystemExit(f"ABORTA: la secuencia pide {np.abs(u_exc).max():.2f} V "
                         f"(limite {U_MAX_SEG} V)")

    n_tot = len(u_exc)
    n_buf = int(buffer_s * fs)
    print("=" * 74)
    print(f"CAPTURA '{etiqueta}'")
    print("=" * 74)
    print(f"  secuencia : {ruta_exc}")
    print(f"  muestras  : {n_tot}  ({n_tot/fs:.1f} s a {fs:.0f} Hz)")
    print(f"  comando   : {u_exc.min():+.3f} .. {u_exc.max():+.3f} V")
    print(f"  buffer    : {n_buf} muestras ({buffer_s*1e3:.0f} ms de latencia de aborto)")
    print(f"  limites   : x en [{X_MIN_SEG}, {X_MAX_SEG}] mm · F < {F_MAX_SEG} kN · "
          f"P_A < {PA_MAX_SEG} bar · P_B < {PB_MAX_SEG} bar")

    # ---- comprobacion previa: SIEMPRE, aunque no se arme ---------------
    print("\n  Comprobacion previa (solo lectura):")
    with nidaqmx.Task() as t:
        _abre_ai(t, mods)
        t.timing.cfg_samp_clk_timing(1000.0, sample_mode=AcquisitionType.FINITE,
                                     samps_per_chan=500)
        d0 = np.array(t.read(number_of_samples_per_channel=500))
    est = {c["nombre"]: escala(d0[i], c).mean()
           for i, c in enumerate(CANALES_AI.values())}
    for k, v in est.items():
        print(f"    {k:10s} = {v:9.3f}")

    x0 = est["posicion"]
    if not (X_MIN_SEG < x0 < X_MAX_SEG):
        raise SystemExit(f"ABORTA: la posicion inicial {x0:.1f} mm esta fuera de "
                         f"[{X_MIN_SEG}, {X_MAX_SEG}] mm. Recolocar el vastago.")
    if est["fuerza"] > F_MAX_SEG:
        raise SystemExit(f"ABORTA: hay {est['fuerza']:.1f} kN de carga. La "
                         f"identificacion se hace SIN probeta (CLAUDE.md §6.3).")
    print(f"    [ok] posicion inicial {x0:.1f} mm, sin carga")

    if not armar:
        print("\n  MODO ENSAYO EN SECO: no se ha pasado --armar, no se escribe nada.")
        print("  Todo lo de arriba se ha comprobado. Para ejecutar de verdad,")
        print("  repetir el comando anadiendo  --armar")
        return

    # ---- tareas ---------------------------------------------------------
    print("\n  ARMADO. Emitiendo... (Ctrl-C aborta a 0 V)")
    reg = np.empty((4, n_tot), dtype=float)
    n_leidas = 0
    abortado = None

    with nidaqmx.Task() as tao, nidaqmx.Task() as tai:
        # AI: continuo, es el reloj maestro
        _abre_ai(tai, mods)
        tai.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS,
                                       samps_per_chan=n_buf * 8)

        # AO: continuo, arranca con el trigger del AI -> alineacion por hardware
        _abre_ao(tao, mods)
        tao.timing.cfg_samp_clk_timing(fs, sample_mode=AcquisitionType.CONTINUOUS,
                                       samps_per_chan=n_buf * 8)
        tao.out_stream.regen_mode = RegenerationMode.DONT_ALLOW_REGENERATION
        tao.triggers.start_trigger.cfg_dig_edge_start_trig(
            f"/{mods['chasis']}/ai/StartTrigger", Edge.RISING)

        # precarga y arranque: el AO espera al trigger del AI
        i_esc = min(2 * n_buf, n_tot)
        tao.write(u_exc[:i_esc], auto_start=False)
        tao.start()
        tai.start()

        try:
            while n_leidas < n_tot:
                blq = np.array(tai.read(number_of_samples_per_channel=n_buf))
                m = min(blq.shape[1], n_tot - n_leidas)
                reg[:, n_leidas:n_leidas + m] = blq[:, :m]
                n_leidas += m

                # --- supervision: los limites mandan sobre la secuencia ----
                x = escala(blq[0], CANALES_AI["ai0"])
                F = escala(blq[1], CANALES_AI["ai1"])
                PA = escala(blq[2], CANALES_AI["ai2"])
                PB = escala(blq[3], CANALES_AI["ai3"])
                if x.min() < X_MIN_SEG or x.max() > X_MAX_SEG:
                    abortado = f"posicion fuera de rango ({x.min():.1f}..{x.max():.1f} mm)"
                elif F.max() > F_MAX_SEG:
                    abortado = f"fuerza {F.max():.1f} kN > {F_MAX_SEG} kN"
                elif PA.max() > PA_MAX_SEG:
                    abortado = f"presion A {PA.max():.1f} bar > {PA_MAX_SEG} bar"
                elif PB.max() > PB_MAX_SEG:
                    abortado = f"presion B {PB.max():.1f} bar > {PB_MAX_SEG} bar"
                if abortado:
                    break

                # --- rellenar el AO por delante ---------------------------
                if i_esc < n_tot:
                    j = min(i_esc + n_buf, n_tot)
                    tao.write(u_exc[i_esc:j], auto_start=False)
                    i_esc = j

                print(f"\r    {n_leidas/n_tot*100:5.1f}%  x={x.mean():7.2f} mm  "
                      f"F={F.mean():6.2f} kN  P_A={PA.mean():6.1f}  P_B={PB.mean():6.1f} bar",
                      end="", flush=True)
        except KeyboardInterrupt:
            abortado = "interrumpido por el operador (Ctrl-C)"
        finally:
            print()
            try:
                tao.stop()
                tao.write(0.0, auto_start=True)   # estado seguro
            except Exception:
                pass
            tai.stop()

    if abortado:
        print(f"  !! ABORTADO: {abortado}")
        print("     El AO se ha llevado a 0 V. Los datos hasta el aborto se guardan.")

    # ---- guardar --------------------------------------------------------
    n = n_leidas
    t = np.arange(n) / fs
    ruta = f"results/captura_{etiqueta}.csv"
    os.makedirs("results", exist_ok=True)
    cols = [c["nombre"] for c in CANALES_AI.values()]
    with open(ruta, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "u_V"] + [f"{c['nombre']}_{c['unidad']}"
                                     for c in CANALES_AI.values()])
        esc = [escala(reg[i, :n], c) for i, c in enumerate(CANALES_AI.values())]
        for k in range(n):
            w.writerow([f"{t[k]:.6f}", f"{u_exc[k]:.6f}"] +
                       [f"{e[k]:.6f}" for e in esc])
    print(f"\n[ok] {ruta}   ({n} muestras, {n/fs:.1f} s)")
    print(f"     columnas: t_s, u_V, {', '.join(cols)}")
    print("\n  Comprobar antes de dar la captura por buena:")
    print("   * que el recorrido de posicion sea el previsto por gen_excitacion;")
    print("   * que no haya tramos planos sospechosos (sensor saturado o perdido);")
    print("   * la temperatura del aceite al inicio y al final (CLAUDE.md §6.4).")


# ============================================================================
def main() -> None:
    ap = argparse.ArgumentParser(
        description="Capa de E/S del cDAQ-9184. Por defecto NO escribe en el AO.")
    ap.add_argument("--diag", action="store_true", help="enumera y lee (SOLO LECTURA)")
    ap.add_argument("--sensores", action="store_true", help="lectura en vivo (SOLO LECTURA)")
    ap.add_argument("--secs", type=float, default=10.0)
    ap.add_argument("--latencia", action="store_true", help="mide el lazo de software")
    ap.add_argument("--cero", action="store_true", help="lleva el AO a 0 V")
    ap.add_argument("--captura", metavar="CSV_EXCITACION",
                    help="emite la secuencia y registra los sensores")
    ap.add_argument("--etiqueta", default="train")
    ap.add_argument("--fs", type=float, default=1000.0)
    ap.add_argument("--di", action="store_true",
                    help="lee las entradas digitales (SOLO LECTURA)")
    ap.add_argument("--hpu", choices=["on", "off"], help="arranca/para la UPH")
    ap.add_argument("--con-hpu", action="store_true",
                    help="arranca la UPH, ejecuta la maniobra y la para, todo en "
                         "el MISMO proceso (mantiene tomado el puerto DO)")
    ap.add_argument("--calibra-presion", action="store_true",
                    help="calibra los canales de presion por dos puntos")
    ap.add_argument("--man-a", type=float, help="lectura del manometro A [bar] "
                    "(opcional: se puede dar despues)")
    ap.add_argument("--man-b", type=float, help="lectura del manometro B [bar] "
                    "(opcional: se puede dar despues)")
    ap.add_argument("--previa", type=float, default=15.0,
                    help="segundos antes de empezar a registrar, para acercarse "
                         "a los manometros (defecto 15)")
    ap.add_argument("--caracteriza", action="store_true",
                    help="mide la curva comando->velocidad en ambos sentidos")
    ap.add_argument("--satura", action="store_true",
                    help="mide el tope de caudal de la bomba (recorre la carrera "
                         "con comando alto en los dos sentidos)")
    ap.add_argument("--u-test", type=float, default=10.0,
                    help="comando para la saturacion [V]. Por defecto FONDO DE "
                         "ESCALA: la prueba busca el techo de caudal, y "
                         "estrangular la valvula al 80%% trabaja en su contra "
                         "(con 8 V se exigian 3.06 L/min; con 10, unos 5.5).")
    ap.add_argument("--calienta", type=float, default=180.0,
                    help="segundos de bomba en marcha antes de medir [s]")
    ap.add_argument("--signo", type=float, default=0.0,
                    help="+1/-1 para imponer el sentido y saltarse el tanteo "
                         "(0 = descubrirlo midiendo)")
    ap.add_argument("--jog", type=float, metavar="MM",
                    help="recoloca el vastago en esa posicion [mm]")
    ap.add_argument("--armar", action="store_true",
                    help="REQUERIDO para que se escriba algo en el AO o en el DO")
    args = ap.parse_args()

    if not any([args.diag, args.sensores, args.latencia, args.cero, args.captura,
                args.di, args.hpu, args.caracteriza, args.calibra_presion,
                args.satura, args.jog is not None]):
        ap.print_help()
        return

    mods = diag() if args.diag else encuentra_modulos()
    if not mods.get("ai") or not mods.get("ao"):
        if not args.diag:
            print("ERROR: no se localizan los modulos. Ejecutar --diag primero.")
        return

    if args.di:
        lee_di(mods)

    if args.hpu:
        if not args.armar:
            print(f"\n  --hpu {args.hpu} escribiria el puerto DO completo "
                  f"({mods['do']}/port0/line0:7). Anadir --armar.")
        else:
            set_hpu(mods, args.hpu == "on", espera=args.secs)

    if args.sensores:
        lee_sensores(mods, args.secs)

    if args.calibra_presion:
        # Las lecturas de manometro son OPCIONALES: si no se dan, se registra y
        # se imprime lo necesario para resolver la recta despues. Asi no hace
        # falta saber los valores de antemano ni ir con prisa hasta el equipo.
        calibra_presion(mods, args.man_a, args.man_b, secs=args.secs,
                        previa=args.previa, armar=args.armar)

    # Maniobras que MUEVEN el actuador. Con --con-hpu se envuelven en arranque y
    # parada de la UPH dentro del MISMO proceso: es la unica forma de garantizar
    # que el permisivo (line5) sigue afirmado durante toda la maniobra, sin
    # depender de si el modulo retiene el estado al liberar la tarea.
    def _maniobras() -> None:
        if args.caracteriza:
            caracteriza(mods, armar=args.armar)
        if args.jog is not None:
            jog(mods, args.jog, armar=args.armar)
        if args.satura:
            satura(mods, u_test=args.u_test, signo_dado=args.signo,
                   calienta=args.calienta, armar=args.armar)
        if args.captura:
            captura(mods, args.captura, args.etiqueta, fs=args.fs, armar=args.armar)

    hay_maniobra = bool(args.caracteriza or args.jog is not None or args.captura
                        or args.satura)
    if hay_maniobra and args.con_hpu and args.armar:
        with PuertoDO(mods) as do:
            do.hpu(True)
            print("  esperando 25 s a que se estabilice la presion...")
            print("  (los primeros segundos tras arrancar la bomba no son fiables: §5.3b)")
            time.sleep(25.0)
            try:
                _maniobras()
            finally:
                do.hpu(False)
    elif hay_maniobra:
        _maniobras()

    if args.cero or args.latencia:
        if not args.armar:
            print("\n  Esta operacion escribe en el AO (0 V). Anadir --armar para ejecutarla.")
        else:
            if args.cero:
                pon_cero(mods)
            if args.latencia:
                mide_latencia(mods)


if __name__ == "__main__":
    main()
