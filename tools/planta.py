#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
planta.py — LOS NUMEROS DE LA PRENSA. Fuente unica para todo el proyecto.

============================ GUIA DE LECTURA ============================
Este es el fichero por el que conviene empezar a estudiar el codigo. No hace
nada: solo declara que sabemos de la maquina y de donde lo sabemos.

POR QUE EXISTE
--------------
Los mismos numeros estaban repetidos en `daq.py` y en `gen_excitacion.py`, con
nombres distintos (`U_NULL_MEDIDO` frente a `U_NULL`), y las areas del cilindro
aparecian escritas a mano en dos sitios mas mientras `planta_sim.py` las
derivaba de la geometria. Con esa disposicion, re-medir la planta obligaba a
tocar cuatro ficheros, y bastaba olvidarse de uno para que el proyecto empezara
a usar dos plantas distintas sin avisar de nada.

Aqui hay UNA definicion de cada cosa. Si se vuelve a medir, se cambia aqui.

COMO LEER CADA NUMERO
---------------------
Cada bloque dice su ORIGEN, y no todos valen lo mismo:

    MEDIDO      lo dice el equipo. Manda sobre cualquier otra fuente.
    CATALOGO    hoja de datos del fabricante. Fiable, pero de un componente
                nominal, no de este ejemplar concreto.
    MEMORIA     memoria de Fluidtek 2017. Esta DESACTUALIZADA: cada vez que se
                ha contrastado contra el equipo ha aparecido una discrepancia
                (carrera, rangos de sensor, cilindro simetrico, K_amp...).
                Usar solo para lo que no se haya podido medir.

El detalle de como se obtuvo cada medida esta en la skill `resultados-medidos`;
el resumen operativo, en CLAUDE.md §5.0.
"""

from __future__ import annotations

import math

# ============================================================================
# 1. GEOMETRIA DEL ACTUADOR
# ============================================================================
# Diametros: MEMORIA (§2.3). Carrera: dato del laboratorio — la memoria citaba
# un sensor de 400 mm y son 150 mm.
D_PISTON = 0.160        # [m] diametro del embolo
D_VASTAGO = 0.100       # [m] diametro del vastago
L_CARRERA = 0.150       # [m] carrera util

# El cilindro es ASIMETRICO y la memoria lo modela como simetrico. Su
# "A_p = 122.52 cm2" es en realidad el area de la camara ANULAR; la de fondo es
# un 64 % mayor. De esa asimetria salen tres cosas que se miden en el equipo:
# la ganancia distinta por sentido, la intensificacion de presion, y que la
# fuerza sea P_A*A_A - P_B*A_B y no A_p*(P_A - P_B).
A_A = math.pi / 4 * D_PISTON**2                      # [m2] camara SIN vastago
A_B = math.pi / 4 * (D_PISTON**2 - D_VASTAGO**2)     # [m2] camara CON vastago
REL_AREAS = A_A / A_B                                # 1.641

# MONTAJE VERTICAL, vastago hacia abajo: los desplazamientos POSITIVOS van a
# favor de la gravedad. De la punta cuelga la celda de carga. El peso es lo que
# hace descender el vastago solo cuando el carrete esta cerrado.
M_MOVIL = 150.0         # [kg] masa movil total (MEMORIA)
M_COLGANTE = 28.0       # [kg] celda de carga (laboratorio)
G = 9.81                # [m/s2]
PESO = (M_MOVIL + M_COLGANTE) * G                    # [N] ~1.75 kN, hacia +

# ============================================================================
# 2. LEY COMANDO -> VELOCIDAD  (MEDIDA, 2026-08-13)
# ============================================================================
# Barrido de 13 escalones con la UPH en marcha y sin probeta. Las dos ramas
# salieron casi perfectamente lineales (R2 ~ 0.999), y la captura APRBS de
# 610 s las reprodujo despues dentro del 3 %.
#
# El termino independiente NO es cero: hay deriva a comando nulo porque el peso
# cae por la fuga del carrete. Por eso la VELOCIDAD CERO se consigue en
# U_NULL ~ -0.37 V y no en 0 V. Es el dato que mas condiciona el proyecto,
# porque las consignas normadas caen justo a su lado.
K_POS, B_POS = 0.4459, 0.1139    # [mm/s por V], [mm/s]   rama u > 0
K_NEG, B_NEG = 0.3779, 0.1397    # [mm/s por V], [mm/s]   rama u < 0
U_NULL = -B_NEG / K_NEG          # [V] comando de velocidad nula: -0.370 V
ASIMETRIA = K_NEG / K_POS        # 0.847 medido (el modelo de 2 camaras da 0.772)

# EL NULL SE MUEVE CON LA TEMPERATURA DEL ACEITE. Entre las capturas `train` y
# `val`, tomadas con diez minutos de diferencia, se desplazo 71 mV = 1.58
# mm/min: MAS que la consigna entera del ensayo de losa. La especificacion de
# Moog (<= 2 % por 55 C) lo explica. Consecuencia: U_NULL es un punto de
# partida, no una constante, y el lazo necesita accion integral.
DERIVA_NULL_POR_55C = 0.20       # [V] especificacion Moog, con +-10 V <-> +-40 mA

# ============================================================================
# 3. TOPE DE CAUDAL  (MEDIDO, 2026-08-13)
# ============================================================================
# La placa de la UPH dice 1.7 L/min; lo medido es 1.9x esa cifra. Manda lo
# medido: con 1.7 el modelo saturaria a 1.41 mm/s y la planta real no lo hace.
#
# Como se vio: a fondo de escala, subir el comando de 8 a 10 V (+25 %) solo
# subio la velocidad de 2.536 a 2.634 mm/s (+3.9 %). La valvula se abre mas y
# no entra mas aceite.
Q_BOMBA = 3.18 / 60000           # [m3/s] = 3.18 L/min

# Solo satura al EXTENDER, que llena la camara grande. Retrayendo se llena la
# anular, asi que ese mismo caudal daria 4.32 mm/s y no se alcanza: ese sentido
# sigue limitado por el orificio de la servovalvula.
V_MAX_EXT = Q_BOMBA / A_A * 1e3  # [mm/s] 2.64 — techo real al extender
V_MAX_RET = Q_BOMBA / A_B * 1e3  # [mm/s] 4.32 — no se alcanza en la practica

# ============================================================================
# 4. LO QUE LIMITA LA MEDIDA
# ============================================================================
SIGMA_POSICION = 0.105   # [mm] ruido del sensor a 1 kHz, en reposo (MEDIDO)
# Hay ruido de modo comun a 133.8 Hz y 1462.6 Hz en posicion y en las dos
# presiones —los tres sensores alimentados a 24 V— y no en la celda. Bajarlo es
# la palanca de mayor retorno del proyecto: fija el techo de ajuste del modelo
# y limita la velocidad minima medible.

LATENCIA_LAZO = 5.46e-3  # [s] mediana de leer AI + escribir AO (MEDIDO)
TS_CONTROL = 0.020       # [s] Ts de control: deja la mitad del periodo libre
TS_MODELO = 0.100        # [s] Ts del modelo. A 20 ms el incremento por muestra
#                          queda POR DEBAJO del ruido (SNR 0.60 frente a 6.73)

# TRAS ARRANCAR LA HPU hay un transitorio de 2-3 MINUTOS durante el cual
# ninguna medida corta es fiable: se midio la deriva decayendo de +0.32 a
# -0.02 mm/s en 180 s. Con 25 s no basta.
CALENTAMIENTO = 180.0    # [s] espera obligatoria antes de medir nada

# ============================================================================
# 5. CONSIGNAS DE LOS ENSAYOS NORMADOS
# ============================================================================
# Aqui esta el argumento central del proyecto: las dos consignas se separan
# 62 mV y viven pegadas al cruce por cero, donde ademas cambia la rama de
# ganancia. Un PID lineal tiene que gobernar ahi con la misma sintonia que a
# 1 mm/s; es el hueco que llena un feedforward no lineal aprendido.
V_ENSAYO_LOSA = 1.5      # [mm/min]
V_ENSAYO_VIGA = 0.1      # [mm/min]


# ============================================================================
# 6. LAS TRES CUENTAS QUE SE USAN EN TODAS PARTES
# ============================================================================
def vel_medida(u):
    """Velocidad [mm/s] que produce el comando `u` [V], segun lo MEDIDO.

    Acepta escalar o array. Es la ley de la planta tal como se comporta, no
    como deberia comportarse: incluye la deriva y la asimetria entre ramas.
    """
    import numpy as np
    u = np.asarray(u, dtype=float)
    return np.where(u >= 0.0, K_POS * u + B_POS, K_NEG * u + B_NEG)


def u_para_vel(v: float, u_max: float = 10.0) -> float:
    """Comando [V] que produce la velocidad `v` [mm/s]. Inversa de `vel_medida`.

    Se usa para disenar la excitacion en el espacio de VELOCIDADES y despejar
    el comando despues. Muestrear el comando directamente no garantiza cubrir
    bien las velocidades una vez que el null esta desplazado: se comprobo, y la
    banda del ensayo de viga se quedaba con el 0 % de las muestras.
    """
    u = (v - B_POS) / K_POS if v >= B_POS else (v - B_NEG) / K_NEG
    return max(-u_max, min(u_max, u))


def fuerza_hidraulica(P_A_bar, P_B_bar):
    """Fuerza neta del actuador [kN] a partir de las DOS presiones.

        F = P_A*A_A - P_B*A_B

    NO es A_p*(P_A - P_B): el cilindro es asimetrico y con la formula simetrica
    el termino de la camara de fondo sale un 64 % corto. Da una medida de
    fuerza INDEPENDIENTE de la celda de carga, util para verificarla y para
    trabajar el lazo de fuerza sin probeta.

    Ojo al interpretarla sin probeta: la celda mide la reaccion EXTERNA (y sin
    probeta marca ~0), mientras que esto es la fuerza INTERNA del cilindro, que
    equilibra el peso mas la friccion.
    """
    return (P_A_bar * 1e5 * A_A - P_B_bar * 1e5 * A_B) / 1e3


if __name__ == "__main__":
    print("NUMEROS DE LA PRENSA UPH 50")
    print("=" * 64)
    print(f"  areas          A_A = {A_A*1e4:7.2f} cm2   A_B = {A_B*1e4:7.2f} cm2"
          f"   relacion {REL_AREAS:.3f}")
    print(f"  carrera util   {L_CARRERA*1e3:.0f} mm        peso movil {PESO/1e3:.2f} kN")
    print(f"  ley medida     u>0: v = {K_POS:.4f}*u + {B_POS:.4f}")
    print(f"                 u<0: v = {K_NEG:.4f}*u + {B_NEG:.4f}")
    print(f"  velocidad cero en u = {U_NULL:+.3f} V   (asimetria {ASIMETRIA:.3f})")
    print(f"  caudal maximo  {Q_BOMBA*60000:.2f} L/min  ->  extender topa a "
          f"{V_MAX_EXT:.3f} mm/s")
    print(f"  ruido posicion {SIGMA_POSICION*1e3:.0f} um     latencia del lazo "
          f"{LATENCIA_LAZO*1e3:.2f} ms")
    print()
    print("  Comando para las consignas normadas:")
    for nom, vmm in (("losa", V_ENSAYO_LOSA), ("viga", V_ENSAYO_VIGA)):
        print(f"    {nom:5s} {vmm:4.1f} mm/min -> u = {u_para_vel(vmm/60):+.4f} V")
