#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
nn_modelo.py — Modelo NARX de la prensa servohidraulica (Fase 1).

============================ GUIA DE LECTURA ============================
Que hace
--------
Identifica la dinamica comando -> movimiento con una red neuronal entrenada
sobre las capturas del equipo (`results/captura_{train,val}.csv`), y la valida
en SIMULACION LIBRE, que es el uso real del modelo.

LA DECISION ESTRUCTURAL: se predice el INCREMENTO, no la posicion
------------------------------------------------------------------
La planta es un INTEGRADOR (CLAUDE.md §3.3): el comando fija la velocidad y la
posicion es su integral. Si la red predijera la posicion absoluta,

        y(k+1) = f( y(k), y(k-1), ..., u(k), ... )

el termino y(k+1) ~ y(k) domina por completo y la red aprende la identidad: el
ajuste sale del 99 % y el modelo no sirve para nada. La formulacion correcta es

        dy(k+1) = f( dy(k), ..., dy(k-ny+1), u(k), ..., u(k-nu+1) )
        y(k+1)  = y(k) + dy(k+1)

donde dy = y(k) - y(k-1). En simulacion libre se realimenta el INCREMENTO
predicho y se reintegra.

EL Ts DEL MODELO NO ES EL Ts DEL CONTROL, y hay una razon medida
-----------------------------------------------------------------
El ruido de posicion es sigma = 0.105 mm (§5.2). Al diezmar promediando N
muestras baja como sqrt(N), pero el incremento por muestra CRECE con Ts. La
relacion senial/ruido del incremento a 1 mm/s -que es el orden del ensayo de
losa- sale asi:

        Ts        sigma_y     |dy| a 1 mm/s     SNR
        20 ms     0.0235 mm     0.020 mm        0.60     <- bajo el ruido
        50 ms     0.0148 mm     0.050 mm        2.38
       100 ms     0.0105 mm     0.100 mm        6.73     <- por defecto
       200 ms     0.0074 mm     0.200 mm       19.05

A Ts = 20 ms (el del lazo de control, §6.1) el incremento esta POR DEBAJO del
ruido justo en la banda que importa. Por eso el modelo se identifica mas lento.

No se pierde dinamica al hacerlo: a 50 Hz la servovalvula (120 Hz) y la
resonancia hidraulica (>315 Hz) ya estan por encima de Nyquist. En esta banda
la planta es un integrador con una ganancia estatica no lineal MAS un retardo
de transporte de ~5.5 ms; no hay dinamica rapida que capturar. Bajar el Ts solo
anadiria ruido.

RESULTADO PRINCIPAL: EL MEJOR MODELO NO TIENE REALIMENTACION
-------------------------------------------------------------
Se barrio cuanta historia de dy conviene realimentar, y el resultado va en
contra de la intuicion NARX (Ts = 100 ms, nu = 3, 3 reinicios):

    ny    un paso    LIBRE train    LIBRE val
     0     51.5 %      51.5 %         49.6 %     <-- el mejor
     1     56.1 %    -133.5 %       -440.9 %     <-- DIVERGE
     2     62.2 %      37.8 %         41.8 %
     3     62.1 %      33.9 %         31.9 %
     5     63.0 %      44.1 %         41.8 %

Cada dy realimentado MEJORA el ajuste a un paso y EMPEORA la simulacion libre.
La razon es medible: dy lleva el ruido del sensor de posicion (SNR ~ 6 a
Ts = 100 ms, §5.2) y u no lleva ninguno. Al entrenar a un paso, la red descubre
que puede usar el ruido de dy(k) para predecir parte del ruido de dy(k+1) —eso
sube el ajuste a un paso— pero en simulacion libre esa entrada ya no es el
ruido medido sino el ERROR de la propia red, que se realimenta y crece.

Y encaja con la fisica: a 100 ms la servovalvula (120 Hz) y la resonancia
hidraulica (>315 Hz) estan MUY por encima de Nyquist. En esta banda la planta
es un integrador con ganancia estatica no lineal, sin estado interno propio que
recordar. No hay nada que realimentar.

Por eso el modelo por defecto es FIR NO LINEAL:

        dy(k+1) = f( u(k), u(k-1) )

que ademas no puede acumular error por construccion: la simulacion libre y el
un paso son la MISMA cuenta. Barrido de nu (ny=0): 48.7 % con nu=1, 50.1 % con
nu=2, y a partir de nu=5 el ajuste de train sube mientras el de val baja.

METRICA: SIMULACION LIBRE, nunca "un paso adelante"
----------------------------------------------------
El "un paso" recibe la salida MEDIDA en cada instante y siempre se ve bien:
en una planta integradora es practicamente un test de que la red sabe copiar.
La simulacion libre realimenta la prediccion de la red, que es como se usa el
modelo cuando el neurocontrolador se entrena contra el. Es la unica que informa.

Para una planta integradora hay que medirla sobre el INCREMENTO (la velocidad):
la posicion es su integral y cualquier sesgo, por pequenio que sea, se acumula
sin limite. La reconstruccion de posicion se reporta aparte, por VENTANAS.

⚠ LINEA BASE, no modelo final
------------------------------
Esta version IGNORA a proposito que el null se desplazo 71 mV entre train y val
(CLAUDE.md §5.6c) — unos 20 C de aceite. Es la opcion (1) de las tres que se
plantearon: sirve de referencia y hace VISIBLE el problema. Se espera un SESGO
en validacion que ninguna cantidad de neuronas va a arreglar, porque no es
sobreajuste: es pedirle a un modelo que describa dos plantas ligeramente
distintas. El programa lo mide y lo reporta explicitamente.

Uso
---
    python tools/nn_modelo.py                      # FIR, Ts=100 ms, 15 ocultas
    python tools/nn_modelo.py --Ts 0.05 --hidden 30
    python tools/nn_modelo.py --barrido-ts         # compara varios Ts
"""

from __future__ import annotations

import argparse
import os

import numpy as np


# ============================================================================
# 1. DATOS
# ============================================================================
def carga(ruta: str, Ts: float, fs_orig: float = 1000.0) -> dict:
    """Lee una captura y la diezma a `Ts` PROMEDIANDO.

    El promediado no es un detalle: hace de filtro antialiasing y ademas baja
    el ruido como sqrt(N). Quedarse con una muestra de cada N (diezmado a secas)
    dejaria pasar todo el ruido de banda ancha y los tonos de 133.8 Hz y
    1462.6 Hz que se midieron en §5.2.
    """
    d = np.genfromtxt(ruta, delimiter=",", names=True)
    n_dec = int(round(Ts * fs_orig))
    n = (len(d["t_s"]) // n_dec) * n_dec

    def prom(v):
        return v[:n].reshape(-1, n_dec).mean(axis=1)

    return {
        "t": prom(d["t_s"]),
        "u": prom(d["u_V"]),
        "y": prom(d["posicion_mm"]),
        "P_A": prom(d["presion_A_bar"]),
        "P_B": prom(d["presion_B_bar"]),
        "Ts": Ts,
        "ruta": ruta,
    }


def regresor(dat: dict, ny: int, nu: int) -> tuple:
    """Construye la matriz de regresores y el objetivo.

        entrada X[k] = [ dy(k), ..., dy(k-ny+1), u(k), ..., u(k-nu+1) ]
        objetivo d[k] = dy(k+1)

    Devuelve (X, d, k0) donde k0 es el indice de la muestra original que
    corresponde a la primera fila, necesario para la simulacion libre.
    """
    y, u = dat["y"], dat["u"]
    dy = np.diff(y)                       # dy[i] = y[i+1] - y[i]
    k0 = max(ny, nu)                      # primera k con historia completa
    N = len(dy) - k0 - 1
    X = np.empty((N, ny + nu))
    for j in range(ny):                   # dy(k-j)
        X[:, j] = dy[k0 - j: k0 - j + N]
    for j in range(nu):                   # u(k-j)
        X[:, ny + j] = u[k0 - j: k0 - j + N]
    d = dy[k0 + 1: k0 + 1 + N]            # dy(k+1)
    return X, d, k0


# ============================================================================
# 2. LA RED, escrita a mano
# ============================================================================
class RedNARX:
    """Perceptron de una capa oculta: ne - nh - 1.

        h = tanh(X·V + bv)          [N, nh]
        o = h·W + bw                [N]

    Se escribe el gradiente de forma explicita porque el proyecto es didactico
    y tiene que poder reproducirse en el informe. Con  E = 1/2 <(o - d)^2>:

        e      = o - d                          [N]
        dE/dW  = h^T·e / N
        dE/dbw = <e>
        delta  = (e ⊗ W) * (1 - h^2)            [N, nh]   (derivada de tanh)
        dE/dV  = X^T·delta / N
        dE/dbv = <delta>
    """

    def __init__(self, ne: int, nh: int, rng: np.random.Generator):
        # Inicializacion de Xavier: mantiene la varianza al atravesar la capa
        self.V = rng.normal(0, np.sqrt(1.0 / ne), (ne, nh))
        self.bv = np.zeros(nh)
        self.W = rng.normal(0, np.sqrt(1.0 / nh), nh)
        self.bw = 0.0

    # -------------------------------------------------------------- adelante
    def adelante(self, X):
        h = np.tanh(X @ self.V + self.bv)
        return h @ self.W + self.bw, h

    def __call__(self, X):
        return self.adelante(X)[0]

    # ---------------------------------------------------------------- atras
    def gradiente(self, X, d):
        N = len(d)
        o, h = self.adelante(X)
        e = o - d
        gW = h.T @ e / N
        gbw = e.mean()
        delta = np.outer(e, self.W) * (1.0 - h * h)
        gV = X.T @ delta / N
        gbv = delta.mean(axis=0)
        return (gV, gbv, gW, gbw), 0.5 * np.mean(e * e)

    # ------------------------------------------------------------------ Adam
    def entrena(self, X, d, epocas=300, lote=256, lr=3e-3, rng=None,
                verboso=False):
        m = [np.zeros_like(p) for p in (self.V, self.bv, self.W)] + [0.0]
        v = [np.zeros_like(p) for p in (self.V, self.bv, self.W)] + [0.0]
        b1, b2, eps = 0.9, 0.999, 1e-8
        paso = 0
        N = len(d)
        rng = rng or np.random.default_rng(0)
        for ep in range(epocas):
            orden = rng.permutation(N)
            for i in range(0, N, lote):
                idx = orden[i:i + lote]
                g, _ = self.gradiente(X[idx], d[idx])
                paso += 1
                for k, (p_nom, gk) in enumerate(zip(("V", "bv", "W", "bw"), g)):
                    m[k] = b1 * m[k] + (1 - b1) * gk
                    v[k] = b2 * v[k] + (1 - b2) * gk * gk
                    mhat = m[k] / (1 - b1**paso)
                    vhat = v[k] / (1 - b2**paso)
                    act = lr * mhat / (np.sqrt(vhat) + eps)
                    setattr(self, p_nom, getattr(self, p_nom) - act)
            if verboso and (ep + 1) % 50 == 0:
                _, E = self.gradiente(X, d)
                print(f"      epoca {ep+1:4d}   E = {E:.6e}")


# ============================================================================
# 2b. RED DINAMICA entrenada por DBP (Dynamic Back-Propagation)
# ============================================================================
class RedDBP:
    r"""Modelo RECURRENTE entrenado propagando las sensibilidades EN EL TIEMPO.

    Estructura (Narendra & Parthasarathy, 1990):

        h(k)   = tanh( [x(k), u(k)]·V + bv )         [nh]
        x(k+1) = h(k)·W + b                          [ns]
        dy(k)  = x(k)[0]                             solo la 1a componente se observa

    El estado `x` es INTERNO: la red no recibe nunca la medida, ni siquiera al
    entrenar. Esa es la diferencia con el NARX serie-paralelo, y trae dos
    consecuencias:

      1. NO HAY REGRESOR RUIDOSO. En el NARX la entrada dy(k) es la medida, con
         su ruido, y eso sesga la estimacion (errores en las variables). Aqui el
         ruido queda solo en el OBJETIVO, donde no sesga: solo anade varianza.
      2. SE ENTRENA SOBRE EL ERROR DE SIMULACION LIBRE, que es la metrica que de
         verdad importa. El NARX se entrena a un paso y se SELECCIONA por
         simulacion libre, que es un parche: se optimiza un criterio y se elige
         por otro. Aqui coinciden por construccion.

    DBP frente a BPTT: DBP propaga las sensibilidades dx/dtheta hacia ADELANTE
    en el tiempo (es lo mismo que calcula RTRL). BPTT obtiene el MISMO gradiente
    hacia atras, desenrollando la trayectoria. Se usa DBP porque es lo que pide
    el curso y porque no necesita guardar la trayectoria entera.

    LA RECURSION, que es todo el metodo:

        S(k) = dx(k)/dtheta                          [ns, P]
        S(k+1) = J(k)·S(k)  +  E(k)
                 \_______/     \__/
                 propagado      termino explicito de este paso

        J(k)[j,i] = dx(k+1)_j / dx(k)_i = sum_m V[i,m]·(1-h_m^2)·W[m,j]

    y el gradiente del error acumulado es  sum_k e(k)·S(k)[0,:]  con
    e(k) = x(k)[0] - dy_medido(k).
    """

    def __init__(self, ns: int, ni: int, nh: int, rng: np.random.Generator):
        ne = ns + ni
        self.ns, self.ni, self.nh, self.ne = ns, ni, nh, ne
        self.V = rng.normal(0, np.sqrt(1.0 / ne), (ne, nh))
        self.bv = np.zeros(nh)
        self.W = rng.normal(0, np.sqrt(1.0 / nh), (nh, ns))
        self.b = np.zeros(ns)

    # ------------------------------------------------------------ un paso
    def paso(self, x, u):
        e = np.concatenate([x, np.atleast_1d(u)])
        h = np.tanh(e @ self.V + self.bv)
        return h @ self.W + self.b, h, e

    # ------------------------------------------------- simulacion libre
    def simula(self, useq, x0=None):
        """Recorre la secuencia SIN ver ni una sola medida."""
        x = np.zeros(self.ns) if x0 is None else np.array(x0, dtype=float)
        out = np.empty(len(useq))
        for k in range(len(useq)):
            out[k] = x[0]
            x, _, _ = self.paso(x, useq[k])
        return out

    # ------------------------------------------------------ gradiente DBP
    def gradiente(self, useq, dseq, x0=None):
        """Sensibilidades hacia adelante sobre una subsecuencia. Devuelve
        (gV, gbv, gW, gb) y el error cuadratico medio."""
        ns, nh, ne = self.ns, self.nh, self.ne
        x = np.zeros(ns) if x0 is None else np.array(x0, dtype=float)
        # S: derivada del estado respecto de cada parametro
        SV = np.zeros((ns, ne, nh)); Sbv = np.zeros((ns, nh))
        SW = np.zeros((ns, nh, ns)); Sb = np.zeros((ns, ns))
        gV = np.zeros_like(self.V); gbv = np.zeros_like(self.bv)
        gW = np.zeros_like(self.W); gb = np.zeros_like(self.b)
        E = 0.0
        L = len(dseq)
        for k in range(L):
            err = x[0] - dseq[k]
            E += err * err
            # el gradiente solo "ve" la primera componente del estado
            gV += err * SV[0]; gbv += err * Sbv[0]
            gW += err * SW[0]; gb += err * Sb[0]

            xn, h, e = self.paso(x, useq[k])
            g = 1.0 - h * h                                   # tanh'
            # J[j,i] = dx(k+1)_j/dx(k)_i
            J = ((self.V[:ns, :] * g) @ self.W).T             # [ns, ns]

            # propagar y anadir el termino explicito de este paso
            SV = np.einsum('ji,iab->jab', J, SV)
            Sbv = J @ Sbv
            SW = np.einsum('ji,iab->jab', J, SW)
            Sb = J @ Sb
            Wg = self.W * g[:, None]                          # [nh, ns]
            SV += np.einsum('a,mj->jam', e, Wg)               # dx_j/dV[a,m]
            Sbv += Wg.T                                       # dx_j/dbv[m]
            for j in range(ns):
                SW[j, :, j] += h                              # dx_j/dW[m,j]
                Sb[j, j] += 1.0
            x = xn
        return (gV / L, gbv / L, gW / L, gb / L), 0.5 * E / L

    # ------------------------------------------------------------- Adam
    def entrena(self, useq, dseq, epocas=60, largo=400, lr=5e-3, rng=None):
        """Adam sobre subsecuencias. Se trocea la serie porque las
        sensibilidades se propagan dentro del trozo: trozos largos dan un
        gradiente mas fiel pero menos actualizaciones por epoca."""
        pars = ("V", "bv", "W", "b")
        m = [np.zeros_like(getattr(self, p)) for p in pars]
        v = [np.zeros_like(getattr(self, p)) for p in pars]
        b1, b2, eps = 0.9, 0.999, 1e-8
        paso_n = 0
        rng = rng or np.random.default_rng(0)
        n_tr = len(dseq) // largo
        for _ in range(epocas):
            for i in rng.permutation(n_tr):
                a = i * largo
                gs, _ = self.gradiente(useq[a:a + largo], dseq[a:a + largo])
                paso_n += 1
                for k, (p, gk) in enumerate(zip(pars, gs)):
                    m[k] = b1 * m[k] + (1 - b1) * gk
                    v[k] = b2 * v[k] + (1 - b2) * gk * gk
                    mh = m[k] / (1 - b1**paso_n); vh = v[k] / (1 - b2**paso_n)
                    setattr(self, p, getattr(self, p) - lr * mh / (np.sqrt(vh) + eps))


def verifica_gradiente_dbp(semilla=0):
    """Contrasta el gradiente DBP contra diferencias finitas.

    Es la comprobacion que separa "el codigo corre" de "el codigo calcula lo
    que dice calcular". Sin esto, un signo mal puesto en la recursion se
    manifiesta como "la red no aprende" y se pierde un dia buscandolo en el
    sitio equivocado.
    """
    rng = np.random.default_rng(semilla)
    red = RedDBP(ns=2, ni=1, nh=4, rng=rng)
    u = rng.normal(size=25); d = rng.normal(size=25) * 0.1
    gs, _ = red.gradiente(u, d)
    print("  VERIFICACION DEL GRADIENTE (DBP vs diferencias finitas)")
    h = 1e-6
    for nom, g in zip(("V", "bv", "W", "b"), gs):
        P = getattr(red, nom); num = np.zeros_like(P); it = np.nditer(P, flags=['multi_index'])
        while not it.finished:
            i = it.multi_index; o = P[i]
            P[i] = o + h; _, Ep = red.gradiente(u, d)
            P[i] = o - h; _, Em = red.gradiente(u, d)
            P[i] = o; num[i] = (Ep - Em) / (2 * h)
            it.iternext()
        err = np.max(np.abs(num - g)) / (np.max(np.abs(num)) + 1e-30)
        print(f"    {nom:3s}: error relativo maximo = {err:.2e}")


# ============================================================================
# 3. SIMULACION LIBRE — la metrica que manda
# ============================================================================
def simula_libre(red, esc, dat: dict, ny: int, nu: int, k0: int, N: int):
    """Realimenta la PREDICCION de la red, no la medida.

    El estado son los `ny` ultimos incrementos PREDICHOS. El comando `u` es
    conocido (es lo que se emitio). Devuelve el vector de incrementos simulados.
    """
    u = dat["u"]
    dy_med = np.diff(dat["y"])
    hist = list(dy_med[k0 - ny + 1: k0 + 1][::-1])    # dy(k), dy(k-1), ...
    salida = np.empty(N)
    for i in range(N):
        k = k0 + i
        x = np.array(hist[:ny] + [u[k - j] for j in range(nu)])
        xn = (x - esc["mx"]) / esc["sx"]
        dyp = float(red(xn[None, :])[0]) * esc["sd"] + esc["md"]
        salida[i] = dyp
        hist.insert(0, dyp)
        hist.pop()
    return salida


def ajuste(real, pred) -> float:
    """Fit % al estilo de la System Identification Toolbox:
        100 * (1 - ||real-pred|| / ||real-mean(real)||)
    100 = perfecto; 0 = tan bueno como predecir la media."""
    return 100.0 * (1.0 - np.linalg.norm(real - pred) /
                    np.linalg.norm(real - real.mean()))


def r2(real, pred) -> float:
    ss = np.sum((real - pred) ** 2)
    st = np.sum((real - real.mean()) ** 2)
    return 1.0 - ss / st


def evalua(red, esc, dat, ny, nu, etiq):
    """Un paso + simulacion libre + reconstruccion de posicion por ventanas."""
    X, d, k0 = regresor(dat, ny, nu)
    Xn = (X - esc["mx"]) / esc["sx"]

    # --- un paso adelante (recibe la medida en cada instante) --------------
    d1 = red(Xn) * esc["sd"] + esc["md"]
    # --- simulacion libre (se realimenta a si misma) -----------------------
    dl = simula_libre(red, esc, dat, ny, nu, k0, len(d))

    res = {
        "etiq": etiq, "n": len(d),
        "fit_1": ajuste(d, d1), "r2_1": r2(d, d1),
        "fit_L": ajuste(d, dl), "r2_L": r2(d, dl),
        "sesgo": float(np.mean(dl - d)),          # mm por muestra
        "Ts": dat["Ts"], "d": d, "dl": dl, "k0": k0,
    }
    res["sesgo_mm_min"] = res["sesgo"] / dat["Ts"] * 60.0
    return res


def posicion_por_ventanas(dat, res, ventana_s=60.0):
    """Reintegra la velocidad simulada en ventanas, reiniciando de la posicion
    MEDIDA al principio de cada una.

    Por que por ventanas: la posicion es la integral del incremento, asi que un
    sesgo constante crece SIN LIMITE. Reintegrar 600 s seguidos no mide la
    calidad del modelo, mide el sesgo multiplicado por el tiempo. Con ventanas
    se ve lo que de verdad importa para el control: cuanto se desvia el modelo
    en el horizonte en que se va a usar.
    """
    Ts = dat["Ts"]; nv = int(ventana_s / Ts)
    y = dat["y"]; k0 = res["k0"]
    errs = []
    for a in range(0, len(res["dl"]) - nv, nv):
        y0 = y[k0 + a]
        yp = y0 + np.cumsum(res["dl"][a:a + nv])
        ym = y[k0 + a + 1: k0 + a + nv + 1]
        errs.append(np.abs(yp - ym).max())
    return np.array(errs)


# ============================================================================
# 4. PROGRAMA
# ============================================================================
def entrena_y_evalua(Ts, ny, nu, nh, epocas, reinicios, semilla, verboso=True):
    tr = carga("results/captura_train.csv", Ts)
    va = carga("results/captura_val.csv", Ts)
    Xtr, dtr, _ = regresor(tr, ny, nu)

    esc = {"mx": Xtr.mean(0), "sx": Xtr.std(0) + 1e-12,
           "md": dtr.mean(), "sd": dtr.std() + 1e-12}
    Xn = (Xtr - esc["mx"]) / esc["sx"]
    dn = (dtr - esc["md"]) / esc["sd"]

    if verboso:
        print(f"\n  Ts = {Ts*1000:.0f} ms   regresor {ny}x dy + {nu}x u = "
              f"{ny+nu} entradas   red {ny+nu}-{nh}-1")
        print(f"  patrones: train {len(dtr)}   val {len(regresor(va,ny,nu)[1])}")

    # SELECCION POR REINICIOS. Entrenar a un paso NO garantiza que el modelo sea
    # estable al realimentarse: hay semillas que ajustan bien y divergen en
    # simulacion libre. Se entrenan varias y se elige la que mejor SIMULA en
    # datos NO VISTOS.
    mejor, mejor_fit = None, -np.inf
    for r in range(reinicios):
        rng = np.random.default_rng(semilla + r)
        red = RedNARX(ny + nu, nh, rng)
        red.entrena(Xn, dn, epocas=epocas, rng=rng)
        rv = evalua(red, esc, va, ny, nu, "val")
        if verboso:
            print(f"    reinicio {r+1}/{reinicios}: fit libre en val = "
                  f"{rv['fit_L']:6.2f} %")
        if rv["fit_L"] > mejor_fit:
            mejor, mejor_fit = red, rv["fit_L"]
    return mejor, esc, tr, va


def techo_ruido(dat: dict, sigma_1k: float = 0.105, fs_orig: float = 1000.0) -> float:
    """Ajuste MAXIMO alcanzable, impuesto por el ruido del sensor.

    Parte de sigma = 0.105 mm medido en reposo (§5.2). Al diezmar promediando N
    muestras el ruido baja como sqrt(N); el incremento y(k)-y(k-1) combina dos
    muestras independientes, de ahi el sqrt(2):

        sigma_dy = sigma_1k / sqrt(N) * sqrt(2)
        techo    = 100 * (1 - sigma_dy / std(dy))

    Sin esta referencia el fit% no se puede interpretar: un 50 % suena mal hasta
    que se ve que el maximo posible es ~58 %.
    """
    N = int(round(dat["Ts"] * fs_orig))
    sigma_dy = sigma_1k / np.sqrt(N) * np.sqrt(2)
    return 100.0 * (1.0 - sigma_dy / np.diff(dat["y"]).std())


def informe(red, esc, tr, va, ny, nu):
    rt = evalua(red, esc, tr, ny, nu, "train")
    rv = evalua(red, esc, va, ny, nu, "val")
    print("\n  " + "=" * 64)
    print(f"  {'serie':7s} {'un paso':>10s} {'SIM LIBRE':>11s} {'R2 libre':>10s}"
          f" {'sesgo':>14s}")
    for r in (rt, rv):
        print(f"  {r['etiq']:7s} {r['fit_1']:9.2f}% {r['fit_L']:10.2f}%"
              f" {r['r2_L']:10.4f} {r['sesgo_mm_min']:+11.3f} mm/min")
    print("  " + "=" * 64)
    for r, dat in ((rt, tr), (rv, va)):
        e = posicion_por_ventanas(dat, r, 60.0)
        print(f"  {r['etiq']:7s}: error MAXIMO de posicion en ventanas de 60 s: "
              f"mediana {np.median(e):5.2f} mm, peor {e.max():5.2f} mm")
    print()
    for r, dat in ((rt, tr), (rv, va)):
        techo = techo_ruido(dat)
        print(f"  {r['etiq']:7s}: techo impuesto por el ruido = {techo:5.1f} %"
              f"   -> alcanzado el {r['fit_L']/techo*100:.0f} % de lo posible")
    if ny == 0:
        print("\n  (ny=0: el modelo NO se realimenta, asi que 'un paso' y")
        print("   'simulacion libre' son la misma cuenta y no puede acumular error)")
    return rt, rv


def dbp_prepara(dat, esc=None):
    """Serie (u, dy) escalada, lista para el modelo dinamico."""
    dy = np.diff(dat["y"])
    u = dat["u"][:len(dy)]
    if esc is None:
        esc = {"mu": u.mean(), "su": u.std() + 1e-12,
               "md": dy.mean(), "sd": dy.std() + 1e-12}
    return (u - esc["mu"]) / esc["su"], (dy - esc["md"]) / esc["sd"], dy, esc


def corre_dbp(args):
    tr = carga("results/captura_train.csv", args.Ts)
    va = carga("results/captura_val.csv", args.Ts)
    un_tr, dn_tr, dy_tr, esc = dbp_prepara(tr)
    un_va, dn_va, dy_va, _ = dbp_prepara(va, esc)

    print(f"\n  Ts = {args.Ts*1000:.0f} ms   estado ns={args.ns}   red "
          f"{args.ns+1}-{args.hidden}-{args.ns}   trozos de {args.largo} muestras")
    verifica_gradiente_dbp()

    mejor, fmej = None, -np.inf
    for r in range(args.reinicios):
        rng = np.random.default_rng(args.seed + r)
        red = RedDBP(args.ns, 1, args.hidden, rng)
        red.entrena(un_tr, dn_tr, epocas=args.epocas, largo=args.largo, rng=rng)
        pv = red.simula(un_va) * esc["sd"] + esc["md"]
        f = ajuste(dy_va, pv)
        print(f"    reinicio {r+1}/{args.reinicios}: fit libre en val = {f:6.2f} %")
        if f > fmej:
            mejor, fmej = red, f

    print("\n  " + "=" * 64)
    print(f"  {'serie':7s} {'SIM LIBRE':>11s} {'R2':>9s} {'sesgo':>16s} {'techo':>8s}")
    for et, un, dy, dat in (("train", un_tr, dy_tr, tr), ("val", un_va, dy_va, va)):
        p = mejor.simula(un) * esc["sd"] + esc["md"]
        techo = techo_ruido(dat)
        print(f"  {et:7s} {ajuste(dy, p):10.2f}% {r2(dy, p):9.4f} "
              f"{np.mean(p-dy)/dat['Ts']*60:+11.3f} mm/min {techo:7.1f}%")
    print("  " + "=" * 64)
    np.savez("results/nn_dbp.npz", V=mejor.V, bv=mejor.bv, W=mejor.W, b=mejor.b,
             ns=args.ns, Ts=args.Ts, **esc)
    print("\n[ok] results/nn_dbp.npz")


def main():
    ap = argparse.ArgumentParser(description="Modelo NARX de la prensa (Fase 1)")
    ap.add_argument("--tipo", choices=["estatico", "dinamico"], default="estatico",
                    help="estatico = NARX/FIR entrenado a un paso; "
                         "dinamico = recurrente entrenado por DBP")
    ap.add_argument("--ns", type=int, default=2, help="dimension del estado (DBP)")
    ap.add_argument("--largo", type=int, default=400,
                    help="longitud de la subsecuencia para propagar (DBP)")
    ap.add_argument("--Ts", type=float, default=0.100, help="Ts del modelo [s]")
    ap.add_argument("--ny", type=int, default=0,
                    help="incrementos pasados realimentados (0 = FIR, el mejor)")
    ap.add_argument("--nu", type=int, default=2, help="comandos pasados")
    ap.add_argument("--hidden", type=int, default=15)
    ap.add_argument("--epocas", type=int, default=250)
    ap.add_argument("--reinicios", type=int, default=3)
    ap.add_argument("--epocas-dbp", type=int, default=25)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--barrido-ts", action="store_true",
                    help="compara varios Ts y sale")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    print("=" * 70)
    tit = ("NARX/FIR entrenado a UN PASO" if args.tipo == "estatico"
           else "RED DINAMICA entrenada por DBP (simulacion libre directa)")
    print(f"MODELO SOBRE INCREMENTOS — {tit}")
    print("=" * 70)

    if args.tipo == "dinamico":
        corre_dbp(args)
        return

    if args.barrido_ts:
        print(f"\n{'Ts':>7s} {'fit libre train':>16s} {'fit libre val':>14s}"
              f" {'sesgo val':>15s}")
        for Ts in (0.020, 0.050, 0.100, 0.200):
            red, esc, tr, va = entrena_y_evalua(
                Ts, args.ny, args.nu, args.hidden, args.epocas, 2,
                args.seed, verboso=False)
            rt = evalua(red, esc, tr, args.ny, args.nu, "train")
            rv = evalua(red, esc, va, args.ny, args.nu, "val")
            print(f"{Ts*1000:5.0f}ms {rt['fit_L']:15.2f}% {rv['fit_L']:13.2f}%"
                  f" {rv['sesgo_mm_min']:+12.3f} mm/min")
        return

    red, esc, tr, va = entrena_y_evalua(
        args.Ts, args.ny, args.nu, args.hidden, args.epocas,
        args.reinicios, args.seed)
    rt, rv = informe(red, esc, tr, va, args.ny, args.nu)

    os.makedirs("results", exist_ok=True)
    np.savez("results/nn_modelo.npz", V=red.V, bv=red.bv, W=red.W, bw=red.bw,
             mx=esc["mx"], sx=esc["sx"], md=esc["md"], sd=esc["sd"],
             Ts=args.Ts, ny=args.ny, nu=args.nu)
    print("\n[ok] results/nn_modelo.npz")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(13, 7))
        for a, r, dat in zip(ax, (rt, rv), (tr, va)):
            t = dat["t"][r["k0"] + 1: r["k0"] + 1 + r["n"]]
            v_med = r["d"] / dat["Ts"] * 60
            v_sim = r["dl"] / dat["Ts"] * 60
            a.plot(t, v_med, lw=.5, color="tab:cyan", label="medido")
            a.plot(t, v_sim, lw=.5, color="tab:red", alpha=.8,
                   label="simulacion libre")
            a.set_ylabel("velocidad [mm/min]")
            a.set_title(f"{r['etiq']} — fit libre {r['fit_L']:.1f} %, "
                        f"sesgo {r['sesgo_mm_min']:+.2f} mm/min")
            a.legend(fontsize=8); a.grid(alpha=.3)
        ax[-1].set_xlabel("t [s]")
        fig.tight_layout()
        fig.savefig("results/nn_modelo.png", dpi=110)
        print("[ok] results/nn_modelo.png")


if __name__ == "__main__":
    main()
