#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
planta_sim.py — Simulador fisico del sistema servohidraulico UPH 50 (Fluidtek).

============================ GUIA DE LECTURA ============================
Para que sirve
--------------
Es el BANCO DE PRUEBAS del proyecto. Antes de gastar tiempo de maquina (y antes
de mover un cilindro de 200 kN con una red neuronal sin entrenar) queremos poder
responder a estas preguntas en el escritorio:

  * la secuencia de excitacion que vamos a lanzar, ¿recorre lo que creemos y se
    queda dentro de la ventana de posicion segura?
  * el NARX sobre INCREMENTOS (ver CLAUDE.md §3.3), ¿realmente identifica una
    planta con esta estructura, o el ajuste alto es un espejismo?
  * el neurocontrolador entrenado por BPTT, ¿estabiliza una planta integradora
    con delta_h ~ 0?

NO es la planta real y no pretende serlo. Es una planta CON LA MISMA ESTRUCTURA,
que es lo que hace falta para validar el metodo.

MODELO DE DOS CAMARAS — y por que no vale el de una
----------------------------------------------------
La memoria de Fluidtek modela el cilindro como SIMETRICO con un area unica
A_p = 122.52 cm^2 (su Fig. 3.3). El cilindro real NO es simetrico:

    camara A (SIN vastago, fondo) : A_A = pi/4*D^2        = 201.06 cm^2
    camara B (CON vastago, anular): A_B = pi/4*(D^2-d^2)  = 122.52 cm^2
    relacion  A_A/A_B = 1.641

O sea: la "A_p" de la memoria es en realidad el area de la camara ANULAR, y usar
solo esa equivale a suponer un cilindro de doble vastago que aqui no existe.
Consecuencias que un modelo de una sola camara NO puede reproducir:

  1. LA GANANCIA DE VELOCIDAD ES DISTINTA EN CADA SENTIDO, y el sentido de la
     diferencia es CONTRAINTUITIVO. La cuenta ingenua ("A_B es menor, luego
     retraer sera 1.64x mas rapido") es FALSA, porque el caudal no lo fija el
     area sino el orificio: al retraer, la camara GRANDE tiene que evacuar
     A_A*v por el orificio de retorno, y ese es el cuello de botella. Resolviendo
     el regimen (continuidad + balance de fuerzas + ecuacion de orificio) sale
     que RETRAER ES MAS LENTO, con relacion ~0.77. Y los datos de 2017 dan
     0.0114/0.0147 = 0.78 (CLAUDE.md §5.1): coincide. Ver `presiones_regimen`.
  2. INTENSIFICACION DE PRESION. Al retener o frenar, la camara anular puede
     subir a P_A*(A_A/A_B) = 164 bar con solo 100 bar en la de fondo. Es la razon
     de que los dos transductores tengan rangos distintos.
  3. Con DOS presiones medidas por separado (ai2, ai3) se puede validar el modelo
     contra estados internos, no solo contra la posicion.

Ecuaciones implementadas
------------------------
    Servovalvula (2.o orden, del Bode del fabricante a -3 dB):
        xs'' = w_sv^2*(u_norm - xs) - 2*d_sv*w_sv*xs'

    Caudales por la servovalvula de 4 vias (ecuacion de orificio, con signo):
        xs > 0 (extiende):  Q_A = +Kq*xs*sqrt((Ps-P_A)/dPref)   [entra en A]
                            Q_B = +Kq*xs*sqrt((P_B-Pt)/dPref)   [sale de B]
        xs < 0 (retrae) :   Q_A = +Kq*xs*sqrt((P_A-Pt)/dPref)   [sale de A]
                            Q_B = +Kq*xs*sqrt((Ps-P_B)/dPref)   [entra en B]

    Compresibilidad en cada camara (volumen variable con la carrera):
        dP_A/dt = (beta/V_A)*( Q_A - A_A*v - fuga )   ,  V_A = V_A0 + A_A*x
        dP_B/dt = (beta/V_B)*( A_B*v - Q_B + fuga )   ,  V_B = V_B0 + A_B*(L-x)

    Mecanica:
        M*dv/dt = P_A*A_A - P_B*A_B - B_p*v - F_friccion - F_carga
        dx/dt   = v

No linealidades (todas OPCIONALES)
-----------------------------------
Son las que justifican usar una red neuronal en vez de un PID lineal:
  null_off   deriva de la posicion nula del carrete (medida en 2017: §5.1)
  solape     centro cerrado -> zona muerta pequenia
  histeresis juego del carrete (tipo backlash)
  F_stick    friccion estatica de sellos -> stick-slip a muy baja velocidad,
             que es justo el regimen de los ensayos normados (0.1 mm/min)
  probeta    contacto elastico con rotura a F_rotura

Uso
---
    python tools/planta_sim.py --check           # verifica parametros derivados
    python tools/planta_sim.py --demo escalon
    python tools/planta_sim.py --demo asimetria  # extension vs retraccion
    python tools/planta_sim.py --demo probeta

Como libreria:
    from planta_sim import PlantaHidraulica, ParamsPlanta
    p = PlantaHidraulica(ParamsPlanta())
    r = p.simula(u, Ts=0.01)     # u = comando en VOLTIOS (array), ZOH
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass

import numpy as np


# ============================================================================
# 1. PARAMETROS
# ============================================================================
@dataclass
class ParamsPlanta:
    """Parametros fisicos del equipo.

    Fuentes, por orden de autoridad:
      1. MEDIDAS en el equipo (2026-08-12): carrera, escalas de sensor, ganancia
         de velocidad, asimetria y deriva de null. Ver CLAUDE.md §5.3.
      2. CATALOGO Moog 761 Rev. M para la servovalvula G761-3001B H04JOFM4VPL,
         que corrige caudal nominal, corriente y ancho de banda (CLAUDE.md §2.2).
      3. Memoria de Fluidtek 2017 para el resto (beta, V_t, M_t...), sabiendo
         que esta desactualizada: ver las discrepancias en CLAUDE.md §8.
    """

    # --- Geometria del actuador ------------------------------------------
    D_pistón: float = 0.160      # [m] diametro del embolo
    d_vastago: float = 0.100     # [m] diametro del vastago
    L_carrera: float = 0.150     # [m] CARRERA UTIL (dato del laboratorio;
    #                                  la memoria citaba un sensor de 400 mm)
    V_A0: float = 0.3e-3         # [m^3] volumen muerto camara A (+ tuberia)
    V_B0: float = 0.3e-3         # [m^3] volumen muerto camara B (+ tuberia)

    M_t: float = 150.0           # [kg]    masa movil total
    # MONTAJE VERTICAL con el vastago hacia abajo (dato del laboratorio,
    # 2026-08-12): los desplazamientos positivos van A FAVOR de la gravedad.
    # De la punta cuelga la celda de carga, ~28 kg. El peso del conjunto movil
    # entra como fuerza CONSTANTE en el sentido positivo, y es lo que explica
    # que el actuador descienda solo con el carrete cerrado (§5.3).
    vertical: bool = True
    m_colgante: float = 28.0     # [kg]    celda de carga colgando del vastago
    B_p: float = 0.0             # [N*s/m] amortiguamiento viscoso
    beta: float = 1.7e9          # [Pa]    modulo de compresibilidad del aceite

    # OJO: fuga cruzada FISICA entre camaras, por defecto 0. NO confundir con el
    # K_ce = 2e-12 m^5/(N*s) de la memoria: aquel es el coeficiente LINEALIZADO
    # caudal-presion, que agrupa la fuga real MAS la pendiente dQ/dP de la propia
    # servovalvula. En este modelo esa pendiente ya la aporta la ecuacion de
    # orificio no lineal (`_caudales`), asi que meter K_ce aqui la contaria DOS
    # VECES. Se comprobo el efecto: con K_fuga=2e-12 la relacion de velocidades
    # retraer/extender salia 0.363, y con K_fuga=0 sale 0.772, que es justo lo
    # que da la solucion analitica de regimen. Subir este valor solo si se mide
    # una fuga real en el cilindro.
    K_fuga: float = 0.0          # [m^5/(N*s)] fuga cruzada entre camaras

    # --- Servovalvula: MOOG G761-3001B  H04JOFM4VPL -----------------------
    # Datos del catalogo Moog 761 Series (Rev. M, 2024), decodificando el
    # codigo de pedido. Sustituyen a los de la memoria de 2017, que describia
    # otra variante:
    #   H   High response          04  4 L/min (1.0 gpm) a dP_N = 35 bar POR LAND
    #   J   axis cut, ZERO LAP     O   315 bar, cuerpo de aluminio
    #   F   pilot STANDARD dyn.    M   centrada sin senial
    #   4   pilotaje interno (P)   V   juntas FKM
    #   P   conector 4 pin lado P  L   +-40 mA single/paralelo, +-20 mA serie
    omega_sv: float = 2 * math.pi * 120.0   # [rad/s] 120 Hz a -3 dB para
    #   H04..F (pag. 9 del catalogo). La memoria usaba 150 Hz, que no
    #   corresponde a esta variante: el ..G (High dynamics) da 140 Hz y este
    #   es ..F (Standard dynamics). Ademas el dato del catalogo se mide a
    #   210 bar de pilotaje y aqui se trabaja a 100 bar, asi que la respuesta
    #   real sera algo MAS LENTA que estos 120 Hz.
    delta_sv: float = 0.7        # [-]

    # Corriente para 100 % de carrera del carrete. El codigo L admite dos
    # conexionados y NO es lo mismo: en serie son +-20 mA y en single/paralelo
    # +-40 mA. La ganancia medida en el equipo (0.446 mm/s por V) encaja con
    # +-40 mA (predice 0.377) y no con +-20 mA (predice 0.755), asi que se
    # toma el paralelo. CONFIRMAR mirando el cableado del amplificador.
    i_nom: float = 40e-3         # [A]
    Q_nom: float = 4.0 / 60000   # [m^3/s] 4 L/min nominales (no 4.78)
    dP_nom: float = 35e5         # [Pa] caida POR LAND, tal como la define el
    #                                 catalogo (70 bar entre los dos lands)

    # --- No linealidades declaradas por el fabricante (pag. 7) -------------
    # Se dejan como referencia de que valores son PLAUSIBLES al activar las no
    # linealidades de abajo; medidas a 210 bar, 32 mm2/s y 40 C.
    #   histeresis tipica            <= 3.0 % de la corriente nominal
    #   umbral (threshold) tipico    <= 0.5 %
    #   deriva de null por dT=55 C   <= 2.0 %
    #   tolerancia de caudal          +-10 %
    # El carrete es ZERO LAP: el catalogo describe "minimal change in gain
    # through null region", que es justo lo que se midio (dos ramas lineales
    # con R2 ~ 0.999 y sin meseta). Por eso `solape` se deja en 0.

    # --- Cadena de mando --------------------------------------------------
    K_amp: float = 0.003         # [A/V] (!) ganancia del amplificador
    u_max: float = 10.0          # [V]   rango del AO (NI 9263)

    # --- Circuito ---------------------------------------------------------
    P_s: float = 100e5           # [Pa] presion de suministro (limitadora)
    P_t: float = 2e5             # [Pa] presion de retorno (tanque)
    Q_bomba: float = 10.0 / 60000  # [m^3/s] (!) tope de caudal de la bomba

    # --- No linealidades (0 = desactivada) --------------------------------
    null_off: float = 0.0
    solape: float = 0.0
    histeresis: float = 0.0
    F_stick: float = 0.0         # [N] friccion estatica (breakaway)
    F_coul: float = 0.0          # [N] friccion de Coulomb
    v_stick: float = 1e-6        # [m/s] umbral de "parado"

    # --- Carga / probeta --------------------------------------------------
    k_probeta: float = 0.0       # [N/m]
    x_contacto: float = 0.0      # [m]
    F_rotura: float = float("inf")

    # --- Integracion ------------------------------------------------------
    dt_int: float = 5e-5         # [s] paso interno (20 kHz)

    # ---------------------------------------------------------------- areas
    @property
    def A_A(self) -> float:
        """Area de la camara SIN vastago (fondo) [m^2]. ~201.06 cm^2."""
        return math.pi / 4 * self.D_pistón**2

    @property
    def A_B(self) -> float:
        """Area de la camara CON vastago (anular) [m^2]. ~122.52 cm^2.
        Es la que la memoria llama A_p."""
        return math.pi / 4 * (self.D_pistón**2 - self.d_vastago**2)

    @property
    def rel_areas(self) -> float:
        return self.A_A / self.A_B

    @property
    def K_q(self) -> float:
        return self.Q_nom / 1.0     # caudal a xs=1 y caida nominal

    # ------------------------------------------------- parametros derivados
    def omega_h(self, x: float | None = None) -> float:
        """Frecuencia natural hidraulica [rad/s], en la posicion x [m].

        Depende de la POSICION porque los volumenes cambian con la carrera: es
        una planta de parametros variables, cosa que el modelo de la memoria
        (volumen fijo V_t) no recoge. El minimo esta cerca del centro.
        """
        x = 0.5 * self.L_carrera if x is None else x
        V_A = self.V_A0 + self.A_A * x
        V_B = self.V_B0 + self.A_B * (self.L_carrera - x)
        k_hid = self.beta * (self.A_A**2 / V_A + self.A_B**2 / V_B)
        return math.sqrt(k_hid / self.M_t)

    def presiones_regimen(self, extiende: bool) -> tuple:
        """Presiones de regimen en vacio [Pa], resolviendo las dos condiciones
        que se cumplen cuando la velocidad es constante y sin carga:

            continuidad : Q_A = A_A*v   y   Q_B = A_B*v
            fuerza      : P_A*A_A = P_B*A_B      (aceleracion nula, sin carga)

        Combinadas con la ecuacion de orificio dan, llamando rA = A_A/A_B:
            extiende:  (Ps - P_A) = rA^2 * (P_B - Pt)
            retrae  :  (P_A - Pt) = rA^2 * (Ps - P_B)
        Se resuelve por biseccion sobre P_B (la ecuacion es monotona).
        """
        rA = self.rel_areas
        Ps, Pt = self.P_s, self.P_t

        def resid(PB: float) -> float:
            PA = PB / rA
            if extiende:
                return (Ps - PA) - rA**2 * (PB - Pt)
            return (PA - Pt) - rA**2 * (Ps - PB)

        lo, hi = 0.0, Ps * 5
        for _ in range(200):
            mid = 0.5 * (lo + hi)
            if resid(lo) * resid(mid) <= 0.0:
                hi = mid
            else:
                lo = mid
        PB = 0.5 * (lo + hi)
        return PB / rA, PB

    def K_vel(self, extiende: bool = True) -> float:
        """Ganancia de velocidad en vacio [m/s por voltio], EN REGIMEN.

        Distinta en cada sentido, pero NO en la relacion A_A/A_B que uno
        esperaria a primera vista: al retraer, la camara GRANDE tiene que
        evacuar A_A*v por el orificio de retorno, y ese es el cuello de botella.
        El resultado es que RETRAER es mas LENTO que extender (~0.77x), no mas
        rapido. Es un ejemplo de por que no basta con mirar las areas.
        """
        P_A, P_B = self.presiones_regimen(extiende)
        dP = (self.P_s - P_A) if extiende else (P_A - self.P_t)
        Q = self.K_q * math.sqrt(max(dP, 0.0) / self.dP_nom)   # a xs = 1
        xs_por_V = self.K_amp / self.i_nom
        return xs_por_V * Q / self.A_A


# ============================================================================
# 2. LA PLANTA
# ============================================================================
class PlantaHidraulica:
    """Integra el modelo con RK4 a paso fijo, con ZOH sobre el comando.

    Estado (6 variables, todo en SI):
        xs   posicion normalizada del carrete   [-]
        xsd  su derivada                        [1/s]
        P_A  presion camara sin vastago         [Pa]
        P_B  presion camara con vastago         [Pa]
        v    velocidad del piston               [m/s]
        x    posicion del piston                [m]  (0 = totalmente retraido)
    """

    def __init__(self, p: ParamsPlanta | None = None, x0: float = 0.075):
        self.p = p or ParamsPlanta()
        self.reset(x0)

    def reset(self, x0: float = 0.075) -> None:
        p = self.p
        self.xs = 0.0
        self.xsd = 0.0
        # Reposo: ambas camaras a presion de tanque (como los manometros a 0
        # con la HPU apagada, verificado por el laboratorio el 2026-08-12).
        self.P_A = p.P_t
        self.P_B = p.P_t
        self.v = 0.0
        self.x = float(np.clip(x0, 0.0, p.L_carrera))
        self._hist_ref = 0.0
        self._rota = False

    # ---------------------------------------------------------------- carrete
    def _carrete_efectivo(self, xs: float) -> float:
        """No linealidades estaticas del carrete: histeresis (con memoria),
        offset de null y solape (zona muerta)."""
        p = self.p
        if p.histeresis > 0.0:
            h = p.histeresis / 2.0
            if xs > self._hist_ref + h:
                self._hist_ref = xs - h
            elif xs < self._hist_ref - h:
                self._hist_ref = xs + h
            xs = self._hist_ref

        xs = xs + p.null_off

        if p.solape > 0.0:
            if abs(xs) <= p.solape:
                xs = 0.0
            else:
                xs = math.copysign(abs(xs) - p.solape, xs)
        return xs

    # ------------------------------------------------------------------ caudal
    def _caudales(self, xs: float, P_A: float, P_B: float) -> tuple:
        """Caudales de la servovalvula de 4 vias.

        Devuelve (Q_A, Q_B) con el convenio:
            Q_A = caudal que ENTRA en la camara A
            Q_B = caudal que SALE de la camara B
        Ambos con el mismo signo que xs, de modo que xs>0 extiende el vastago.
        """
        p = self.p
        if xs == 0.0:
            return 0.0, 0.0
        s = math.copysign(1.0, xs)
        a = abs(xs)

        if s > 0:                       # P->A , B->T  (extiende)
            dA = max(p.P_s - P_A, 0.0)
            dB = max(P_B - p.P_t, 0.0)
        else:                           # P->B , A->T  (retrae)
            dA = max(P_A - p.P_t, 0.0)
            dB = max(p.P_s - P_B, 0.0)

        Q_A = s * p.K_q * a * math.sqrt(dA / p.dP_nom)
        Q_B = s * p.K_q * a * math.sqrt(dB / p.dP_nom)

        # Saturacion por el caudal disponible de la bomba
        if abs(Q_A) > p.Q_bomba:
            Q_A = math.copysign(p.Q_bomba, Q_A)
        if abs(Q_B) > p.Q_bomba:
            Q_B = math.copysign(p.Q_bomba, Q_B)
        return Q_A, Q_B

    # ------------------------------------------------------------------- carga
    def _fuerza_carga(self, x: float) -> float:
        """Fuerza que la probeta opone [N]: contacto elastico unilateral con
        rotura. Cuando F supera F_rotura la rigidez desaparece PARA SIEMPRE —
        y con ella la validez de cualquier modelo entrenado antes (§8)."""
        p = self.p
        if p.k_probeta <= 0.0 or self._rota:
            return 0.0
        pen = x - p.x_contacto
        if pen <= 0.0:
            return 0.0
        F = p.k_probeta * pen
        if F >= p.F_rotura:
            self._rota = True
            return 0.0
        return F

    # -------------------------------------------------------------- derivadas
    def _deriv(self, s: tuple, u_norm: float) -> tuple:
        p = self.p
        xs, xsd, P_A, P_B, v, x = s

        xsdd = p.omega_sv**2 * (u_norm - xs) - 2 * p.delta_sv * p.omega_sv * xsd

        xs_ef = self._carrete_efectivo(xs)
        Q_A, Q_B = self._caudales(xs_ef, P_A, P_B)

        # Volumenes variables con la carrera -> planta de parametros variables
        V_A = p.V_A0 + p.A_A * x
        V_B = p.V_B0 + p.A_B * (p.L_carrera - x)
        fuga = p.K_fuga * (P_A - P_B)

        dP_A = (p.beta / V_A) * (Q_A - p.A_A * v - fuga)
        dP_B = (p.beta / V_B) * (p.A_B * v - Q_B + fuga)

        F_net = P_A * p.A_A - P_B * p.A_B - p.B_p * v - self._fuerza_carga(x)
        if p.vertical:
            # Peso del conjunto movil, siempre en el sentido POSITIVO (hacia
            # afuera y hacia abajo). Con el carrete cerrado esta es la fuerza
            # que hace descender el vastago por la fuga residual del null.
            F_net += (p.M_t + p.m_colgante) * 9.81

        # Friccion seca de los sellos (stick-slip): la no linealidad que
        # estropea los ensayos a 0.1 mm/min, donde el vastago avanza a tirones.
        if p.F_stick > 0.0 or p.F_coul > 0.0:
            if abs(v) < p.v_stick:
                if abs(F_net) <= p.F_stick:
                    F_net = 0.0
                else:
                    F_net -= math.copysign(p.F_stick, F_net)
            else:
                F_net -= math.copysign(p.F_coul, v)

        return (xsd, xsdd, dP_A, dP_B, F_net / p.M_t, v)

    # ------------------------------------------------------------------- paso
    def _rk4(self, u_norm: float, h: float) -> None:
        s = (self.xs, self.xsd, self.P_A, self.P_B, self.v, self.x)
        k1 = self._deriv(s, u_norm)
        s2 = tuple(a + 0.5 * h * b for a, b in zip(s, k1))
        k2 = self._deriv(s2, u_norm)
        s3 = tuple(a + 0.5 * h * b for a, b in zip(s, k2))
        k3 = self._deriv(s3, u_norm)
        s4 = tuple(a + h * b for a, b in zip(s, k3))
        k4 = self._deriv(s4, u_norm)
        ns = tuple(a + (h / 6.0) * (b + 2 * c + 2 * d + e)
                   for a, b, c, d, e in zip(s, k1, k2, k3, k4))
        self.xs, self.xsd, self.P_A, self.P_B, self.v, self.x = ns

        # Las presiones no pueden bajar de tanque ni dispararse sin limite
        self.P_A = min(max(self.P_A, 0.0), 500e5)
        self.P_B = min(max(self.P_B, 0.0), 500e5)

        # Topes mecanicos: choque inelastico
        if self.x <= 0.0:
            self.x, self.v = 0.0, max(0.0, self.v)
        elif self.x >= self.p.L_carrera:
            self.x, self.v = self.p.L_carrera, min(0.0, self.v)

    # --------------------------------------------------------------- simular
    def simula(self, u_V, Ts: float, x0: float | None = None) -> dict:
        """Simula con `u_V` (VOLTIOS) aplicado con ZOH cada `Ts`.

        Devuelve arrays muestreados a Ts, en las MISMAS unidades y con los
        MISMOS nombres que produce la captura real (`daq.py`), para que el
        pipeline de identificacion no distinga entre simulacion y equipo:
            t [s] · u [V] · posicion [mm] · v [mm/s] · fuerza [kN]
            presion_A [bar] · presion_B [bar] · xs [-]
        """
        p = self.p
        if x0 is not None:
            self.reset(x0)

        u_V = np.asarray(u_V, dtype=float)
        n = len(u_V)
        nsub = max(1, int(round(Ts / p.dt_int)))
        h = Ts / nsub

        t = np.arange(n) * Ts
        x = np.empty(n); v = np.empty(n); F = np.empty(n)
        pA = np.empty(n); pB = np.empty(n); xs = np.empty(n)

        for k in range(n):
            x[k], v[k] = self.x, self.v
            F[k] = self._fuerza_carga(self.x)
            pA[k], pB[k], xs[k] = self.P_A, self.P_B, self.xs

            uv = float(np.clip(u_V[k], -p.u_max, p.u_max))
            u_norm = (uv * p.K_amp) / p.i_nom
            for _ in range(nsub):
                self._rk4(u_norm, h)

        return {
            "t": t,
            "u": np.clip(u_V, -p.u_max, p.u_max),
            "posicion": x * 1e3,
            "v": v * 1e3,
            "fuerza": F * 1e-3,
            "presion_A": pA * 1e-5,
            "presion_B": pB * 1e-5,
            "xs": xs,
        }


# ============================================================================
# 3. DEMOS / VERIFICACION
# ============================================================================
def _check(p: ParamsPlanta) -> None:
    print("=" * 70)
    print("PARAMETROS DERIVADOS DEL MODELO DE DOS CAMARAS")
    print("=" * 70)
    print(f"  A_A (sin vastago) = {p.A_A*1e4:8.2f} cm2")
    print(f"  A_B (con vastago) = {p.A_B*1e4:8.2f} cm2   <- la 'A_p' de la memoria")
    print(f"  relacion A_A/A_B  = {p.rel_areas:8.3f}")
    print(f"  carrera util      = {p.L_carrera*1e3:8.1f} mm")
    print()
    ke, kr = p.K_vel(True), p.K_vel(False)
    PAe, PBe = p.presiones_regimen(True)
    PAr, PBr = p.presiones_regimen(False)
    print("  ASIMETRIA — ganancia de velocidad en vacio y presiones de regimen:")
    print(f"    extendiendo (P->A): {ke*1e3:7.4f} mm/s por V   "
          f"P_A={PAe/1e5:6.2f} bar  P_B={PBe/1e5:6.2f} bar")
    print(f"    retrayendo  (P->B): {kr*1e3:7.4f} mm/s por V   "
          f"P_A={PAr/1e5:6.2f} bar  P_B={PBr/1e5:6.2f} bar")
    print(f"    -> relacion retraer/extender = {kr/ke:.3f}")
    print("       CONTRAINTUITIVO: no es A_A/A_B = 1.64 'a favor' de retraer.")
    print("       Al retraer, la camara GRANDE debe evacuar A_A*v por el orificio")
    print("       de retorno: ese es el cuello de botella, y retraer sale MAS")
    print("       LENTO. En 2017 tambien se midio el sentido negativo mas lento")
    print("       (0.0114 vs 0.0147, relacion 0.78) — coincide muy bien (§5.1).")
    print()
    print("  INTENSIFICACION DE PRESION (retencion/frenado):")
    print(f"    P_B = P_A * A_A/A_B -> con {p.P_s/1e5:.0f} bar en A, "
          f"{p.P_s/1e5*p.rel_areas:.0f} bar en B")
    print("    Por eso el transductor de B es 0-400 bar y el de A 0-100 bar.")
    print()
    print("  FRECUENCIA HIDRAULICA (depende de la carrera: parametros variables)")
    print(f"    {'x [mm]':>8s} {'w_h [rad/s]':>13s} {'f_h [Hz]':>10s}")
    for frac in (0.05, 0.25, 0.5, 0.75, 0.95):
        xx = frac * p.L_carrera
        w = p.omega_h(xx)
        print(f"    {xx*1e3:8.1f} {w:13.1f} {w/2/math.pi:10.1f}")
    print("    (la memoria daba un unico valor, 1414.7 rad/s = 225 Hz, porque")
    print("     suponia volumen fijo V_t y cilindro simetrico)")


def _demo_escalon(p: ParamsPlanta) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    Ts = 0.001
    n = int(8.0 / Ts)
    u = np.zeros(n)
    u[int(0.5 / Ts):int(3.5 / Ts)] = 1.0
    u[int(4.0 / Ts):int(7.0 / Ts)] = -1.0

    r = PlantaHidraulica(p, x0=0.075).simula(u, Ts)

    fig, ax = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    ax[0].plot(r["t"], r["u"], color="tab:orange"); ax[0].set_ylabel("comando [V]")
    ax[1].plot(r["t"], r["posicion"], color="tab:cyan"); ax[1].set_ylabel("posicion [mm]")
    ax[2].plot(r["t"], r["v"], color="tab:green"); ax[2].set_ylabel("velocidad [mm/s]")
    ax[3].plot(r["t"], r["presion_A"], label="P_A (sin vastago)", color="tab:red")
    ax[3].plot(r["t"], r["presion_B"], label="P_B (con vastago)", color="tab:purple")
    ax[3].set_ylabel("presion [bar]"); ax[3].legend(fontsize=8)
    ax[3].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    ax[0].set_title("Escalon +1 V / -1 V — la posicion RAMPA (planta integradora)")
    fig.tight_layout()
    fig.savefig("results/sim_escalon.png", dpi=120)
    print("[ok] results/sim_escalon.png")

    ext = (r["t"] > 2.0) & (r["t"] < 3.4)
    ret = (r["t"] > 5.5) & (r["t"] < 6.9)
    print(f"  velocidad extendiendo (+1 V): {r['v'][ext].mean():+7.4f} mm/s")
    print(f"  velocidad retrayendo  (-1 V): {r['v'][ret].mean():+7.4f} mm/s")
    rel = abs(r['v'][ret].mean() / r['v'][ext].mean())
    print(f"  relacion |retraer/extender| = {rel:.3f}"
          f"   (regimen analitico: {p.K_vel(False)/p.K_vel(True):.3f})")


def _demo_asimetria(p: ParamsPlanta) -> None:
    """Barrido de comando en ambos sentidos: la curva estatica u -> velocidad."""
    Ts = 0.002
    print("\n  CURVA ESTATICA comando -> velocidad (ambos sentidos)")
    print(f"  {'u [V]':>8s} {'v extend [mm/s]':>17s} {'v retrae [mm/s]':>17s} {'rel':>7s}")
    for uu in (0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0):
        vs = []
        for sg in (+1.0, -1.0):
            n = int(3.0 / Ts)
            r = PlantaHidraulica(p, x0=0.075).simula(np.full(n, sg * uu), Ts)
            seg = r["t"] > 1.0
            vs.append(r["v"][seg].mean())
        rel = abs(vs[1] / vs[0]) if vs[0] else float("nan")
        print(f"  {uu:8.2f} {vs[0]:17.4f} {vs[1]:17.4f} {rel:7.3f}")
    print(f"  Relacion de regimen esperada: {p.K_vel(False)/p.K_vel(True):.3f}"
          " (NO A_A/A_B = 1.641; ver `presiones_regimen`).")
    print("  Se acerca a ella segun sube la amplitud: a comando MUY pequenio la")
    print("  relacion tiende a ~0.89 porque el transitorio pesa mas que el regimen")
    print("  en la ventana de medida, y a 10 V vuelve a subir porque satura el")
    print("  caudal de la bomba. En medio (1-5 V) sale 0.77-0.78, que es el valor")
    print("  analitico y el que se midio en 2017 (0.78).")


def _demo_probeta(p: ParamsPlanta) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    p.k_probeta = 2.0e8
    p.x_contacto = 0.0755
    p.F_rotura = 180e3

    Ts = 0.005
    n = int(40.0 / Ts)
    r = PlantaHidraulica(p, x0=0.075).simula(np.full(n, 0.08), Ts)

    fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    ax[0].plot(r["t"], r["posicion"], color="tab:cyan"); ax[0].set_ylabel("posicion [mm]")
    ax[1].plot(r["t"], r["fuerza"], color="tab:red"); ax[1].set_ylabel("fuerza [kN]")
    ax[2].plot(r["t"], r["presion_A"], label="P_A", color="tab:red")
    ax[2].plot(r["t"], r["presion_B"], label="P_B", color="tab:purple")
    ax[2].set_ylabel("presion [bar]"); ax[2].legend(fontsize=8)
    ax[2].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.3)
    ax[0].set_title("Contacto con probeta y rotura")
    fig.tight_layout()
    fig.savefig("results/sim_probeta.png", dpi=120)
    print("[ok] results/sim_probeta.png")
    print(f"  fuerza maxima: {r['fuerza'].max():.1f} kN"
          f"   P_A maxima: {r['presion_A'].max():.1f} bar")


def main() -> None:
    ap = argparse.ArgumentParser(description="Simulador del servohidraulico UPH 50")
    ap.add_argument("--demo", choices=["escalon", "asimetria", "probeta"])
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--dt-int", type=float, default=None)
    args = ap.parse_args()

    p = ParamsPlanta()
    if args.dt_int:
        p.dt_int = args.dt_int

    if args.check or not args.demo:
        _check(p)
    if args.demo == "escalon":
        _demo_escalon(p)
    elif args.demo == "asimetria":
        _demo_asimetria(p)
    elif args.demo == "probeta":
        _demo_probeta(p)


if __name__ == "__main__":
    main()
