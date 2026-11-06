# CLAUDE.md — Prensa servohidráulica Fluidtek UPH 50 (LEDI-PUCP)

> Documento maestro del proyecto: hardware, modelo, resultados vigentes, estado y
> pendientes. Reemplaza a `PENDIENTES.md` y `docs/protocolo_fase0.md` (eliminados el
> 2026-11-02). El detalle histórico de las mediciones de agosto 2026 (procedimientos,
> datos crudos, NARX/DBP) está en `.claude/skills/resultados-medidos/SKILL.md`; ahí los
> números que dependen de la geometría del cilindro usan la de la memoria (160/100 mm),
> hoy en revisión (§2.3).
>
> **Orden de autoridad de los datos:** medido en el equipo > placa/catálogo > memoria de
> Fluidtek 2017 (`docs/`, desactualizada).

---

## 1. Objetivo

Prensa de ensayo de materiales (losas, vigas de concreto) del Laboratorio de Estructuras
PUCP. Hoy se controla con LabVIEW (PID de posición) sobre un NI cDAQ-9184 por Ethernet.

Líneas de trabajo:
1. **Software de control LabVIEW** (`labview_control_software/`): v15 en uso.
2. **Modelo físico en Simulink** (`simulink/`), Jelali & Kroll 2003.
3. **Identificación y control neuronal** (NARX → neurocontrolador → comparación con el PID).
4. **Diagrama eléctrico** del tablero en CADe_SIMU (`diagrama_electrico/`).

Modos de ensayo: posición (aproximación), fuerza (precarga), rampa de fuerza (kN/min) y
velocidad (losa 1.5 mm/min, viga 0.1 mm/min).

---

## 2. Hardware

### 2.1 Unidad hidráulica (placa UPH 50, Fluidtek 02/2015)
| | |
|---|---|
| Tanque | 50 L, indicador de nivel/temperatura HL91 |
| Bomba | engranajes Casappa PLP10-1, **1.7 L/min**; Pmáx 150 bar, Ptrab **100 bar** |
| Motor M1 | WEG W22, **1.5 kW (2 HP)**, 1730 rpm, 12 bornes; conectado **ΔΔ a 220 V** (6.08 A) |
| Circuito | filtro de succión y de retorno con bypass, limitadora, antirretorno, manómetro, acumulador de vejiga (volumen y precarga **desconocidos**), filtros en línea con señal de saturación |
| Aceite | RANDO HD |

La memoria calculó 1 HP; el motor instalado es de 2 HP. La placa dice 380 V, pero los
guardamotores (6–10 A, ajustados a ~6.2 A) solo cuadran con 220 V.

### 2.2 Servoválvula Moog G761-3001B H04JOFM4VPL
4 L/min a 35 bar por arista · carrete **zero lap** · ±40 mA (paralelo) · **−3 dB a 120 Hz**
(no los 150 Hz de la memoria) · histéresis ≤ 3 % · deriva de null ≤ 2 % por 55 °C
(≈ 0.20 V de comando: **el null deriva con la temperatura del aceite**).
Mando: NI 9263 ±10 V → convertidor Delta VC2124 → **±40 mA** (`K_amp = 0.004 A/V`).

### 2.3 Actuador — diámetro medido por ensayo de fuerza (2026-11-04)
| | Memoria | Medido |
|---|---|---|
| Émbolo D | 160 mm | **160 mm: ajuste 161.5 mm (±3 mm), compatible con 160** (ensayo contra la celda); tubo exterior **190 mm (medido)** |
| Vástago d | 100 mm | **80 mm (medido 2026-10-31)** |
| A_A / A_B | 201.1 / 122.5 cm² | **201.1 / 150.8 cm²** |
| Carrera | — | **física 300 mm**; útil 0–150 mm (límite de la VI y del Temposonic) |
| Montaje | vertical, vástago hacia abajo; masa móvil 150 kg + celda 28 kg | |

**Cómo se determinó D (2026-11-04).** Serie de escalones de fuerza 5–10–20–30–40–30–20–10–5 kN
contra una viga de acero con `Prensa.escalonesFuerza` (control PI de fuerza, UPH en marcha,
catman Easy como segunda referencia), registrando celda, Temposonic y los dos transductores
de presión alineados. Modelo estático `F = A_A·(P_A − P_B) + A_r·P_B + c` con `A_r = A_A − A_B =
50.27 cm²` (vástago de 80 mm medido) y las escalas de hoja de datos (A: 12.5 bar/V, TR44.2 0–100
bar; B: 50 bar/V, TR49.2 0–400 bar; ceros 1.9826 / 1.9682 V):
- **D = 161.5 mm, offset c = −0.03 kN, rms 0.25 kN** con los 9 escalones (subida 160.8 mm,
  bajada 163.1 mm; la histéresis de fricción es de ~0.3 kN). La fuerza hidráulica con
  160/80 mm coincide con la celda a ≤ 0.8 kN en todo el rango (p. ej. 40 kN: celda 39.6,
  hidráulica 39.3).
- **El offset valida las escalas:** con las escalas de agosto/manómetros c sale +15 kN; a lo largo
  del valle c ≈ 0 la incertidumbre de escala (±8 %) movería D entre 156 y 168 mm.
- A 40 kN las presiones son solo P_A ≈ 46 y P_B ≈ 35 bar. Rigidez del apoyo ~14 kN/mm
  (celda frente a Temposonic).
- Tres indicios independientes coinciden: ventana de calibración (P_B/P_A = 1.35), equilibrio
  del émbolo libre (P_B/P_A = 1.38 → D ≈ 153–165 mm) y esta serie. **Los 125 mm que se habían
  inferido quedan descartados** (partían de presiones medidas con los cables invertidos).

**Pendiente:** repetir la serie con la UPH a otra temperatura y, si se puede, con otra
viga/posición; confirmar con la placa del cilindro. `params_prensa.m`, `PID.c` y el modelo
Simulink ya se actualizaron a 160/80 el 2026-11-04 (§3); queda repetir la prueba de
saturación con el cableado definitivo. Fuerza hidráulica:
`F = P_A·A_A − P_B·A_B` (cilindro asimétrico).

**Episodio de sobrecarga (informado por el laboratorio, 2026-11-02).** El equipo llegó
alguna vez a ~170–180 kN, empezó a saturar y **falló una soldadura del manifold**. Con
160/80 mm eso son ~85–90 bar de P_A (con P_B baja): **dentro de la presión de trabajo de 100
bar**, y a 100 bar la fuerza máxima es **~201 kN** (cilindro de clase 200 kN, coherente con la
celda de 200 kN). Que la soldadura fallara a 85–90 bar sugiere que el manifold no está
dimensionado para la presión de trabajo, o que hubo picos (cierre brusco, limitadora, golpe
al tope) por encima de lo que registra la VI. El límite de 170 kN de `PID.c` (v11) viene de
ese episodio y **no es un margen seguro**. Fecha, presiones registradas y tarado real de la
limitadora: desconocidos (§5).

### 2.4 Instrumentación y E/S (cDAQ-9184, alias `cDAQ9184-1ADC24C`)
| Canal | Señal | Escala |
|---|---|---|
| `ai0` (NI 9222) | posición, Temposonic | 0–10 V → 0–150 mm (15 mm/V) |
| `ai1` | celda de carga (convertidor multirango) | 0–10 V → 0–200 kN (20 kN/V) |
| `ai2` | presión cámara A (sin vástago), **FOX TR44.2 0–100 bar** | 4–20 mA / 500 Ω; hoja de datos 12.5 bar/V, cero 2 V. *Escala de agosto (32.293 bar/V, cero 1.9697 V) obsoleta: ver abajo* |
| `ai3` | presión cámara B (con vástago), **FOX TR49.2 0–400 bar** | hoja de datos 50 bar/V, cero 2 V. *Escala de agosto (18.163 bar/V, cero 1.9801 V) obsoleta* |
| `ao0` (NI 9263) | comando servoválvula | ±10 V → ±40 mA |

**Asignación DEFINITIVA de presiones (fijada por el laboratorio el 2026-11-04):**
**cámara A (sin vástago) = FOX TR44.2, 0–100 bar, 4–20 mA a 2 hilos → `ai2`; cámara B
(con vástago) = FOX TR49.2, 0–400 bar, 4–20 mA a 2 hilos → `ai3`.** (El laboratorio había
leído «TR45.2 0–200 bar» por error el 2026-11-04 y lo corrigió el mismo día tras la
calibración: la etiqueta es TR49.2 0–400 bar, como se reportó en origen.) Esta posición no
se vuelve a cambiar.

⚠ **Antes del 2026-11-04 los cables estaban INVERTIDOS** (el sensor de A entraba por `ai3` y
el de B por `ai2`), mientras que el software y la calibración de agosto asumían `ai2` = A y
`ai3` = B. Consecuencias: (1) las escalas de agosto (32.293 / 18.163 bar/V, cero 1.9697 /
1.9801 V) se obtuvieron asignando los manómetros 30/50 bar al canal equivocado y **no
valen**; (2) toda captura anterior con columnas `presion_A`/`presion_B` (CSV de agosto,
datos NARX, log `Lg[12..14]` de la VI, informe de 2017) tiene las presiones permutadas y
con escala errónea; solo posición, celda y comando no se ven afectados; (3) el cierre del
equilibrio «P_B/P_A = A_A/A_B» de agosto no prueba nada sobre D. Hasta recalibrar, usar la
hoja de datos (A 12.5 y B 50 bar/V, cero 2 V) y verificar contra los manómetros.

**2026-11-04, UPH apagada, cables reconectados con la asignación definitiva:** ceros
medidos (5 s a 1 kHz, sin deriva en 30 s, σ ≈ 6 mV): **`ai2` (A, TR44.2) = 1.9826 V,
`ai3` (B, TR49.2) = 1.9682 V**. Los ceros de agosto (1.9697 / 1.9801 V) coinciden con
estos con los canales intercambiados: el offset de 4 mA es del sensor y viajó con él, lo
que confirma la inversión de cableado. `params_prensa.m` lleva ahora estos ceros y las
escalas de hoja de datos (12.5 / 50 bar/V) como **PROVISIONALES** hasta fijar la escala
con la celda (§5).

**Punto de calibración 2026-11-04 (primera vez que `Prensa.m` escribe AO/DO en hardware: sin
incidentes).** UPH en marcha 3 min, AO en U_NULL = −0.3697 V, vástago ~68 mm, ventana de
30 s estable: **V_A = 4.6925 ± 0.0029 V, V_B = 2.8853 ± 0.0012 V** (ΔV sobre el cero:
2.710 V y 0.917 V). Manómetros (asignados por la tubería, leídos a ojo): **A = 30 bar,
B ≈ 50 bar**. Escalas implícitas: **A 11.07 bar/V** (hoja de datos 12.5: el sensor marca
+13 % sobre el manómetro) y **B 54.5 bar/V** (hoja de datos del TR49.2 0–400 bar: 50
bar/V, el sensor marca 8 % por debajo del manómetro). Con la hoja de datos: P_A = 33.9 bar
y P_B = 45.9 bar, relación 1.35. La UPH vuelve a parar con DI7 = 0.

⚠ **Los manómetros no deciden D.** Equilibrio estático A_A/A_B = P_B/P_A: con los
manómetros (50/30 = 1.67) sale D ≈ 126 mm, pero ±1.5 bar de lectura lo mueven entre 120 y
135 mm; con los sensores (1.35) sale D ≈ 157 mm, cerca de los 160 mm de la memoria. Hace
falta la prueba de fuerza contra la celda (§2.3, §5).

**Digitales — confirmadas por etiquetas de cableado, memoria y VI (2026-10-31).**
Cada `DOn` del NI 9472 activa el relé `K(n+1)` (Phoenix 2961192, 24 VDC).

| Línea | Función | Elemento |
|---|---|---|
| DI0 | seta de emergencia | contacto NC 21-22 · 1 = seguro |
| DI1 | nivel de aceite | 1 = OK |
| DI2 | saturación del filtro en línea | 1 = limpio |
| DI5 | guardamotor GM1 | aux NC 21-22 · **1 = disparado o apagado** |
| DI6 | guardamotor GM2 | aux NC 21-22 |
| DI7 | confirmación de marcha | KM1 aux 13-14 · 1 = motor activo |
| DO0 | arranque bomba | K1 → bobina KM1 → M1 |
| DO1 | circuito M2 (reserva, no hay segundo motor en la UPH) | K2 → KV1 |
| DO5 | **alimentación de los transductores de presión** | K6 (bornes 77/78); alto siempre |
| DO6 | alimenta algo de la hoja 10 (borne 82) | K7 · sin identificar |
| DO2–4, DO7 | sin identificar (K3–K5, K8) | |

⚠ **La seta solo llega a DI0**: K1 alimenta directo la bobina de KM1, así que la parada de
emergencia depende de que la VI lea DI0 y baje DO0. Verificar con la VI detenida y, si no
corta, cablear la seta en serie con el mando de KM1 (electricista).

### 2.5 Tablero
QA1 (Siemens 3VT1, 32 A) → repartidor RB → GM1 (GV2ME14) → KM1 (LC1D09BL, bobina 24 VDC)
→ M1 · GM2 → KV1 → (M2, reserva) · CB1–CB3 (230 V) · PS1 Allen-Bradley 24 VDC 5 A ·
CB4–CB6 (24 VDC) · fusible F3 → Vsup del NI 9472. El cDAQ solo está disponible después de
encender QA1 (la v15 reintenta la reserva al arrancar).

---

## 3. Modelo Simulink (`simulink/`)

Jelali & Kroll 2003: servoválvula de 2.º orden, orificios con underlap y offset de null
(ec. 4.5–4.6), cámaras con E'(p) de Lee y volúmenes variables (4.51–4.52), suministro
dinámico con bomba + limitadora + acumulador (4.2.4), fricción de Stribeck (4.60), peso,
topes y sensores con escala, ruido, ADC y latencia.

- `params_prensa.m`: todos los parámetros, cada uno con su origen (MEDIDO / PLACA /
  CATÁLOGO / AJUSTADO / SUPUESTO). **Actualizado el 2026-11-04 a la geometría medida:
  160/80 mm (§2.3), Qp 3.18 L/min, kQ 0.977, x0 3.09 %, xu 0 (zero-lap)**, limitadora blanda
  (`Ps0` 43 bar, `dP_rv` 57 bar, `Ps_ini` 97 bar, 2026-11-05), L_fis 0.300 m, presiones
  iniciales 34.7/46.9 bar, escalas de presión de hoja de datos (§2.4).
- `prensa_fluidtek.slx`: generado por `construir_modelo_prensa.m`; validación con
  `validar_modelo_prensa.m` y `sim_satura.m` (estos scripts y los resultados están fuera
  del repositorio, en el respaldo de `D:\fluidteck_project`).

**Validación con los parámetros de 2026-11-05** (`Prensa.simula`, sin sobrescrituras; los de
2026-11-04, con limitadora rígida, daban RMS 0.0138 pero no reproducían la retracción):
curva comando→velocidad (ley medida de §4) RMS **0.0162 mm/s** (antes 0.042 con 125/80);
a comando 0 V el vástago se extiende a +0.138 mm/s (medido +0.14) y en el null a −0.017;
presiones en reposo a U_NULL **36.0 / 49.6 bar** (medido 34.7 / 46.9; antes 38.7 / 69.1);
saturación a +10 V: la velocidad decae de **3.91 a 2.64 mm/s** (medido 4.0 → 2.64), lo que
exige 3.18 L/min con A_A = 201 cm²: otra confirmación de D = 160 mm.
**Suministro hidráulico: resuelto el 2026-11-05 (limitadora blanda).** Pruebas del 2026-11-04 y
2026-11-05 con el cableado definitivo, 160/80, sin probeta, y con el manómetro de la UPH
leído por el laboratorio (`Prensa.satura` y retracciones a varios comandos;
`results/saturacion.csv`):

| Maniobra | v [mm/s] | P_A / P_B cilindros [bar] | Manómetro de la UPH [bar] |
|---|---|---|---|
| Reposo, bomba en marcha (AO en el null) | 0 | 34.7 / 46 | **96–98** |
| Extendiendo +10 V (baja, a favor de la gravedad) | 4.1 → 2.56 | 26 → 10 / 34 → 13 | 95 → **~40** (ayer 25–40) |
| Retrayendo −10 V (sube, contra la gravedad) | 2.9 (ayer 2.63) | 30 / 41 (ayer 24.5 / 33.6) | **55** (ayer 50) |
| Retrayendo −5 V | 1.77 | 46 / 62.5 | **85** |
| Retrayendo −3 V | 1.00 | 47 / 64 | **90** |
| Retrayendo −1 V | 0.26 | 42 / 57 | **90** |

- **Lo que explica todo: la limitadora es «blanda»** (gran sobrepresión): empieza a abrir hacia
  **43 bar** y solo deja pasar el caudal completo de la bomba (3.18 L/min) hacia **~100 bar**.
  Con poco caudal de consumo (−1, −3, −5 V) el suministro queda cerca de 85–98 bar; con la
  válvula abierta del todo (−10 V) el consumo (2.7 L/min) lo hunde a 55–60; en la extensión
  el consumo (3.1 L/min) lo deja en ~40. Con la limitadora rígida a 100 bar que tenía el
  modelo la retracción a −10 V salía a 3.5 mm/s y 52/71 bar (medido 2.9 y 30/41).
- **Descartado:** un regulador fijo a 50 bar (las cámaras llegan a 57–67 bar a −5/−3/−1 V),
  una limitadora tarada a ~48 bar (en reposo hay 96–98) y la fuga lineal de la bomba (misma
  curva, peor ajuste). **Los 50 bar «exactos» de la retracción de 2026-11-04 eran el
  suministro a −10 V** (hoy 55): depende del consumo, no es un tarado.
- **Bomba:** meseta de la extensión = **3.08 L/min** (A_A·v) ≈ 3.18 L/min medido; el área de
  201 cm² y D = 160 mm se confirman. Las presiones de las cámaras a −1/−3/−5 V suben porque el
  carrete casi cerrado exige más caída de presión para el mismo caudal.
- **Modelo (`params_prensa.m`, 2026-11-05):** `Ps0` = 43 bar (inicio de apertura), `dP_rv` = 57
  bar, `Ps_ini` = 97 bar, con `kQ` = 0.977 y `x0` = 3.09 % reajustados. Retracción
  modelo/medido: −10 V 2.71/2.90 mm/s, cilindros 29/41 vs 30/41 bar, suministro 56 vs 55;
  −5 V 1.55/1.77 mm/s; −3 V 0.95/1.00; −1 V 0.26/0.26 (errores de 10–20 % en las presiones
  de −5 y −1 V); extensión 3.91 → 2.64 mm/s con suministro 88 → 34 bar (medido ~40).
- **Consecuencias:** (1) **limitadora ≈ 96–100 bar**: la fuerza máxima estática de extensión
  es ≈ 190–200 kN, y el episodio de 170–180 kN fue con presión de trabajo normal (§2.3);
  (2) velocidad máxima sostenida ≈ 2.6 mm/s al extender y ≈ 2.9 al retraer (4.1 mm/s solo
  mientras dura el acumulador), útil para dimensionar rampas del controlador; (3) la
  fuerza que el equipo puede sostener a baja velocidad es mayor que la que puede aplicar a
  alta, porque el suministro se hunde cuando el consumo se acerca al caudal de la bomba.
Antes de este cambio el modelo se había validado con 125/80 (RMS 0.039 / 0.108 mm/s).

---

## 4. Resultados vigentes (detalle en la skill `resultados-medidos`)

- **Comando → velocidad** (sin probeta): `u>0: v = 0.4459·u + 0.1139` ·
  `u<0: v = 0.3779·u + 0.1397` mm/s. Sin zona muerta (zero lap).
- **Null ≈ −0.37 V**; a 0 V el vástago cae 8.4 mm/min (peso + fuga). El null se movió
  71 mV en 10 min con la temperatura: **el lazo necesita acción integral** y las
  comparaciones deben hacerse en la misma sesión, anotando la temperatura del aceite.
- Consignas normadas: losa 1.5 mm/min → −0.304 V; viga 0.1 mm/min → −0.365 V
  (a 4.6 mV del cruce por cero).
- **Esperar 3 min tras arrancar la bomba** antes de medir (transitorio de presión).
- **Latencia del lazo** cDAQ por Ethernet: 5.46 ms mediana (máx 9.9 ms) → **Ts de
  control 20 ms**. Si sale ~200 ms, la tarea AI no se arrancó explícitamente.
- **Ruido** (σ): posición 0.105 mm, fuerza 0.42 kN, PA 0.07 bar, PB 0.29 bar; modo común
  a 133.8 Hz en los sensores de 24 V. El ruido de posición es el techo de los modelos.
- **Modelos NARX (Fase 1):** estático FIR 50.1 % en simulación libre (87 % del techo que
  impone el ruido); dinámico DBP 48.4 %. El sesgo de validación es la deriva del null.

---

## 5. Estado y pendientes (al 2026-11-02)

| Tema | Estado |
|---|---|
| VI v15 (reserva automática del cDAQ, protección NaN) | en uso; falta prueba tras reinicio completo con UPH encendida |
| Licencia driver HBM LabVIEW (QuantumX MX840B) | vencía ~2026-11-05; cotización pedida a HBK el 2026-10-28; si vence, registrar en catman (el control por cDAQ no depende de ella) |
| Diagrama eléctrico CADe_SIMU | hecho y simulado con las E/S reales (§2.4) |
| Modelo Simulink | **actualizado y revalidado el 2026-11-05 con 160/80 y limitadora blanda** (§3): RMS 0.0162 mm/s, extensión y retracción reproducidas (10–20 % en presiones intermedias). Falta: acumulador (volumen/precarga supuestos), volúmenes muertos, fricción, y repetir con otra temperatura del aceite |
| Diámetro interior del cilindro | **medido 160 mm** (ajuste 161.5 ± 3 mm, 2026-11-04, §2.3); falta repetir a otra temperatura/posición y confirmar con la placa |
| Episodio de 170–180 kN (soldadura del manifold) | recuperar fecha y presiones (`fuerzavscarga18.xls` de 2017, en el respaldo) y verificar el estado y la presión de diseño del manifold antes de presurizar cerca de 100 bar |
| `PID.c` (Formula Node de la v15/v16) | **v17 (2026-11-06): sin duplicar los parámetros.** `ParamPID.c` (segundo Formula Node: `cod`, `Kx`, `Tix`, `Kf` → `P[8]`, `act`) es el único lugar con los valores por defecto y los rangos; `PID.c` ya no los repite y, si `P` llega incompleto, termina el paso en FALLA con `u = 0` (antes inventaba defaults). Verificado: 0 diferencias en 320 000 pasos frente a v16 con `P` válido, 5 600 casos de `ParamPID` idénticos al anterior, y mutaciones detectadas. **Se retiró «Confirmar parámetros» (`conf`)** (2026-11-06): el ensayo arranca en el flanco de subida de INICIAR (`ini ≥ 5`, `inip < 5`); `inio = ini`; sin cambios con `conf` = 1 en 320 000 pasos. **Falta pegar ambos textos en la VI** (VI detenida, UPH apagada). Antes, **v16 (2026-11-05): versión simplificada y legible** (392 líneas, antes 438; cabecera corta con el historial en git, nombres descriptivos en vez de `xsp/xpr/dsp0/F1/x1`, estados y causas con nombre `ST_*`/`CAU_*`, números mágicos con nombre, sin el código muerto del QuantumX; mismos terminales y mismo comportamiento: **720 000 pasos de prueba diferencial sin diferencias**, con y sin el corte por presiones). **Falta pegarla en la VI** (la VI aún tiene la versión anterior; VI detenida y UPH apagada). El rótulo «limitador de presion(Bar) IMPORTANTE» del diagrama es un residuo de las primeras versiones (aclarado por el laboratorio el 2026-11-05): **el único limitador de presión es el del Formula Node, hoy desactivado (`prot_pres_on = 0`)**. Lo siguiente es de la versión anterior: **Actualizado el 2026-11-04** (verificado con 320 000 pasos de prueba diferencial: el controlador es idéntico al anterior; solo cambia la lógica de presiones): convención única **`pa` = cámara A sin vástago (`ai2`), `pb` = cámara B con vástago (`ai3`)**, llegan **en bar reales** (se escalan en el diagrama, antes del nodo: `pa = (V_ai2 − 1.9826)·12.5`, `pb = (V_ai3 − 1.9682)·50`); `Fp = 2.0106·pa − 1.5080·pb` (160/80 mm); `PAmax` = 95 bar; `Lg[12..14]` = pa, pb, Fp. **En la VI (2026-11-04):** los dos escalados están cableados (`ai2`→`pa`, `ai3`→`pb`) y el texto está pegado en el Formula Node (el archivo `.vi` contiene el `PID.c` actual, verificado). **Falta probarla con la UPH en marcha.** El corte por presión sigue **desactivado** (`protPres = 0`); reactivarlo tras comprobar con la UPH en marcha que `Lg[12]`/`Lg[13]` marcan ≈ 34/47 bar a −0.37 V y `Lg[14]` ≈ celda. Decisión pendiente: `Fcorte` 160 kN ≈ 80 bar y 200 kN ≈ 100 bar: **160 kN ya está en la zona del episodio de la soldadura (§2.3)**, bajarlo |
| Manómetro de la UPH | **leído el 2026-11-04/05**: reposo 96–98 bar, extensión +10 V 95 → ~40, retracción −10/−5/−3/−1 V = 55/85/90/90 bar (§3). Limitadora blanda (empieza a abrir hacia 43 bar, caudal completo a ~100). Falta el tarado nominal de la placa de la UPH y si se puede subir/bajar |
| Acumulador | buscar placa (volumen, precarga) |
| Escalas de presión | **bloquea la prueba del diámetro.** Resuelto el 2026-11-04: cableado definitivo (A→`ai2`, B→`ai3`, §2.4) y escalas de hoja de datos con ceros medidos, validadas por la serie de fuerza (offset 0 kN). Ya aplicadas en `params_prensa.m`, `Prensa.m` y `PID.c`; falta aplicarlas en el diagrama de la VI y revisar el log y los archivos de agosto |
| Fricción de Stribeck, volúmenes muertos | supuestos: identificar con rampas lentas a velocidad constante |
| Seta de emergencia | verificar si corta KM1 sin la VI (§2.4) |
| Neurocontrolador (BPTT) y comparación con el PID | sin empezar |
| Harness de lazo cerrado en Simulink con el controlador de la v15 | sin empezar |
| Agentic Toolkit MATLAB/Simulink | instalado en la PC con MATLAB; falta `satk_initialize` + `shareMATLABSession()` en `startup.m` |

---

## 6. Seguridad (no negociable)

Cilindro de hasta 200 kN, planta casi sin amortiguamiento (δ_h ≈ 1e-3).
- Alguien junto a la seta en toda maniobra; sin probeta para identificación.
- **No presurizar por encima de la presión de trabajo** (100 bar ≈ **201 kN** con 160/80 mm,
  medido 2026-11-04): el equipo ya llegó a 170–180 kN (≈ 85–90 bar) y falló una soldadura del
  manifold (§2.3), es decir, **a una presión por debajo de los 100 bar de trabajo**. Hasta
  verificar el manifold, no pasar de ~40–50 kN en ensayos de identificación; el corte de
  160 kN (≈ 80 bar) de `PID.c` no es margen seguro: bajarlo.
- AO a 0 V **antes** de energizar la bomba (el NI 9263 conserva la última tensión).
- **Al parar la UPH, separar el vástago ≥ 15 mm de cualquier probeta o tope** (y no dejar
  el AO en 0 V mientras haya presión): 0 V está +0.37 V sobre el null, y con el acumulador
  aún cargado el vástago se extiende y empujó una viga hasta **~17 kN** (2026-11-04, con
  solo 3 mm de separación), sin que nadie lo gobernara.
- Límites por software: ventana de trabajo 20–130 mm (recolocación 4–146 mm),
  fuerza 50 kN, PA 110 bar, PB 130 bar.
- LabVIEW y NI MAX reservan los módulos: cerrarlos antes de usar MATLAB (`Prensa.m`) y viceversa.
- Editar VIs solo con la VI detenida y la UPH apagada. En HBM `Init.vi`, siempre *Skip*.

---

## 7. Inventario

| Ruta | Contenido |
|---|---|
| `CLAUDE.md` | este documento |
| `README.md` | portada breve |
| `.claude/skills/resultados-medidos/SKILL.md` | registro detallado de las mediciones de agosto 2026 (material para el informe) |
| `labview_control_software/` | VIs **v15** (producción) y **v16** (copia de la v15 preparada para pegar `PID.c` v17 y `ParamPID.c`; la v13, que leía la fuerza del QuantumX, se retiró del repositorio el 2026-11-06 y sigue en el historial de git), `GlobalesPrensa.vi`, `TIME.vi`, `PID.c` (código del Formula Node del controlador, para pegar en la VI) y `ParamPID.c` (Formula Node de los parámetros del PID, que alimenta el arreglo `P` de `PID.c`). **Ver el diagrama sin abrir la VI:** `LabVIEWCLI.exe -OperationName CreateComparisonReport -VI1 <v13>.vi -VI2 <v15>.vi -ReportType html -ReportPath <informe>.html -o` (el comando está en `C:\Program Files (x86)\National Instruments\Shared\LabVIEW CLI\`) deja en `<informe>_files/` las imágenes `fp_2.png` (panel) y `bd_2.png` (diagrama completo de la v15, ~10 000 × 4 000 px) y la lista de objetos cambiados; funciona con VIs reales, con `TIME.vi` dio el error 66 |
| `simulink/` | `params_prensa.m`, `prensa_fluidtek.slx` |
| `matlab/Prensa.m` | script MATLAB del proyecto |
| `diagrama_electrico/` | `FLUIDTEK_UPH50.cad` (CADe_SIMU V4.2) y su generador `fluidtek.py` |
| `docs/` | memoria Fluidtek 2017, Jelali & Kroll 2003 cap. 2–4, catálogo Moog 761, EFT (no versionados) |
| `results/` | vacío en este equipo; datos medidos en el respaldo |
| Copias fuera del repo | `C:\Users\LEDI\fluidteck_respaldo_simulink\` (scripts de Simulink retirados) y `C:\Users\LEDI\fluidteck_respaldo_labview\PID_v15_original.c` (Formula Node original) |

---

## 8. Convenciones

- Responder en **español**. Unidades: mm, mm/s, kN, bar, V.
- Signo: u > 0 extiende el vástago (hacia abajo) y aumenta la posición medida.
- Redes neuronales escritas a mano, sin toolbox (despliegue = pesos dentro de un VI); NARX sobre
  **incrementos**, métrica = **simulación libre**.
- Antes de usar un número de la memoria de 2017, contrastarlo con este documento.
