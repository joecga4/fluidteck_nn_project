# CLAUDE.md — Identificación y control neuronal de una prensa servohidráulica (PUCP)

> **Doble propósito de este archivo:** (1) instrucciones de contexto para Claude Code
> durante el desarrollo, y (2) **fuente de documentación para el informe** (metodología,
> resultados, arquitectura y decisiones, todo en un solo lugar). Las secciones 1–8 son
> material directo para el informe; 9–10 son convenciones de trabajo.
>
> Proyecto hermano: `pi5_qnx_project` (motor DC bajo QNX). Se reutiliza su **metodología**
> —captura de excitación rica → modelo NARX → neurocontrolador → comparación de leyes de
> control en la planta real— pero **no su código**: allá el target era C sobre QNX; aquí es
> LabVIEW sobre un cDAQ. Y la planta es estructuralmente distinta (§3.3).

---

## 1. Contexto y objetivo

Sistema **servohidráulico de ensayo de materiales** (prensa) del laboratorio de la PUCP:
la **UPH 50** fabricada por **Fluidtek** (memoria de proyecto del 29-dic-2017, en
`docs/`). Ensaya probetas de concreto (losas, vigas) aplicando desplazamiento o carga
controlada mediante un cilindro hidráulico gobernado por una servoválvula Moog.

**Estado actual del equipo:** control **PID con autotuning de LabVIEW**, en lazo de
**posición**, corriendo en la PC contra un chasis NI cDAQ-9184 por Ethernet.

**Objetivo del proyecto:** sustituir/complementar ese PID por un **control basado en redes
neuronales**, en dos fases (la misma estructura del curso y del proyecto del motor):

1. **Identificación** — una red neuronal que modele la dinámica del servo-sistema
   (comando de la servoválvula → posición / fuerza), validada en **simulación libre**.
2. **Control** — un **neurocontrolador** entrenado contra ese modelo, desplegado en el
   equipo real y comparado contra el PID actual en las mismas condiciones.

**Modos de operación a cubrir** (decisión del 2026-08-12, §9):
| Modo | Variable controlada | Uso | Consigna típica |
|---|---|---|---|
| **Posición** | desplazamiento del vástago (mm) | aproximación a la probeta | escalón / posición absoluta |
| **Fuerza** | carga de la celda (kN) | precarga / contacto | escalón de kN |
| **Rampa de fuerza** | carga (kN) con pendiente | ensayo normado | kN/min |
| **Velocidad** | desplazamiento con pendiente | ensayo normado | mm/min (1.5 losa, 0.1 viga) |

Los dos primeros son lazos de regulación; los dos últimos son **seguimiento de rampa**, que
es donde el error de seguimiento (`ΔXA = velocidad_comando / Kvx`, §3.2) manda.

---

## 2. Hardware del sistema

### 2.1 Cadena hidráulica
- **Unidad de presión UPH 50** con tanque, indicador de nivel/temperatura HL91 30 T1 TB
  (0–100 °C) y motor eléctrico de inducción.
- **Bomba** de engranajes **Casappa PLP10-1 SAE**.
- **Filtros en línea** HF745-20.203 y HF705-10.040 SB040GD. Limpieza exigida por la
  servoválvula: **ISO 16/14/11** (o 15/13/10 para vida larga), filtro **β10 ≥ 75**.
- **Acumulador** de vejiga (absorbe pulsaciones y picos).
- **Manifold** de acero con cartuchos: **válvula check CXBA-XCN** (Sun Hydraulics) y
  **limitadora de presión RDBA-LAN** ajustada a **100 bar** (presión de trabajo del sistema).
- **Aceite** RANDO HD.

### 2.2 Servoválvula — **Moog G761-3001B  H04JOFM4VPL**
*(hoja de datos en `docs/Moog-ServoValves-761Series-Catalog-en.pdf`, Rev. M 2024)*

El código de pedido identifica la válvula sin ambigüedad, y **corrige varios datos de la
memoria**, que describía otra variante:

| Campo | Código | Significado |
|---|---|---|
| 1 Valve version | **H** | High response |
| 2 Rated flow | **04** | **4 L/min (1.0 gpm)** a `Δp_N = 35 bar` **por land** (70 bar en total) |
| 3 Bushing/spool | **J** | 4 vías, *axis cut*, **ZERO LAP** |
| 4 Presión máx / cuerpo | **O** | 315 bar, cuerpo de aluminio |
| 5 Pilot stage design | **F** | **Standard dynamics** (el *preferred model* lleva `G` = High) |
| 6 Carrete sin señal | **M** | Centrado |
| 7 Pilotaje | **4** | Interno (puerto P) |
| 8 Juntas | **V** | FKM (fluorocarbono) |
| 9 Conector | **P** | 4 pines MS sobre el lado P |
| 10 Señal de comando | **L** | **±40 mA** single/paralelo · **±20 mA** en serie |

**Dinámica real de esta variante** (`H04..F`, pág. 9 del catálogo):

| | |
|---|---|
| **−3 dB** | **120 Hz** ⇒ `ω_sv = 754 rad/s` |
| 90° de fase | 200 Hz |
| Escalón 0–100 % | 4 ms |

La memoria usaba **150 Hz**, que no corresponde: el `..G` (High dynamics) da 140 Hz y esta
es `..F`. Además el dato del catálogo se mide a **210 bar de pilotaje** y aquí se trabaja a
**100 bar**, así que la respuesta real será algo **más lenta** que esos 120 Hz.

**No linealidades declaradas por el fabricante** (pág. 7, a 210 bar / 32 mm²/s / 40 °C):

| | |
|---|---|
| Histéresis típica | **≤ 3.0 %** |
| Umbral (*threshold*) | ≤ 0.5 % |
| **Deriva de null por ΔT = 55 °C** | **≤ 2.0 %** |
| Tolerancia de caudal | ±10 % |

>  **El carrete es ZERO LAP, y eso confirma la medida.** El catálogo describe el *axis cut*
> como «minimal change in gain through null region», frente a las opciones de solape (`A` 3 %,
> `C` mínimo, `D` 10 %) que esta válvula **no** lleva. Es exactamente lo que midió el barrido:
> dos ramas lineales con R² ≈ 0.999 y **sin meseta** (§5.3b). La estimación inicial de una
> «zona muerta de 0.24 V» era incompatible con el hardware, además de con los datos.

> ✅ **Conexionado resuelto: single/paralelo, ±40 mA.** El código `L` admite ±40 mA
> (single/paralelo) o ±20 mA (serie), y no es un detalle: de ahí cuelga toda la escala del
> comando. Lo zanja el ajuste del amplificador, **10 V → 40 mA** (§2.4): coincide con el
> valor single/paralelo, así que **fondo de escala del AO = 100 % del carrete**.

**Deriva de null por temperatura — cierra un cabo que teníamos suelto.** La especificación
de ≤ 2 % por ΔT = 55 °C, con `±10 V ↔ ±40 mA`, son **0.20 V de comando por cada 55 °C**.
Y teníamos dos medidas que parecían no encajar:

- el barrido dio velocidad nula en **−0.370 V**;
- sostener 180 s en −0.370 V **seguía derivando** −0.95 mm/min, lo que implica un null real
  cercano a **−0.41 V**.

Esos **41 mV de diferencia equivalen a ~11 °C** de aceite según la especificación — un
calentamiento perfectamente normal en una sesión con la bomba en marcha. **No son dos
medidas contradictorias: es la misma planta a dos temperaturas.**

Consecuencia de fondo: **el null no es una constante que se calibre una vez.** 0.2 V de
deriva son ~4.7 mm/min, **tres veces la consigna del ensayo de losa**. Refuerza las dos
reglas ya anotadas: comparar siempre en la misma sesión registrando la temperatura del
aceite (§6.4), y que **el lazo necesita acción integral** — el null no se puede fijar en
lazo abierto por bien que se mida (§5.3c2).

### 2.3 Actuador
| Parámetro | Símbolo | Valor |
|---|---|---|
| Diámetro de pistón (émbolo) | D | 160 mm |
| Diámetro de vástago | d | 100 mm |
| **Carrera útil** | **L** | **150 mm** ← *dato del laboratorio (2026-08-12)* |
| **Área cámara A** (sin vástago, fondo) | **A_A** | **201.06 cm²** |
| **Área cámara B** (con vástago, anular) | **A_B** | **122.52 cm²** |
| **Relación de áreas** | A_A/A_B | **1.641** |
| Volumen total del cilindro | V_t | 3.4e−3 m³ |
| Masa total del sistema | M_t | 150 kg |
| **Montaje** | — | **VERTICAL**, vástago hacia abajo |
| **Masa colgante** (celda de carga) | m_c | **28 kg** |
| Amortiguamiento en la carga | B_p | 0 (despreciado en la memoria) |
| Módulo de compresibilidad del aceite | β | 1.7e9 Pa |
| Ganancia de presión (fugas) | K_ce | 2e−12 m⁵/(N·s) |

>  **Montaje vertical (dato del laboratorio, 2026-08-12).** El actuador trabaja en
> vertical y **los desplazamientos positivos son con el vástago saliendo, a favor de la
> gravedad**. De la punta cuelga la celda de carga, **≈ 28 kg**. Con `A_B = 122.52 cm²`,
> sostener esa masa son **0.22 bar** en la cámara anular — despreciable frente a las
> presiones de trabajo, pero **no despreciable para el equilibrio a comando nulo** (§5.3).
>
>  **El cilindro es ASIMÉTRICO y la memoria lo modela como simétrico.** La `A_p = 122.52 cm²`
> de la memoria (su Fig. 3.3) es en realidad el área de la **cámara anular**; la cámara de
> fondo tiene `π/4·0.160² = 201.06 cm²`, un **64 % más**. Usar una sola área equivale a
> suponer un cilindro de doble vástago, que no es este. Tres consecuencias, todas medibles:
>
> 1. **La fuerza no es `A_p·(P_A − P_B)`** sino **`F = P_A·A_A − P_B·A_B`**.
> 2. **La ganancia de velocidad difiere entre sentidos** — pero *no* en la relación 1.641
>    que sugiere la intuición: ver §5.4, el resultado es al revés de lo esperable.
> 3. **Intensificación de presión:** al retener o frenar, `P_B = P_A·(A_A/A_B)`, o sea
>    164 bar en la anular con solo 100 bar en la de fondo. **Eso explica por qué el
>    transductor de B es de 0–400 bar y el de A solo de 0–100 bar** (§2.4): no es un
>    capricho del fabricante, es esta ecuación con margen.

### 2.4 Instrumentación
| Sensor | Modelo | Rango | Señal |
|---|---|---|---|
| **Posición** | sensor de desplazamiento del actuador (la memoria cita un MTS Temposonics RH-M0400M) | **0–150 mm** ← *dato del laboratorio* | **0–10 V** ⇒ **15 mm/V** |
| **Fuerza** | celda **HMD2004LCD** (galgas, cilíndrica Ø165 × 81 mm) | **0–200 kN** a compresión, sobrecarga 150 % | 0–15 mV → amplificada a **0–10 V**; histéresis ±0.02 %, no linealidad ±0.02–0.03 % |
| **Presión cámara A** (sin vástago) | **Kobold SEN-3390/2** | **0–100 bar** | **2–10 V** (ver abajo) |
| **Presión cámara B** (con vástago) | **Kobold SEN-3390/2** | **0–400 bar** | **2–10 V** (ver abajo) |

> **Los transductores de presión tienen span 2–10 V, no 0–10 V.** Deducido de la medida, no
> supuesto: con la HPU apagada los manómetros marcan **0 bar en ambas cámaras**, y sin
> embargo los canales leen **1.9688 V** y **1.9810 V**. Con span 0–10 V eso serían 19.7 y
> 79.2 bar — imposible. Con span 2–10 V dan **−0.39 y −0.95 bar**, es decir cero dentro del
> error de ambos canales a la vez. Es la firma inconfundible de un transductor de
> **4–20 mA leído sobre 500 Ω** (4 mA → 2 V, 20 mA → 10 V). *Confirmar presurizando y
> comparando contra el manómetro.*
>
> ⚠ La carrera de 150 mm **contradice** el modelo `RH-M0400M` (400 mm) que cita la memoria.
> Manda el dato del laboratorio; queda anotado como una discrepancia más de la memoria (§8).

- **Amplificador de la servoválvula — ajuste real: `±10 V → ±40 mA`**, confirmado por el
  laboratorio el 2026-08-13. Es decir **`K_amp = 0.004 A/V`**; la memoria daba 0.003 A/V
  (30 mA a 10 V) y está desactualizada.

  > Coincide **exactamente** con la corriente nominal de la válvula (código `L`: ±40 mA
  > single/paralelo, §2.2). O sea: **el fondo de escala del AO corresponde al 100 % del
  > recorrido del carrete**, ni de más ni de menos. La cadena está bien dimensionada.

### 2.5 Adquisición y cómputo
| Módulo | Descripción |
|---|---|
| **NI cDAQ-9184** | chasis CompactDAQ **Ethernet** de 4 ranuras |
| **NI 9222** | AI ±10 V, **4 canales simultáneos**, 16 bits, **500 kS/s** ← posición (PV), celda, 2× presión |
| **NI 9263** | AO ±10 V, 4 canales, 16 bits, 100 kS/s ← comando a la servoválvula (CV) |
| **NI 9421** | DI 24 V, 8 canales ← motor encendido, saturación de filtro, sobrepresión, seta de emergencia |
| **NI 9472** | DO 24 V, 8 canales → relés/contactores del motor, parada de emergencia |

- **Eléctricos:** contactor, guardamotor, relés, interruptor termomagnético, motor de inducción.
- **Conexión:** PC ↔ cDAQ por **Ethernet CAT6**; configuración vía **NI MAX**.

#### 2.5.1 Acceso desde Python — verificado en esta PC el 2026-08-12
El driver **NI-DAQmx 26.0** está instalado y el chasis está registrado y **no simulado**.
Con `nidaqmx` (paquete oficial de NI, https://github.com/ni/nidaqmx-python) se enumera:

| Alias DAQmx | Módulo | S/N | Canales |
|---|---|---|---|
| `cDAQ9184-1ADC24C` | chasis cDAQ-9184 | 1ADC24C | — |
| `…Mod1` | **NI 9421** (DI) | 1C1E2F2 | `port0/line0..7` |
| `…Mod2` | **NI 9222 (BNC)** (AI) | 1C33510 | `ai0..ai3` |
| `…Mod3` | **NI 9263** (AO) | 1C224C9 | `ao0..ao3` |
| `…Mod4` | **NI 9472** (DO) | 1C098CD | `port0/line0..7` |

Coincide exactamente con lo que describe la memoria (§2.5). **Consecuencia importante para
el proyecto:** no hace falta pasar por LabVIEW para adquirir ni para excitar la planta.

**Reparto de canales del NI 9222 — confirmado por el laboratorio (2026-08-12):**
| Canal | Señal |
|---|---|
| `ai0` | **posición** del actuador (sensor de desplazamiento del vástago) |
| `ai1` | **celda de carga** |
| `ai2` | **presión de la cámara A** del actuador |
| `ai3` | **presión de la cámara B** del actuador |

Que los dos transductores midan **las dos cámaras** (y no, p. ej., suministro y retorno)
tiene tres consecuencias que conviene aprovechar:
1. **`P_A` y `P_B` son variables de estado del modelo físico** (§3). Se puede validar el
   modelo contra **estados internos**, no solo contra la salida — una prueba mucho más
   exigente que comparar posiciones.
2. **Dan una medida de fuerza independiente de la celda**, pero con la fórmula
   **asimétrica**: `F = P_A·A_A − P_B·A_B` (§2.3), **no** `A_p·(P_A − P_B)`. Con la fórmula
   simétrica el término de la cámara de fondo saldría un **64 % corto**. Sirve para
   verificar la calibración de la celda y para trabajar el lazo de fuerza sin probeta.
3. **Son candidatos naturales a entrar en el regresor del NARX.** En una planta integradora
   poco amortiguada, la presión lleva la información de la dinámica hidráulica que la
   posición por sí sola no muestra (la posición es su integral y la filtra). Merece probarse
   como entrada del modelo y compararlo contra el NARX solo con posición.

**Y sobre todo: la captura de identificación puede hacerse con temporización POR HARDWARE.**
Es la diferencia entre un `Ts` nominal y un `Ts` exacto:
- La secuencia de excitación completa se **precarga en el buffer del AO (NI 9263)** y el
  chasis la emite con **su propio reloj**, no con un lazo de software.
- El AI (NI 9222, muestreo **simultáneo** en sus 4 canales) se arranca con el **mismo
  trigger**, de modo que comando y medidas quedan **alineados por hardware**.
- El jitter del sistema operativo desaparece del dato: Windows solo tiene que vaciar
  buffers a disco a tiempo, no cumplir un plazo por muestra.

Esto **elimina la principal debilidad** que este proyecto tenía frente a `pi5_qnx_project`
(allá el `Ts = 20.00 ms` exacto lo daba el RTOS). Para la fase de identificación, la
temporización por hardware del cDAQ es **igual de buena o mejor**.

⚠ **Lo que la temporización por hardware NO resuelve: el lazo cerrado.** Un controlador
tiene que leer, calcular y escribir dentro del periodo, y eso es forzosamente
*software-timed*, punto a punto, sobre un enlace **Ethernet**. Ahí sí entran la latencia
del enlace y el jitter de Windows, igual con Python que con LabVIEW. **Medir esa latencia
de ida y vuelta es la Fase 0** (§6.1) y determina el `Ts` alcanzable del control.

⚠ **Conflicto de reserva de dispositivo — ya observado en la práctica (2026-08-12).** Dos
aplicaciones no pueden tener reservado el mismo módulo a la vez. En una prueba, los **cuatro
módulos** aparecieron reservados: los tenían **LabVIEW** y **NI MAX con un panel de prueba
abierto sobre el NI 9263** (el AO — es decir, un panel capaz de mandar tensión a la
servoválvula). Diagnóstico: intentar reservar cada módulo y mirar el error
`resource already reserved`. **Decisión (2026-08-12): durante las sesiones de captura se
cierran LabVIEW y MAX, y Python manda en todo el chasis.**

⚠ **El reparto de líneas digitales es DESCONOCIDO y no se puede adivinar.** El NI 9472 lleva
**el arranque del motor de la UPH y la parada de emergencia desde la pantalla** (memoria
§2.5): una de sus ocho líneas es la seta. Se buscó el mapeo en tres sitios y no está en
ninguno — la memoria solo describe funciones, los VIs guardan el diagrama comprimido (sin
cadenas legibles) y **NI MAX no tiene tareas ni canales globales guardados**. Por eso
`daq.py` deja `LINEA_HPU = None` y **exige `--linea-hpu N` explícito**. Sale del plano
eléctrico del tablero o del diagrama de bloques de `HPU.vi`.

**Estado de las DI con la UPH apagada (2026-08-12):** `line0 = 1`, `line1 = 1`, resto `0`.
Probablemente contactos normalmente cerrados («seta no pulsada», «filtro no saturado»), pero
**sin confirmar**. `--hpu` lee las DI antes y después de conmutar precisamente para
identificar cuál es «motor encendido».

---

## 3. Modelo físico de la planta (capítulo III de la memoria)

> ⚠ **Esta sección documenta el modelo DE LA MEMORIA, que es de una sola cámara y supone el
> cilindro simétrico.** Se conserva porque es la referencia del proyecto original y de donde
> salen `K_SA = 20.5 %/mm` y el criterio de estabilidad. **El modelo que usamos nosotros es
> el de DOS CÁMARAS de §5.4 / `tools/planta_sim.py`**, que reproduce la asimetría medida y
> del que `ω_h` y `δ_h` salen como consecuencia en vez de ponerse a mano. Donde ambos
> discrepen, manda §5.4.

### 3.1 Parámetros derivados
```
ω_h = √(4·β·A_p² / (V_t·M_t)) = √(4·1.7e9·(122.52e−4)² / (3.4e−3·150)) = 1414.74 rad/s  (225 Hz)
δ_h = (K_ce/A_p)·√(β·M_t/V_t) = (2e−12/122.52e−4)·√(1.7e9·150/3.4e−3) = 1.41e−3
ω_sv = 942.47 rad/s (150 Hz) ,  δ_sv = 0.7
K_sv = 0.00398 m³/(A·s) ,  K_amp = 0.003 A/V ,  K_f = 1 ,  K1 = 1000 mm/m ,  K2 = 0.1 V/%
```

### 3.2 Función de transferencia en lazo abierto
```
        LA(s) =            K_vx
                ───────────────────────────────────────
                (1 + s/ω_sv)·(s²/ω_h² + 2δ_h·s/ω_h + 1)·s
```
- **Es un INTEGRADOR** por el `s` del denominador: el comando de la servoválvula fija el
  **caudal**, y el caudal integrado sobre el área del pistón da **posición**. Ver §3.3.
- **Criterio de estabilidad** (margen de ganancia positivo): `K_vx < 2·ω_h·δ_h`.
  Con margen de ganancia de 6 dB: **`K_vx = ω_h·δ_h = 1.41e−3 · 1414.74 = 2 s⁻¹`**.
- De ahí la ganancia del PID de la memoria:
  `K_SA = K_vx·A_p / (K1·K2·K_amp·K_sv·K_f) = 2·122.52e−4 / (1000·0.1·0.003·0.00398) = **20.5 %/mm**`.
- **Rigidez ante fuerza externa:** `ΔY = K_ce·F_L / (A_p²·K_vx)`.
- **Error de seguimiento en rampa:** `ΔX_A = velocidad_de_comando / K_vx`.
  Con `K_vx = 2 s⁻¹` y una rampa de 1.5 mm/min = 0.025 mm/s → **ΔX_A = 12.5 µm** de retraso
  permanente. Es pequeño, pero es **error estructural del lazo tipo 1**: es exactamente lo
  que un neurocontrolador con anticipación puede eliminar.

### 3.3 Diferencia estructural con el proyecto del motor — leer antes de reutilizar nada
| | `pi5_qnx_project` (motor DC) | **este proyecto (servohidráulico)** |
|---|---|---|
| Entrada | duty PWM (%) | comando de servoválvula (V → mA) |
| Salida controlada | velocidad (RPM) | **posición (mm)** / fuerza (kN) |
| Estructura | **planta de 1.er orden**: duty → RPM directamente | **integrador**: comando → **velocidad**, y la posición es su integral |
| Amortiguamiento | alto (fricción + reductor 270:1) | **δ_h = 1.4e−3: prácticamente nulo** → resonancia a 225 Hz muy marcada |
| No linealidad dominante | zona muerta ~38 % del L298N | histéresis + **null offset** del carrete, fricción de sellos (*stick-slip*), asimetría de áreas |
| Carga | inercia fija | **la probeta**: rigidez variable que **cambia al fracturarse** |

**Consecuencia práctica #1 — el NARX no debe predecir la posición directamente.** Con una
planta integradora, `y(k+1) ≈ y(k)` domina y la red aprende la identidad: el ajuste sale
99 % y el modelo no sirve para nada. La formulación correcta es predecir el **incremento**
(equivalentemente, la velocidad):
```
Δy(k+1) = f( Δy(k), Δy(k−1), …, u(k), u(k−1), … )    y luego  y(k+1) = y(k) + Δy(k+1)
```
En simulación libre se reintegra. Esta es la decisión de diseño más importante del proyecto
y hay que sostenerla en todo el pipeline.

**Consecuencia práctica #2 — δ_h ≈ 0 significa que la planta no se auto-amortigua.** Toda
la estabilidad viene del lazo. Los límites de seguridad del §6.3 no son burocracia.

---

## 4. Estado del proyecto (hitos)

| Hito | Fecha | Estado | Resultado clave |
|---|---|---|---|
| Lectura de la memoria y extracción de parámetros de planta | 2026-08-12 | ✅ | §2 y §3 completos (255 pp. procesadas) |
| Auditoría de los datos históricos de 2017 | 2026-08-12 | ✅ | **insuficientes para NN** (4 escalones); sí confirman el integrador y el null offset (§5.1) |
| Decisión de alcance: plataforma, variables, captura nueva | 2026-08-12 | ✅ | LabVIEW+cDAQ · 4 modos · sí hay acceso al equipo (§9) |
| Simulador físico de la planta (`tools/planta_sim.py`) | 2026-08-12 | ✅ | banco de pruebas del pipeline antes de tocar la máquina (§5.5) |
| **cDAQ accesible desde Python** (`nidaqmx` + driver 26.0) | 2026-08-12 | ✅ | chasis real y 4 módulos enumerados; habilita **captura temporizada por hardware** (§2.5.1) |
| Diseño de la secuencia de excitación (`tools/gen_excitacion.py`) | 2026-08-12 | ✅ | APRBS multinivel con signo + plegado dentro de la ventana segura (§6.2) |
| Capa de E/S y diagnóstico (`tools/daq.py`) | 2026-08-12 | ✅ | `--diag` sin mover la planta; captura AO+AI con trigger común |
| Maniobras de máquina: `--hpu`, `--caracteriza`, `--jog` + protocolo de Fase 0 | 2026-08-12 | ✅ | `--armar` obligatorio; el puerto DO se escribe entero para sostener el permisivo |
| **Servoválvula identificada por catálogo** (G761-3001B H04JOFM4VPL) | 2026-08-13 | ✅ | 4 L/min · ±40 mA · **120 Hz** · zero lap (§2.2) |
| **Latencia del lazo medida**: 5.46 ms mediana ⇒ **Ts = 20 ms (50 Hz)** | 2026-08-13 | ✅ | el primer valor (202 ms) era un start/stop implícito por tarea sin arrancar (§6.1) |
| **Cadena de mando cerrada**: amplificador real 10 V → 40 mA | 2026-08-13 | ✅ | `K_amp = 0.004 A/V`; el modelo predice **0.503** vs **0.446** medido: **+13 %**, dentro de la tolerancia del fabricante (§8) |
| **Escala de presión calibrada por dos puntos** | 2026-08-12 | ✅ | balance de fuerzas de −141 kN a **+0.8 kN**; `P_B/P_A = 1.667` vs `A_A/A_B = 1.641` (§5.3c) |
| **Primer arranque de la UPH desde Python** | 2026-08-12 | ✅ | DO line0 = marcha · DO line5 = permisivo · DI line7 = motor encendido; **K₊ = 0.446 / K₋ = 0.378 mm/s·V**, asimetría medida **0.847** vs 0.772 del modelo; deriva **8.4 mm/min** (§5.3) |
| **Fase 0 (parcial): línea base de los 4 sensores en el equipo real** | 2026-08-12 | ✅ | celda y presiones **concuerdan** en fuerza; ruido de modo común a 133.8 Hz; σ_posición = 0.28 mm ⇒ límite de medida de velocidad (§5.2) |
| Mapeo de canales AI confirmado por el laboratorio | 2026-08-12 | ✅ | ai0 posición · ai1 celda · ai2/ai3 **las dos cámaras** (§2.5.1) |
| Rangos reales de sensores + carrera de 150 mm | 2026-08-12 | ✅ | span de presión **2–10 V** deducido y verificado; corrige escalas y límites de seguridad (§2.4) |
| **Modelo de dos cámaras** (cilindro asimétrico) | 2026-08-12 | ✅ | predice relación de velocidades **0.772**, y 2017 midió **0.78** (§5.4) |
| **Captura real en el equipo (train + val)** | 2026-08-13 | ✅ | 2 × 610 s a 1 kHz, **Ts exacto**, sin muestras perdidas; el null se movió **71 mV** entre ambas (§5.6) |
| **Modelo NARX de la planta — línea base** | 2026-08-13 | ✅ | FIR no lineal `dy(k+1)=f(u(k),u(k−1))`: **50.1 %** en simulación libre, el **87 % del techo** que impone el ruido (§5.7) |
| Neurocontrolador (BPTT) | — | ⬜ | |
| Despliegue en LabVIEW y comparación contra el PID | — | ⬜ | |

---

## 5. Resultados

### 5.1 Auditoría de los datos históricos (2017) — `labview/Nueva carpeta/data para identificacion/`
Los archivos `.xls` son en realidad **texto tabulado**, no Excel. Dos familias:

**(a) Lazo abierto** `lazoabierto{1,2,3,4,5}.xls` — columnas `ENTRADA (V)` / `SALIDA (mm)`:

| Archivo | N | Niveles de entrada | **Cambios de escalón** |
|---|---|---|---|
| lazoabierto1 | 2505 | 0, +6, −6, −5 | **3** |
| lazoabierto2 | 2108 | 0, ±12 | **3** |
| lazoabierto3 | 2471 | 0, ±24, −20 | **3** |
| lazoabierto4 | 4373 | 0, ±48, −40 | **3** |
| lazoabierto5 | 4417 | 0, ±48, −40 | **3** |

> **Veredicto: no sirven para entrenar una red.** Cada archivo es *una* subida y *una*
> bajada. Un NARX necesita cientos de transiciones a niveles variados; con 4 segmentos por
> archivo el modelo memoriza esos cuatro puntos. Sirven, eso sí, como **verificación
> cruzada** del modelo físico y como evidencia de dos hechos:

**Hecho 1 — la planta es un integrador, y bastante lineal en ganancia de velocidad:**

| Entrada (u) | Velocidad media | Ganancia (mm/s por unidad) |
|---|---|---|
| +6 | +0.081 mm/s | 0.0135 |
| +24 | +0.373 mm/s | 0.0155 |
| +48 | +0.721 mm/s | 0.0150 |
| −24 | −0.259 mm/s | 0.0108 |
| −48 | −0.578 mm/s | 0.0120 |

**Hecho 2 — hay asimetría y hay deriva de null.** La ganancia en sentido negativo es
**~20 % menor** que en positivo (0.0114 vs 0.0147 medio) — compatible con la asimetría de
áreas de un cilindro con vástago pasante parcial (§2.3) y/o con el ajuste del carrete. Y a
**entrada 0 la posición deriva a +0.03 mm/s**: el carrete no está exactamente en null.
Esa deriva es del **orden del 4 %** de la velocidad a fondo de escala — nada despreciable
en un ensayo de 0.1 mm/min.

⚠ **Dos incertidumbres abiertas en estos datos, a resolver con la captura nueva:**
1. **No hay columna de tiempo.** Las velocidades de arriba asumen **Ts = 0.1 s** (el paso
   que sí aparece en `posibleDATA.lvm`). Si el Ts real fue otro, todas escalan por igual —
   las *relaciones* (linealidad, asimetría, deriva) se mantienen; los valores absolutos no.
2. **La unidad de `ENTRADA` no es voltios.** Los niveles ±6/±12/±24/±48 exceden los ±10 V
   del AO; con `K2 = 0.1 V/%` encajan como **% de la variable de control** (48 % = 4.8 V).
   Bajo esa lectura y con Ts = 0.1 s, la ganancia medida (0.015 mm/s por %) queda **~6.5×
   por debajo** de la que predice el modelo físico (0.974 mm/(s·V) ⇒ 0.0974 mm/s por %).
   **Esa discrepancia de 6.5× está sin explicar** y es el primer objetivo cuantitativo de la
   captura nueva. Candidatos: el Ts supuesto, el ajuste real de `K_amp`, la caída de
   presión de trabajo frente a los 70 bar de la curva, o la limitación de caudal de la bomba.

**(b) Ensayos con carga** `fuerzavscarga18.xls` (25 497 muestras, **con columna de tiempo**,
hasta 18.4 t ≈ 180 kN) y `fuerzavscargaconLCDT.xls` (10 321 muestras, 6 t). `Tiempo(s)`
llega a 4056 s con dt ≈ 0.159 s. Son **ensayos de rotura en lazo cerrado**, no
identificación, pero son la única evidencia disponible de la **relación fuerza–desplazamiento
con probeta** y del comportamiento en el instante de la fractura. Útiles para dimensionar el
lazo de fuerza.

### 5.2 Fase 0 (parcial) — línea base de los sensores, medida el 2026-08-12
Primera medida real sobre el equipo: **5 s a 5 kHz en los 4 AI, sin escribir nada en el AO**
(`tools/daq.py --diag`). La planta estaba en reposo.

| Canal | Media | σ (V) | **σ (ing.)** | pico-pico |
|---|---|---|---|---|
| `ai0` posición | 0.1393 V → **2.09 mm** | 6.98 mV | **0.105 mm** | 0.77 mm |
| `ai1` fuerza | −0.0068 V → −0.14 kN | 21.1 mV | **0.422 kN** | 3.69 kN |
| `ai2` presión A | 1.9686 V → **−0.39 bar** | 5.67 mV | **0.071 bar** | 0.68 bar |
| `ai3` presión B | 1.9802 V → **−0.95 bar** | 5.77 mV | **0.289 bar** | 3.09 bar |

*(escalas corregidas: posición 0–150 mm, presión A 0–100 bar y B 0–400 bar sobre span
2–10 V. La cámara B, al tener 4× de rango, paga 4× de ruido en unidades de ingeniería.)*

**(a) Las escalas de presión quedan determinadas — span 2–10 V.** Con los manómetros a 0 y
la HPU apagada, ambos canales leen ~1.97 V; con span 2–10 V eso da **−0.39 y −0.95 bar**,
cero dentro del error (§2.4). *Corrección respecto a la primera lectura de estos datos: con
la escala 0–10 V que se supuso al principio salían 19.7 y 79.2 bar, y el "acuerdo" que se
anotó entre `A_p·P_L` y la celda era una coincidencia entre dos cantidades que en realidad
valen ambas cero.* La comparación **fuerza-celda vs `P_A·A_A − P_B·A_B`** sigue siendo una
validación cruzada valiosa, pero **hay que hacerla con la planta cargada**, no en reposo.

**(b) Hay ruido de modo común a 133.8 Hz y 1462.6 Hz.** Aparece en **posición y en las dos
presiones** —los tres sensores alimentados a 24 V— y **no** en la celda, que va por su
propio amplificador:

| Canal | Componentes dominantes |
|---|---|
| posición | **133.8 Hz : 5.33 mV** · 7.6 Hz : 3.16 mV · 1462.6 Hz : 3.04 mV |
| presión A | 133.8 Hz : 2.19 mV · 1462.6 Hz : 1.78 mV |
| presión B | 133.8 Hz : 2.24 mV · 1462.6 Hz : 1.77 mV |
| fuerza | *(banda ancha, sin tono dominante)* |

No son 50/60 Hz ni sus armónicos, así que **no es red eléctrica**: el patrón apunta a la
**fuente de 24 V compartida** o a un lazo de masa. Es una pista concreta y accionable —
atacarla mejora directamente la medida de velocidad a baja consigna (punto c). Mientras
tanto, como la captura va temporizada por hardware, **muestrear rápido y diezmar promedia
esos tonos casi gratis**.

**(c) Cuánta velocidad se puede medir — el número que condiciona el lazo lento.**
Con σ_posición = 0.279 mm, la incertidumbre de la velocidad estimada por ajuste lineal
sobre una ventana de duración T (a 100 Hz) es:

| Ventana | σ de la velocidad |
|---|---|
| 1 s | **2.18 mm/min** |
| 3 s | **0.42 mm/min** |
| 10 s | **0.069 mm/min** |
| 30 s | **0.013 mm/min** |
| 60 s | **0.0047 mm/min** |

Contrástese con las consignas normadas: **1.5 mm/min (losa)** y **0.1 mm/min (viga)**.
Conclusión operativa: **la velocidad no se puede obtener por diferencia entre muestras
consecutivas**; hace falta ajuste sobre ventana, y la ventana necesaria es de **~2–3 s para
1.5 mm/min** y de **~10–15 s para 0.1 mm/min** (criterio: σ ≤ 10 % de la consigna). Eso acota el ancho de banda alcanzable del
lazo de velocidad y es un límite **de medida**, no de control: ningún controlador, neuronal
o no, puede regular mejor que lo que puede medir. Bajar el ruido de posición (punto b) es
la palanca más directa que tiene este proyecto.

✅ **Resuelto (2026-08-12):** los ~1.97 V de ambos canales no eran presión sino el **cero
del span 2–10 V**. El laboratorio confirmó que los manómetros marcan 0 en ambas cámaras con
la HPU apagada, lo que fija la escala sin ambigüedad (§2.4).

### 5.3 Fase 0 — primer arranque de la UPH desde Python (2026-08-12)

Primera vez que el equipo se opera desde el pipeline nuevo. Tres resultados.

**(a) Mapeo digital resuelto.** El laboratorio leyó las salidas digitales del VI de LabVIEW:

| línea | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| UPH apagada | F | F | F | F | F | **T** | F | F |
| UPH encendida | **T** | F | F | F | F | **T** | F | F |

- **`DO line0` = arranque de la UPH.** Es la única que cambia.
- **`DO line5` = permisivo, energizado siempre**, también con la bomba parada. La memoria
  dice que el NI 9472 «envía señal de parada de emergencia desde la pantalla»: una salida
  alta en reposo *y* en marcha es exactamente eso, un permisivo en lógica negada. Su función
  exacta no está confirmada, pero **el tratamiento correcto es el mismo: sostenerla**.
  Por eso `daq.py` escribe **el puerto entero** (`PuertoDO`) y nunca una línea suelta: al
  liberar LabVIEW su tarea, las salidas pueden volver a 0 y tirar el permisivo.
- **`DI line7` = motor encendido.** Pasa de 0 a 1 al arrancar, verificado dos veces. Las
  `DI line0` y `line1` siguen altas en ambos estados (contactos normalmente cerrados).

**(b) ✅ CURVA COMANDO → VELOCIDAD, medida completa** (`results/caracterizacion_2026-08-12.csv`).
Barrido de 13 escalones de 2.5 s, UPH en marcha, sin probeta:

| u [V] | v [mm/s] | | u [V] | v [mm/s] |
|---|---|---|---|---|
| −2.00 | −0.6252 | | +0.05 | +0.1456 |
| −1.00 | −0.2240 | | +0.10 | +0.1667 |
| −0.50 | −0.0380 | | +0.25 | +0.2230 |
| −0.25 | +0.0452 | | +0.50 | +0.3203 |
| −0.10 | +0.0906 | | +1.00 | +0.5547 |
| −0.05 | +0.1161 | | +2.00 | +1.0120 |
| **0.00** | **+0.1405** | | | |

Ajuste por ramas, **ambas casi perfectamente lineales**:

```
u > 0 :  v = 0.4459·u + 0.1139     (R² = 0.9991)
u < 0 :  v = 0.3779·u + 0.1397     (R² = 0.9986)
```

- **Ganancia: K₊ = 0.446 mm/s·V, K₋ = 0.378 mm/s·V.**
- **Asimetría |K₋/K₊| = 0.847**, frente al **0.772** que predice el modelo de dos cámaras
  (§5.4). Coinciden dentro del 10 % — el modelo asimétrico está bien planteado.
- **Deriva a comando cero: +0.1405 mm/s = 8.43 mm/min**, confirmada.
- **El comando de velocidad cero no es 0 V sino ≈ −0.26 V (rama +) / −0.37 V (rama −).**

**Y esa deriva NO es un carrete descentrado: es el peso cayendo por la fuga del null.**
Con el actuador **en vertical, positivo a favor de la gravedad** (§2.3), y con la celda de
28 kg colgando, a carrete cerrado queda una fuga residual —el centro cerrado no es estanco—
y el conjunto **desciende solo**. Que la deriva sea positiva (hacia afuera, hacia abajo) es
exactamente lo que predice la gravedad. Un offset mecánico del carrete no tendría por qué
apuntar en el sentido del peso.

⚠ **Consecuencia para el modelo, y es seria: esa deriva DEPENDE DE LA CARGA.** Sin probeta,
el peso cae libre por la fuga. Con probeta en contacto, la reacción cambia el balance y la
deriva será otra (menor, nula, o incluso invertida). **Un modelo identificado sin probeta no
predice el comportamiento con probeta cerca del cruce por cero** — que es justo donde ocurre
el ensayo. Es la misma advertencia de §8 sobre el lazo de fuerza, ahora con una causa
concreta. Habrá que decidir si se captura también con una probeta rígida de referencia.

**Qué significa para los ensayos normados.** Ambos caen en la rama negativa:

| Ensayo | velocidad | comando |
|---|---|---|
| Losa | 1.5 mm/min | **−0.3036 V** |
| Viga | 0.1 mm/min | **−0.3654 V** |
| *(velocidad cero)* | 0 | *−0.370 V* |

Se separan **61.7 mV**, y el de viga queda a **4.6 mV del punto de velocidad nula**. El AO de
16 bits da 0.305 mV de LSB, así que resolución hay (15 LSB); lo que hay es un punto de
trabajo pegado al cruce por cero, **donde además cambia la rama** (la ganancia salta de
0.378 a 0.446 al cruzar). Un PID lineal tiene que gobernar ahí con una ganancia que también
sirva a 1 mm/s: es exactamente el hueco que llena un feedforward no lineal aprendido, y el
equivalente hidráulico de la zona muerta del L298N en `pi5_qnx_project`.

> **⚠ Dos correcciones sobre lo que se anotó antes en esta misma sesión.**
>
> 1. **No hay zona muerta de 0.24 V.** Se estimó con dos puntos y salió
>    `v = 0.718·(u − 0.241)`, de donde se concluyó una zona muerta con los ensayos normados
>    en su borde. **El barrido completo lo desmiente**: las dos ramas son lineales con
>    R² ≈ 0.999 y no aparece meseta. El punto malo era el «tanteo» de +0.30 V, tomado
>    **justo tras arrancar la bomba**, cuando las cámaras aún se estaban presurizando
>    (`P_B` sube de 105 a 147 bar durante los primeros escalones). Ese punto daba
>    0.043 mm/s donde el barrido da 0.223 mm/s a +0.25 V. **Lección: no medir nada en los
>    primeros segundos tras arrancar la UPH**; `_descubre_sentido` sigue usándose solo para
>    el signo, que sí acierta.
> 2. **La deriva a comando cero era buena.** Se midió +0.139 mm/s, luego se retiró por
>    considerarla contaminada por ese mismo transitorio, y el barrido la confirma en
>    **+0.1405 mm/s**. Queda como dato válido.
>
> Lo que sí sobrevive de la primera lectura es la conclusión de fondo: **el punto de trabajo
> de los ensayos está en una región estrecha y no lineal, lejos del cero de comando.**

**(b2) Un lazo P puro no puede posicionar esta planta.** Tres barridos consecutivos
abortaron en la recolocación previa, y el motivo es estructural, no de sintonía: con
`Kp = 0.08 V/mm` un error de 0.5 mm pide 0.04 V, y con el null en −0.3 V ese comando ni
siquiera invierte el sentido de la deriva. El vástago **sube** mientras el lazo pide bajar.
Se resolvió no exigiendo posición exacta antes de barrer (basta ±25 mm del centro) y
añadiendo compensación de offset al comando. Es, en pequeño, el mismo problema que el
neurocontrolador tiene que resolver.

**(c) ✅ ESCALA DE PRESIÓN CALIBRADA — y el balance de fuerzas cierra.**
Calibración por dos puntos (`tools/daq.py --calibra-presion`):

| | canal A (`ai2`) | canal B (`ai3`) |
|---|---|---|
| **Punto 0** (UPH parada, manómetros a 0) | 1.9697 V | 1.9801 V |
| **Punto 1** (UPH en marcha, AO a −0.370 V, media de 180 s) | 2.8987 V | 4.7329 V |
| Manómetro en el punto 1 | **30 bar** | **50 bar** |
| **Escala resultante** | **32.293 bar/V** | **18.163 bar/V** |
| Fondo de escala a 10 V | 259.3 bar | 145.7 bar |

Que ambos canales den ~2.0 V a presión nula **confirma el span 4–20 mA sobre 500 Ω**
(2–10 V) que se había deducido en §2.4. Se usa el `V0` medido y no 2.000 V exactos: así la
recta absorbe también el offset del propio canal de entrada.

**La validación que de verdad cierra el asunto, y no es circular.** Con el pistón en
equilibrio y sin carga externa debe cumplirse `P_A·A_A = P_B·A_B`, es decir
**`P_B/P_A = A_A/A_B`**:

| | valor |
|---|---|
| Relación de áreas `A_A/A_B` (geometría) | **1.6410** |
| Relación de presiones `50/30` (manómetros) | **1.6667** |
| **Discrepancia** | **1.6 %** |

Las presiones se leyeron a ojo en dos manómetros analógicos y las áreas salen de la
geometría del cilindro: que coincidan al 1.6 % confirma **de una vez** las áreas asimétricas,
el reparto `ai2` = cámara grande / `ai3` = anular, y las propias lecturas. Es la relación de
**intensificación** de §2.3, medida directamente.

**Y el balance de fuerzas, que antes era absurdo, ahora cierra:**

| | escala supuesta | **escala calibrada** |
|---|---|---|
| `P_A·A_A` | 21.2 kN | 60.32 kN |
| `P_B·A_B` | 162.5 kN | 61.26 kN |
| Neto hidráulico | −141.3 kN | **−0.94 kN** |
| + peso del conjunto (~178 kg) | — | +1.75 kN |
| **Suma** | −141.3 kN | **+0.80 kN** |
| Celda de carga | −0.14 kN | −0.14 kN |

De **141 kN de disparate a 0.8 kN de residuo**, que es del orden de la fricción de sellos de
un cilindro Ø160. Las presiones ya son utilizables para el modelo y para el lazo de fuerza.

⚠ **Lo que no encaja: los fondos de escala.** Salen **259 y 146 bar**, y el laboratorio
reportó **0–100 y 0–400 bar**. La calibración empírica cierra la física, así que es la que
se usa, pero **conviene mirar la placa de los transductores**: o no son los del catálogo, o
los rangos anotados corresponden a otra cosa.

**(c2) La deriva residual en el null: −0.95 mm/min.** Sosteniendo el AO en −0.370 V durante
180 s, el vástago pasó de 96.76 a 93.92 mm. No es cero, y **es del mismo orden que el ensayo
de losa (1.5 mm/min)**. Dos lecturas, ambas útiles:
- El comando de velocidad nula real está algo más cerca de **−0.41 V** que de −0.370 V, o se
  ha movido con la temperatura del aceite durante la sesión.
- Más importante: **el punto de velocidad nula no se puede fijar en lazo abierto.** Un error
  de 40 mV en el null produce un error de velocidad comparable a la propia consigna del
  ensayo. Hace falta acción integral, y es un argumento más para la ley mixta PID+red.

**(d) Lección de procedimiento, aprendida moviendo el equipo.** El primer arranque se hizo
**sin forzar antes el AO a 0 V**, y el vástago se desplazó 15 mm solo: el NI 9263 **conserva
la última tensión escrita**, y la había dejado un panel de prueba de NI MAX. Presurizar es
dar energía a lo que ya esté pedido. `set_hpu` ahora **pone el AO a cero antes de energizar
la bomba**, siempre. Es la clase de detalle que no aparece en ninguna memoria.

### 5.4 La asimetría del cilindro: el resultado va al revés de lo que parece
Consecuencia directa del modelo de dos cámaras (`tools/planta_sim.py --check`), y una de las
cosas que la memoria no podía dar porque suponía el cilindro simétrico.

**La cuenta ingenua es falsa.** «La cámara anular tiene menos área, luego retraer será
1.64× más rápido» — no. El caudal no lo fija el área sino **el orificio**: al retraer, la
cámara **grande** tiene que evacuar `A_A·v` por el orificio de retorno, y *ese* es el cuello
de botella. Resolviendo el régimen (continuidad + balance de fuerzas + ecuación de orificio):

| Sentido | K velocidad | P_A régimen | P_B régimen |
|---|---|---|---|
| Extendiendo (P→A) | **0.902 mm/s por V** | 19.45 bar | 31.91 bar |
| Retrayendo (P→B) | **0.697 mm/s por V** | 50.06 bar | 82.15 bar |
| | **relación 0.772** | | |

**Retraer es más LENTO, no más rápido.** Y los datos de 2017 dan `0.0114/0.0147 = 0.78`
(§5.1): **coincide con 0.772 dentro del 1 %**. Es la primera predicción no trivial que el
modelo acierta contra datos medidos, y da confianza en la estructura de dos cámaras.

Nótese además que **retraer trabaja a presiones mucho más altas** (50/82 bar frente a
19/32 bar): la cámara anular se acerca a los 100 bar de la limitadora. Eso importa para la
seguridad de la captura y explica el rango de 400 bar de su transductor.

**Un error de modelado que costó encontrar, anotado para no repetirlo.** La primera versión
metía el `K_ce = 2e−12 m⁵/(N·s)` de la memoria como fuga física entre cámaras, y la relación
salía **0.363** en vez de 0.772. `K_ce` es el coeficiente **linealizado** caudal-presión:
agrupa la fuga real *más* la pendiente `dQ/dP` de la propia servoválvula. En un modelo no
lineal esa pendiente **ya la aporta la ecuación de orificio**, así que meter `K_ce` la cuenta
dos veces. Con `K_fuga = 0` el simulador reproduce exactamente la solución analítica.

### 5.6 Captura de identificación — ✅ hecha el 2026-08-13

Dos secuencias **independientes** (semillas 1 y 7), en la **misma sesión**, sin probeta,
emitidas con temporización por hardware:

| | `train` | `val` |
|---|---|---|
| Muestras / duración | 609 543 / 609.5 s | 612 094 / 612.1 s |
| **Ts (mín – máx)** | **1.0000 – 1.0000 ms** | **1.0000 – 1.0000 ms** |
| Recorrido | 71.6 – 99.4 mm | 42.9 – 77.0 mm |
| Comando | −9.95 … +8.43 V | −10.00 … +5.72 V |

**El `Ts` sale exacto hasta la última cifra**, mínimo y máximo idénticos. Es lo que da el
reloj del chasis y lo que ningún lazo de software puede garantizar (§2.5.1). Se emitieron
609 mil muestras sin perder ninguna y sin que saltara ningún límite.

**(a) La captura reproduce el barrido — validación cruzada con otra excitación.**
Ajustando la curva estática sobre los tramos de régimen de la propia captura:

| | captura APRBS | barrido de escalones | dif. |
|---|---|---|---|
| K⁺ | 0.4340 | 0.4459 | −2.7 % |
| K⁻ | 0.3766 | 0.3779 | −0.3 % |

R² = 0.999 en ambas ramas. Dos excitaciones de diseño completamente distinto dan la misma
planta.

**(b) Las dos capturas coinciden entre sí.**

| | `train` | `val` | dif. |
|---|---|---|---|
| K⁺ | 0.4340 | 0.4338 | −0.1 % |
| K⁻ | 0.3766 | 0.3711 | −1.5 % |

**(c) ⚠⚠ EL NULL SE MOVIÓ 71 mV ENTRE LAS DOS CAPTURAS — en diez minutos.**

| | |
|---|---|
| Comando de velocidad nula, `train` | **−0.356 V** |
| Comando de velocidad nula, `val` | **−0.285 V** |
| Desplazamiento | **+71 mV** ⇒ **1.58 mm/min** |

Son **~20 °C de aceite** según la especificación de Moog (0.20 V por 55 °C, §2.2) — un
calentamiento normal en una sesión con la bomba en marcha. Y en velocidad significa:

| Ensayo | consigna | la deriva del null es |
|---|---|---|
| Losa | 1.5 mm/min | **1.1× la consigna entera** |
| Viga | 0.1 mm/min | **16× la consigna** |

**En diez minutos de operación el punto de velocidad cero se movió más que la consigna
completa del ensayo de losa.** Tres consecuencias, y hay que tenerlas presentes al entrenar:

1. **`train` y `val` tienen nulls distintos.** Un NARX que aprenda la relación *absoluta*
   `u → velocidad` verá las dos series como incoherentes y promediará: el error de
   validación no bajará de ahí por mucho que crezca la red. No es sobreajuste, es que **las
   dos series describen plantas ligeramente distintas**.
2. **El modelo no debe apoyarse en un null absoluto.** Predecir incrementos (§3.3) ayuda
   pero no basta: el término independiente sigue estando. Opciones: meter la **temperatura
   del aceite** como entrada del regresor, o aceptar que el sesgo lo corrige el integrador
   del lazo.
3. **Refuerza la ley mixta PID+red** como hipótesis: la red aporta la anticipación y el
   integrador absorbe una deriva que **ningún modelo estático puede seguir**.

Y confirma con números la regla de §6.4: **comparar leyes de control en sesiones distintas
no tiene sentido en esta planta.**

**(d) Tercera validación de las presiones**, ahora sobre 610 s de movimiento:

| | |
|---|---|
| `P_A·A_A − P_B·A_B` (media) | −1.408 kN |
| Peso del conjunto móvil (~178 kg) | +1.75 kN |
| **Suma** | **+0.34 kN** |
| Celda de carga | −0.089 kN |

Los dos términos grandes casi se cancelan, como deben: a velocidades modestas y sin
aceleraciones grandes la fuerza neta ronda cero. El residuo es fricción de sellos. La celda
marca ~0 porque **sin probeta no hay reacción externa que medir**.

Ficheros: `results/captura_{train,val}.csv` · figura `results/captura_train.png`.
⚠ Los CSV pesan ~40 MB cada uno y **no se versionan** (ver `.gitignore`).

### 5.5 Simulador físico — `tools/planta_sim.py`
Implementa §3 en espacio de estados discreto para poder desarrollar y validar **todo el
pipeline de redes antes de gastar tiempo de máquina**:
- servoválvula de 2.º orden (ω_sv, δ_sv) → caudal,
- dinámica hidráulica de 2.º orden (ω_h, δ_h) → velocidad del pistón,
- integrador → posición,
- **no linealidades opcionales**: null offset, histéresis del carrete, saturación de
  caudal, zona muerta, asimetría de ganancia por signo, fricción de sellos, y contacto
  elástico con la probeta (para el lazo de fuerza).

No es la planta real. Es el banco donde se comprueba que el NARX y el neurocontrolador
funcionan sobre una planta con esta estructura, y donde se prueba la secuencia de
excitación antes de lanzarla contra un cilindro de 200 kN.

---

### 5.7 Fase 1 — modelo NARX, línea base (2026-08-13)

`tools/nn_modelo.py`. Red de una capa oculta escrita a mano con numpy, entrenada por Adam
con selección por reinicios sobre **simulación libre en datos no vistos**. Predice el
**incremento**, no la posición (§3.3).

**(a) El Ts del modelo no es el del control, y hay una razón medida.** El ruido de posición
(σ = 0.105 mm, §5.2) baja como √N al diezmar promediando, pero el incremento por muestra
crece con Ts. La relación señal/ruido del incremento a 1 mm/s:

| Ts | σ_y | \|dy\| a 1 mm/s | SNR |
|---|---|---|---|
| **20 ms** (el del control) | 0.0235 mm | 0.020 mm | **0.60** ← bajo el ruido |
| 50 ms | 0.0148 mm | 0.050 mm | 2.38 |
| **100 ms** (elegido) | 0.0105 mm | 0.100 mm | **6.73** |
| 200 ms | 0.0074 mm | 0.200 mm | 19.05 |

A 20 ms el incremento está **por debajo del ruido** justo en la banda de los ensayos. No se
pierde nada identificando más lento: a 50 Hz la servoválvula (120 Hz) y la resonancia
(>315 Hz) ya están sobre Nyquist.

**(b) ⚠ RESULTADO PRINCIPAL: el mejor modelo NO tiene realimentación.** Barrido de cuánta
historia de `dy` conviene realimentar:

| ny | un paso | libre train | **libre val** |
|---|---|---|---|
| **0** | 51.5 % | 51.5 % | **49.6 %** ← el mejor |
| 1 | 56.1 % | −133.5 % | **−440.9 %** ← diverge |
| 2 | 62.2 % | 37.8 % | 41.8 % |
| 3 | 62.1 % | 33.9 % | 31.9 % |
| 5 | 63.0 % | 44.1 % | 41.8 % |

**Cada `dy` realimentado mejora el ajuste a un paso y empeora la simulación libre.** La
causa es medible: `dy` lleva el ruido del sensor y `u` no lleva ninguno. Entrenando a un
paso, la red descubre que puede usar el ruido de `dy(k)` para predecir parte del ruido de
`dy(k+1)` — eso sube el «un paso» — pero en simulación libre esa entrada ya no es el ruido
medido sino **el error de la propia red**, que se realimenta y crece.

Y encaja con la física: a 100 ms no queda estado interno que recordar. **No hay nada que
realimentar.** El modelo adoptado es un **FIR no lineal**:

```
dy(k+1) = f( u(k), u(k−1) )        red 2-15-1
```

que además **no puede acumular error por construcción**: «un paso» y «simulación libre» son
la misma cuenta.

**(c) Resultados de la línea base:**

| | un paso | **sim. libre** | R² | sesgo | techo por ruido | alcanzado |
|---|---|---|---|---|---|---|
| train | 51.7 % | **51.7 %** | 0.767 | −0.40 mm/min | 65.1 % | **79 %** |
| val | 50.1 % | **50.1 %** | 0.751 | **+1.18 mm/min** | 57.7 % | **87 %** |

Error máximo de posición reintegrando en ventanas de 60 s: mediana **0.58 mm** (train) y
**1.13 mm** (val).

> **El 50 % hay que leerlo contra el techo, no contra el 100 %.** El techo es la fracción
> del incremento que **es ruido de medida**: ningún modelo puede superarlo. Se alcanza el
> **87 % de lo alcanzable** en validación. Y da una consecuencia accionable: **bajar el
> ruido de posición sube el techo directamente** — el tono de modo común a 133.8 Hz (§5.2b)
> deja de ser una curiosidad y pasa a ser la palanca de mayor retorno del proyecto.

**(d) ✅ El sesgo de validación ES la deriva del null, y coincide al 3 %.**

| | |
|---|---|
| Predicho desde el desplazamiento de 71 mV (§5.6c) | **1.581 mm/min** |
| Sesgo medido del modelo en validación | **1.632 mm/min** *(1.18 en la corrida final)* |

En `train` el sesgo es ≈ 0, como debe ser al evaluar sobre la misma planta con la que se
entrenó. **Confirma que no es sobreajuste ni falta de capacidad**: es que `train` y `val`
son plantas ligeramente distintas, y el modelo hereda la diferencia como sesgo constante.
Era exactamente lo que la opción (1) tenía que dejar a la vista.

**(e) Más capacidad empeora las cosas.** Con 30 neuronas y 400 épocas el «un paso» sube a
66.1 % y la **simulación libre cae a 17–22 %**. Es la misma trampa de (b) amplificada:
optimizar el criterio equivocado. **La métrica que manda es la simulación libre**, y hay
que seleccionar por ella.

---

## 6. Metodología de trabajo

### 6.1 Fase 0 — caracterizar la cadena antes de capturar nada
- **Verificar sin mover la planta** (UPH apagada, AO a 0): enumerar módulos, leer los 4 AI
  en reposo, comprobar el cero de cada sensor y el ruido de fondo. `tools/daq.py --diag`.
- **Identificación → temporización por HARDWARE** (§2.5.1): `Ts` exacto, comando y medidas
  alineados por el mismo trigger. No hay jitter que medir aquí; sí hay que **verificar que
  no se pierden muestras** (el `overflow`/`overwrite` del buffer sobre Ethernet).
- **Control → temporización por SOFTWARE**: aquí sí hay que medir **la latencia de ida y
  vuelta AO→AI del cDAQ por Ethernet** y el jitter del lazo (min/media/máx/σ) durante varios
  minutos. Ese número fija el `Ts` de control alcanzable. Un retardo de transporte no
  modelado se le aparece al NARX como dinámica falsa, así que debe **medirse y luego
  incluirse en el modelo** como retardo puro.
- **LATENCIA DEL LAZO — medida el 2026-08-13** (`tools/daq.py --latencia`, 2000
  iteraciones, AO sostenido a 0 V):

  | | min | mediana | p95 | máx |
  |---|---|---|---|---|
  | Lectura AI | 2.32 | **2.80** | 3.14 | 7.46 ms |
  | Escritura AO | 2.19 | **2.65** | 3.03 | 6.61 ms |
  | **Total (leer + escribir)** | 4.69 | **5.46** | 6.11 | **9.86 ms** |

  σ = 0.40 ms. **Ts de control = 20 ms (50 Hz)** deja la mitad del periodo libre
  incluso en el peor caso medido. Es el mismo Ts que `pi5_qnx_project`, lo que
  facilita comparar. Ese ~5.5 ms es además el **retardo puro** que hay que incluir
  en el modelo NARX: un retardo de transporte no modelado se le aparece a la red
  como dinámica falsa.

  > ⚠ **Trampa que costó un factor 37.** La primera medida dio **202 ms** de mediana.
  > No era el enlace: la tarea de AI no se arrancaba explícitamente, así que DAQmx
  > hacía un **start/stop implícito en cada `read()`**, y sobre un chasis Ethernet eso
  > es carísimo. Con `TASK_COMMIT` + `start()` una sola vez, la misma medida da
  > 5.46 ms. Si alguna vez el lazo va inexplicablemente lento, mirar esto primero.

  > **Lo que 50 Hz NO captura, dicho explícitamente:** la servoválvula está en 120 Hz
  > y la resonancia hidráulica entre 315 y 530 Hz (§5.4), ambas **por encima de
  > Nyquist**. El modelo a Ts = 20 ms las trata como instantáneas. Para el lazo de
  > posición es razonable —quedan fuera del ancho de banda de control— pero **no se
  > puede afirmar que el modelo las contenga**.

- **Confirmar la ganancia real de la cadena**: `K_amp` del amplificador, el ajuste de null
  del carrete y la ganancia mm/s por voltio. Es lo que zanja la discrepancia de 6.5× (§5.1).
- Decidir el **Ts de trabajo**. Referencias: ω_sv = 150 Hz y ω_h = 225 Hz. Muestrear a
  10 ms (100 Hz) **no captura la resonancia hidráulica**; para el lazo de posición eso
  puede estar bien (la resonancia queda fuera del ancho de banda de control), pero hay que
  decirlo explícitamente y no fingir que el modelo la contiene. Como la captura va por
  hardware, **se puede muestrear rápido (p. ej. 1–5 kHz) y diezmar después**: es gratis y
  permite decidir el Ts del modelo *a posteriori*, con los datos delante.

### 6.1.1 El punto de operación de los ensayos está DENTRO de la zona de null
Consecuencia que condiciona todo el diseño de la excitación. Las normas piden
**1.5 mm/min (losa) y 0.1 mm/min (viga)**, es decir **0.025 y 0.0017 mm/s**. Con la
ganancia de velocidad de la cadena:

| Ganancia usada | 1.5 mm/min | 0.1 mm/min |
|---|---|---|
| Teórica (0.975 mm/s·V) | **0.026 V** = 0.26 % del AO | **0.0017 V** = 0.017 % |
| Medida en 2017 (0.15 mm/s·V) | **0.17 V** = 1.7 % | **0.011 V** = 0.11 % |

O sea: **el ensayo real ocurre entre el 0.02 % y el 2 % del rango del comando** — justo
donde viven el solape del carrete, la histéresis, el offset de null y el *stick-slip* de
los sellos. Es la región **más no lineal y peor condicionada** de toda la planta, y la que
un PID lineal peor cubre. **Ahí es donde una red neuronal tiene algo que aportar**, y es el
argumento central del proyecto.

Implicaciones directas:
1. La excitación **no puede repartir amplitudes de forma uniforme**: debe ser densa en
   amplitudes pequeñas (reparto ~logarítmico), o el modelo será excelente donde no importa
   e inútil donde se ensaya.
2. El **AO de 16 bits sobre ±10 V da 0.3 mV de resolución**; a 0.0017 V de comando eso son
   ~5 LSB. Se puede, pero está justo. Verificar en la Fase 0 si conviene **reducir el rango
   efectivo** (p. ej. reconfigurar la ganancia del amplificador) para ganar resolución.
3. El ruido del AI y la resolución del sensor MTS acotan la velocidad mínima medible:
   a 0.0017 mm/s el vástago recorre **0.1 mm en un minuto**. La medida de velocidad hay que
   sacarla por ajuste sobre ventana larga, no por diferencia entre muestras consecutivas.

### 6.2 Captura para identificación (`tools/gen_excitacion.py`)
Traslada la lección del proyecto del motor: **la calidad del modelo la fija la excitación**.
- **APRBS multinivel con signo** (amplitud y duración pseudoaleatorias, semilla fija ⇒
  reproducible): niveles repartidos entre zona de null, régimen bajo y fondo de escala, en
  **ambos sentidos y con inversiones**, todo en una sola sesión.
- **Duración de los tramos:** allá la primera captura falló por tramos demasiado cortos
  (solo 29 % de muestras asentadas) y hubo que recapturar. Aquí el criterio es distinto
  porque **la planta no "asienta" en posición** (es integradora): lo que asienta es la
  **velocidad**, en ~4·τ_sv ≈ 4 ms. El límite real de la duración del tramo es el
  **recorrido**: 400 mm de carrera a 0.7 mm/s se agotan en 9 minutos. La secuencia debe
  **plegarse dentro de una ventana de posición segura**, invirtiendo el signo cuando se
  acerca a un extremo.
- **Dos capturas independientes** con semillas distintas → `train` y `val`. No partición
  temporal de una sola serie: es una validación más honesta.
- Registrar **todos** los canales (posición, fuerza, ambas presiones, comando) con marca de
  tiempo por muestra.

### 6.3 Seguridad — no negociable
Esta planta puede aplicar **200 kN** y mover 150 kg con **δ_h ≈ 0**. Toda secuencia
automática debe llevar, en el propio VI y antes de cualquier prueba con la red:
- **Límites de posición por software** con margen respecto a los topes mecánicos (la carrera
  útil del sensor es 400 mm; definir una ventana de trabajo estrictamente interior).
- **Límite de fuerza** con corte automático muy por debajo de los 200 kN de la celda.
- **Watchdog**: si el lazo pierde una iteración o el enlace del cDAQ cae, el AO va a null.
- **Identificación en lazo abierto SIN probeta** (vástago libre), como se hizo en 2017.
- Seta de emergencia verificada (ya cableada al NI 9421/9472).

### 6.4 Fases (metodología heredada del proyecto del motor)
1. **Fase 0** — caracterizar el lazo (§6.1) y verificar `K_amp`, el null y la ganancia real.
2. **Fase 1** — captura APRBS train/val → **modelo NARX sobre incrementos** (§3.3),
   métrica = **simulación libre**, nunca "un paso adelante".
3. **Fase 2** — **neurocontrolador** entrenado por BPTT contra el modelo NARX.
4. **Fase 3** — despliegue en LabVIEW y **comparación de leyes en la misma sesión**:
   PID actual · PID+feedforward del modelo · **PID+red (mezcla)** · red sola.
   *En el proyecto del motor la mezcla PID+red fue la ganadora clara (6–8× menos
   sobreimpulso con error de régimen cero). Es la hipótesis de partida aquí también.*
5. **Fase 4** — extender a los modos de fuerza y rampa (§1).

⚠ **Regla de metodología (aprendida a golpes en el proyecto del motor): comparar SIEMPRE
en la misma sesión.** Allá el mismo PID dio 31 % / 135 % / 34 % de sobreimpulso en tres
sesiones distintas por la temperatura del motor. Aquí el equivalente es la **temperatura y
la viscosidad del aceite**: la UPH tiene indicador de temperatura por algo. Registrar la
temperatura del aceite en cada captura y en cada corrida de comparación.

---

## 7. Inventario de archivos

**Estructura:** `docs/` (memoria original y protocolos) · `labview/` (VIs y datos históricos, tal como
llegaron) · `tools/` (Python del host) · `results/` (datos y figuras del informe) ·
`results/` (datos y figuras del informe).

| Archivo | Tipo | Qué hace |
|---|---|---|
| `tools/planta_sim.py` | python | simulador físico del servo-hidráulico de **dos cámaras** (§5.5): servoválvula + dinámica hidráulica + integrador + no linealidades opcionales |
| `tools/gen_excitacion.py` | python | diseña la secuencia APRBS de captura (§6.2) con plegado dentro de la ventana de posición segura; la valida contra el simulador y exporta el CSV que se precarga en el AO |
| `docs/protocolo_fase0.md` | protocolo | checklist de la primera sesión de máquina: qué medir, en qué orden y qué anotar |
| `tools/nn_modelo.py` | python | modelo de la planta con red neuronal (§5.7): predice incrementos, se valida en simulación libre y reporta el techo que impone el ruido |
| `tools/daq.py` | python | capa de E/S sobre `nidaqmx`: `--diag`/`--di`/`--sensores` (solo lectura), `--hpu` (arranque de la UPH), `--caracteriza` (curva comando→velocidad), `--jog` (recolocación), `--latencia`, y la captura AO+AI temporizada por hardware con trigger común (§2.5.1) |
| `docs/Moog-ServoValves-761Series-Catalog-en.pdf` | ref. | catálogo Moog 761 (Rev. M, 2024). Identifica la válvula **G761-3001B H04JOFM4VPL** y da sus curvas y tolerancias reales (§2.2) |
| `docs/Memoria…FLUIDTEK.pdf` | ref. | memoria original de Fluidtek (2017), 255 pp. **Desactualizada** respecto al equipo actual: verificar contra el hardware antes de fiarse de un número |
| `labview/DISENO/` | LabVIEW | proyecto original: `FluidtekPrensaCONident.vi`, `FluidtekPrensaCONidentLEDI.vi`, `Identi.vi`, `HPU.vi`, `LVDT.vi`, `ALMACENAMIENTO.vi`, `DeltaP.vi`… |
| `labview/VI's LEDI/` | LabVIEW | VIs añadidos por el laboratorio: `genLEDI.vi`, `proflu.vi` |
| `labview/Nueva carpeta/` | datos | capturas de 2017 (`.xls` tabulados, `.lvm`), manuales del equipo (`manualSISTEMAhidraulico.docx`) y capturas de pantalla |

---

## 8. Limitaciones y riesgos conocidos

- **El lazo CERRADO no es de tiempo real** (Windows + cDAQ por Ethernet). La *captura* sí
  queda resuelta con temporización por hardware (§2.5.1), pero el control punto a punto
  arrastra la latencia del enlace y el jitter del SO — un problema que en `pi5_qnx_project`
  simplemente no existía. Hay que medirlo (§6.1) y reportarlo, no ignorarlo.
- **Reserva de dispositivo:** LabVIEW y Python no pueden tener el mismo módulo a la vez
  (§2.5.1). Definir quién gobierna el NI 9472/9421 (arranque de UPH y enclavamientos)
  durante una captura desde Python.
- **La memoria de 2017 está desactualizada** (lo advirtió el usuario). Ya se han encontrado
  cuatro discrepancias: **carrera 150 mm** (no 400), **presiones con span 2–10 V y rangos
  100/400 bar** (no 0–10 V / 160 bar), **cilindro asimétrico** (no simétrico con
  `A_p = 122.52 cm²`), y el uso de `K_ce` como fuga (§5.4). Quedan por confirmar: `K_amp`,
  la presión de trabajo real, el Ts del lazo y el estado del ajuste de null.
- [x] ~~**Discrepancia de ganancia**~~ — **cerrada del todo** (§5.4). Recorrido completo:

  | Modelo | Predice | vs medido (0.446 mm/s·V) |
  |---|---|---|
  | Memoria: cilindro simétrico, 4.78 L/min, 150 Hz | — | **factor 6.5×** |
  | + dos cámaras asimétricas | 0.902 | factor 1.65× |
  | + catálogo de la válvula (4 L/min, ±40 mA) | 0.377 | −15 % |
  | **+ amplificador real (10 V → 40 mA)** | **0.503** | **+13 %** |

  El 13 % restante cae **dentro de la tolerancia de caudal del propio fabricante (±10 %)**,
  sin contar la presión de suministro real, el coeficiente de descarga ni la viscosidad del
  aceite. **El modelo físico ya no tiene ningún error estructural pendiente**; lo que queda
  es incertidumbre de parámetros, que es lo que la red va a aprender de los datos.
- **Offset de null grande (≈ −0.3 V) y deriva de 8.4 mm/min a comando cero** (§5.3b): el
  punto de trabajo de los ensayos normados está pegado al cruce por cero y a caballo del
  cambio de rama. Es la dificultad central del proyecto y su principal justificación. La primera medida
  (8.35 mm/min) está contaminada por el transitorio de arranque de la bomba. Re-medir con
  la presión estabilizada.
- [x] ~~**Escala de las presiones sin confirmar**~~ — **calibrada** (§5.3c): el balance de
  fuerzas pasó de −141 kN a +0.8 kN de residuo. Queda una duda menor: los fondos de escala
  deducidos (259 y 146 bar) no coinciden con los 0–100/0–400 bar reportados.
- **Datos históricos insuficientes** para identificación: se requiere captura nueva.
- **El lazo de fuerza es intrínsecamente más difícil**: la rigidez de la probeta entra en
  la dinámica y **cambia de golpe al fracturarse**. Un modelo entrenado con la probeta
  intacta deja de ser válido en el instante más crítico del ensayo. Tratarlo como un caso
  aparte, no como "lo mismo pero con otra señal".
- **Riesgo físico real:** 200 kN y 100 bar. §6.3 no es opcional.

---

## 9. Decisiones tomadas (no reabrir sin motivo)

- **Plataforma = el cDAQ-9184 existente** (2026-08-12). No se migra a un target RT: el
  objetivo es el control neuronal, no rehacer la cadena de E/S.
- **Acceso al cDAQ desde Python con `nidaqmx`** (2026-08-12, tras verificar que el driver y
  el chasis responden — §2.5.1). Revisa la decisión inicial de hacerlo todo en LabVIEW:
  - **Captura de identificación → Python + DAQmx, temporizada por hardware.** Da `Ts`
    exacto y alineación comando/medida por trigger, que es justo lo que un NARX necesita
    y lo que un lazo de software en cualquier lenguaje no puede garantizar.
  - **Entrenamiento y análisis → Python** (ya lo era).
  - **Lazo de control → por decidir en la Fase 0**, según la latencia que se mida: Python
    punto a punto, o el VI de LabVIEW con los pesos exportados. Se decide con el número
    delante, no antes.
  - **LabVIEW se conserva** como HMI de operación, para los ensayos normados y para los
    enclavamientos ya validados. No se toca lo que hoy funciona y es de seguridad.
- **Cuatro modos de control** (2026-08-12): posición (aproximación), fuerza (precarga),
  rampa de fuerza kN/min y velocidad mm/min (ensayo). Se ataca **posición primero**: es el
  lazo que hoy existe y por tanto el único con línea base contra la que comparar.
- **Se capturan datos nuevos** (2026-08-12): los de 2017 no alcanzan (§5.1).
- **El NARX predice incrementos, no posición absoluta** (§3.3). Decisión estructural.
- **Métrica de modelo = simulación libre**, no predicción a un paso.
- **Identificación en lazo abierto y sin probeta.**

---

## 10. Convenciones para Claude Code

- Responder en **español**.
- **Python en el host** (numpy/matplotlib) para identificación, entrenamiento y análisis;
  **LabVIEW en el equipo** para E/S y para el lazo. No proponer C/RTOS: esa fue la decisión
  del §9.
- **Redes implementadas a mano con numpy** (como en `pi5_qnx_project`), no con
  TensorFlow/PyTorch: el despliegue final es un puñado de pesos dentro de un VI, y el
  proyecto es didáctico — el gradiente tiene que poder escribirse en el informe.
- **Signo:** el comando y la velocidad llevan signo (+ = extensión del vástago; confirmar
  el criterio físico en el equipo y fijarlo aquí en cuanto se verifique).
- Unidades: **mm** para posición, **kN** para fuerza, **bar** para presión, **V** para el
  comando del AO (y **%** solo si se documenta la conversión `K2 = 0.1 V/%`).
- Antes de dar por bueno un número de la memoria, comprobar si §8 lo marca como dudoso.
