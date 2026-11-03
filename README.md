# Prensa servohidráulica Fluidtek UPH 50 — LEDI-PUCP

Control, modelado e identificación de la prensa de ensayo de materiales del Laboratorio
de Estructuras de la PUCP: cilindro vertical gobernado por una servoválvula Moog G761,
unidad hidráulica UPH 50 y un NI cDAQ-9184 por Ethernet, con LabVIEW como software de
operación.

Toda la documentación técnica —hardware, E/S, modelo, resultados, estado y pendientes—
está en **[`CLAUDE.md`](CLAUDE.md)**. El registro detallado de las mediciones está en
[`.claude/skills/resultados-medidos/SKILL.md`](.claude/skills/resultados-medidos/SKILL.md).

| Carpeta | Contenido |
|---|---|
| `labview_control_software/` | VI de control en uso (v15) |
| `simulink/` | modelo físico (Jelali & Kroll 2003) y sus parámetros |
| `diagrama_electrico/` | diagrama del tablero en CADe_SIMU |
| `docs/` | memoria de Fluidtek 2017 (desactualizada) y referencias |

## ⚠ Seguridad

El cilindro aplica hasta **200 kN** sobre una planta casi sin amortiguamiento. Ninguna
maniobra sin alguien junto a la seta de emergencia, y la seta, la limitadora y los
finales de carrera deben estar operativos: los límites por software no los sustituyen.
