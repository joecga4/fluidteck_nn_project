function P = params_prensa(varargin)
%PARAMS_PRENSA  Parametros del modelo Simulink de la prensa Fluidtek UPH 50 (PUCP).
%
%   P = params_prensa()                 valores por defecto
%   P = params_prensa('Qp', 3.18/60000) cualquier campo se puede sobrescribir
%
%   Modelo fisico de Jelali & Kroll (2003), "Hydraulic Servo-systems",
%   cap. 3-4: servovalvula de 4 vias con ecuaciones de orificio (4.5-4.6),
%   dinamica de 2.o orden del carrete (4.9), presiones en las camaras con
%   modulo de compresibilidad efectivo de Lee (3.21 / 4.184-4.185), volumenes
%   variables con la carrera (4.46-4.47), fuga interna laminar (4.55),
%   friccion de Stribeck (4.60) y fuente de presion con limitadora (4.2.4).
%
%   ORIGEN DE CADA NUMERO (mismo criterio que tools/planta.py):
%     MEDIDO    lo dice el equipo; manda sobre lo demas
%     AJUSTADO  ajustado a las curvas comando->velocidad medidas
%               (results/caracterizacion_2026-08-12.csv y _0-10V_2026-08-21.csv)
%     CATALOGO  Moog G761-3001B H04JOFM4VPL (docs/Moog-...pdf)
%     MEMORIA   memoria Fluidtek 2017 (DESACTUALIZADA)
%     JELALI    valor tipico del libro
%     SUPUESTO  sin dato: valor razonable, a identificar
%
%   Convenio de signos: x = 0 vastago totalmente retraido; x > 0 extiende el
%   vastago HACIA ABAJO (montaje vertical), a favor de la gravedad. u > 0 extiende.
%   Unidades internas SI (m, m/s, Pa, m^3/s, N).

P = struct();

%% 1. Geometria del actuador (MEMORIA + laboratorio)
P.D      = 0.125;     % [m] diametro del embolo                         PROVISIONAL (2026-10-31)
%   125/80 mm es la geometria que cierra a la vez: vastago medido (80 mm), placa
%   de la UPH (1.7 L/min) y el balance de fuerzas de results/saturacion.csv
%   (PB/PA = 1.64-1.70 en ambos sentidos). La memoria dice 160/100 y el tubo mide
%   ~190 mm por fuera: VERIFICAR el diametro interior (placa del cilindro o prueba
%   de fuerza contra la celda: F_celda/PA = A_A).
P.d      = 0.080;     % [m] diametro del vastago (79.9-80.0 mm, 2026-10-31) MEDIDO (lab)
P.L      = 0.150;     % [m] rango util (VI y Tempsonic: 0-150 mm)       MEDIDO (lab)
P.L_fis  = 0.300;     % [m] carrera fisica del cilindro (tope)          MEDIDO (lab)
%   El volumen de la camara B y el tope usan L_fis; x = 0 vastago retraido.
P.VA0    = 0.3e-3;    % [m^3] volumen muerto camara A (manifold)         SUPUESTO
P.VB0    = 0.3e-3;    % [m^3] volumen muerto camara B (manifold)         SUPUESTO
%   La servovalvula va sobre un manifold montado en el cilindro (memoria
%   sec. 2.2): no hay mangueras entre valvula y camaras, solo pasajes cortos.

%% 2. Masas y gravedad
P.m_movil    = 150;   % [kg] embolo + vastago                           MEMORIA
P.m_colgante = 28;    % [kg] celda de carga en la punta                 MEDIDO (lab)
P.g          = 9.81;  % [m/s^2]
P.vertical   = 1;     % 1 = montaje vertical, vastago hacia abajo       MEDIDO (lab)

%% 3. Servovalvula Moog G761-3001B H04JOFM4VPL  (Jelali ec. 4.9)
P.w_v   = 2*pi*120;   % [rad/s] -3 dB a 120 Hz (version ..F)            CATALOGO
P.z_v   = 0.7;        % [-] amortiguamiento                             CATALOGO/JELALI
P.K_amp = 0.004;      % [A/V] amplificador 10 V -> 40 mA                MEDIDO (lab)
P.i_nom = 0.040;      % [A] corriente nominal (codigo L, paralelo)      CATALOGO
P.u_max = 10;         % [V] rango del AO NI 9263
P.Q_nom = 4.0/60000;  % [m^3/s] 4 L/min a 35 bar por arista             CATALOGO
P.dP_nom = 35e5;      % [Pa] caida nominal por arista                   CATALOGO
P.kQ    = 0.505;      % [-] factor sobre el caudal nominal              AJUSTADO 2026-10-31 (0.828 x 122.7/201.1, valida RMS 0.04/0.11 mm/s)
%   (el catalogo da +-10 %; el ajuste pide -17 %: incluye la viscosidad
%   real del aceite y la presion de suministro, que no se conoce con certeza)
P.x0    = 0.0198;     % [-] offset de null del carrete (1 = 100 %)      AJUSTADO
%   Equivale a ~0.2 V de comando. Moog admite <= 2 % de deriva por 55 C,
%   y el null medido se movio 71 mV en 10 min (CLAUDE.md sec. 5.0): NO es
%   constante, varia con la temperatura del aceite.
P.xu    = 0.0111;     % [-] sobrecubrimiento negativo (underlap)        AJUSTADO
%   Jelali ec. 4.5-4.6. Representa la fuga del "centro critico practico"
%   (holgura radial). Es la que deja caer el peso a comando nulo.
P.p_lam = 1e5;        % [Pa] transicion laminar-turbulenta del orificio  SUPUESTO
%   sqrt(|dp|) se reemplaza por dp/sqrt(|dp|+p_lam): turbulento para
%   |dp| >> 1 bar, laminar cerca de 0 (evita la derivada infinita).
P.histeresis = 0;     % [-] ancho de histeresis del carrete (<=0.03)    CATALOGO (desact.)

%% 4. Fuente de presion (Jelali 4.2.4): bomba + limitadora + acumulador + manguera
P.supply_dyn = 1;     % 1 = presion de suministro dinamica, 0 = Ps constante
P.Ps0   = 100e5;      % [Pa] presion de suministro / tarado limitadora   MEMORIA
P.Ps_ini = P.Ps0;    % [Pa] presion de suministro inicial (acumulador cargado)
P.PT    = 2e5;        % [Pa] presion de retorno (manguera 14 m ~1.4 bar) MEMORIA
P.Qp    = 1.7/60000;  % [m^3/s] caudal de la bomba                       PLACA UPH 50 (02/2015)
%   Prueba de saturacion 2026-10-31 (results/saturacion.csv): a +-10 V la presion
%   de suministro cae a ~47 bar y la velocidad decae al vaciarse el acumulador;
%   con 125/80 el caudal implicito queda entre 1.2 y 1.9 L/min, coherente con la
%   placa. Los 3.92 mm/s de la curva 08-21 los aportaba el acumulador.
P.dP_rv = 2e5;        % [Pa] sobrepresion de la limitadora a caudal Qp  SUPUESTO
P.V_s   = 0.6e-3;     % [m^3] manguera P (7.5 m, DI 9.5 mm) + manifold  MEMORIA
P.V_acc = 0.75e-3;    % [m^3] volumen del acumulador de vejiga           SUPUESTO
P.p_acc0 = 60e5;      % [Pa] precarga de nitrogeno                       SUPUESTO
P.n_acc = 1.4;        % [-] exponente politropico                        JELALI

%% 5. Fluido: modulo de compresibilidad efectivo de Lee (Jelali 3.21)
P.a1 = 0.5; P.a2 = 90; P.a3 = 3;        %                               JELALI
P.E_max = 18000e5;    % [Pa]                                             JELALI
P.p_max = 280e5;      % [Pa]                                             JELALI
%   E'(p) = a1*Emax*log10(a2*p/pmax + a3): 0.50e9 Pa a 2 bar, 1.39e9 a 100 bar.
%   La memoria usa 1.7e9 constante.
P.C_Li = 0;           % [m^3/(s*Pa)] fuga interna entre camaras (4.55)  SUPUESTO (0)

%% 6. Friccion de Stribeck (Jelali 4.60)   F = s*v + sgn(v)*(Fc + Fs*exp(-|v|/cs))
P.sigma = 6e4;        % [N*s/m] viscosa                                  SUPUESTO
P.F_c   = 500;        % [N] Coulomb                                      SUPUESTO
P.F_s   = 300;        % [N] estatica adicional (breakaway = Fc + Fs)     SUPUESTO
P.c_s   = 5e-3;       % [m/s] velocidad de Stribeck                      SUPUESTO
P.v_eps = 1e-5;       % [m/s] suavizado de sgn(v) con tanh               numerico
%   El residuo del balance de fuerzas medido fue +0.8 kN (CLAUDE.md sec. 5.0),
%   del orden de Fc + Fs. Identificar con rampas lentas en ambos sentidos.
%   OJO: con Fs/c_s > sigma la pendiente de Stribeck es negativa y aparece
%   stick-slip (ciclo limite a la frecuencia hidraulica). Con Fs = 800 N y
%   c_s = 1 mm/s se observo en simulacion; estos valores lo evitan a 0.5 mm/s.

%% 7. Topes mecanicos y probeta
P.k_stop = 1e9;       % [N/m] tope elastico (penalizacion)               numerico
P.c_stop = 1e6;       % [N*s/m]                                          numerico
P.k_p    = 0;         % [N/m] rigidez de la probeta (0 = sin probeta)
P.x_c    = 0.100;     % [m] posicion de contacto con la probeta
P.F_rot  = 1e12;      % [N] fuerza de rotura (la rigidez desaparece)

%% 8. Sensores y adquisicion (MEDIDO, CLAUDE.md sec. 2.4 y sec. 5.0)
P.Kx_sens = 15;       % [mm/V] posicion 0-150 mm sobre 0-10 V
P.KF_sens = 20;       % [kN/V] celda (Krenel 0-10 V; LabVIEW usa MEAN x 20)
P.pA_off  = 1.9697;   % [V] canal ai2 a presion nula
P.pA_esc  = 32.293;   % [bar/V]
P.pB_off  = 1.9801;   % [V] canal ai3 a presion nula
P.pB_esc  = 18.163;   % [bar/V]
P.ruido_on = 0;       % 1 = anade el ruido medido de los sensores
P.sig_x  = 0.105;     % [mm]  sigma posicion a 1 kHz                     MEDIDO
P.sig_F  = 0.422;     % [kN]                                             MEDIDO
P.sig_pA = 0.071;     % [bar]                                            MEDIDO
P.sig_pB = 0.289;     % [bar]                                            MEDIDO
P.Ts_daq = 1e-3;      % [s] muestreo del NI 9222 en las capturas         MEDIDO
P.latencia = 5.46e-3; % [s] lazo AI+AO por Ethernet (mediana)            MEDIDO
P.Ts_ctrl  = 0.020;   % [s] periodo del lazo de control

%% 9. Condiciones iniciales y simulacion
P.x_ini  = 0.075;     % [m] posicion inicial
P.PA_ini = 30e5;      % [Pa] presiones iniciales (cerca del equilibrio en reposo)
P.PB_ini = 52e5;
P.t_fin  = 10;        % [s]

%% Sobrescrituras por argumento
for k = 1:2:numel(varargin)
    P.(varargin{k}) = varargin{k+1};
end

%% Derivados
P.A_A  = pi/4*P.D^2;                 % 201.06 cm^2  camara sin vastago
P.A_B  = pi/4*(P.D^2 - P.d^2);       % 122.52 cm^2  camara anular
P.m    = P.m_movil + P.m_colgante;   % masa en movimiento
P.W    = P.vertical*P.m*P.g;         % peso, en el sentido +x
P.Cv   = P.kQ*P.Q_nom/sqrt(P.dP_nom);% [m^3/(s*sqrt(Pa))] por unidad de carrete
P.K_rv = P.Qp/P.dP_rv;               % pendiente de la limitadora
P.u2xv = P.K_amp/P.i_nom;            % [1/V] comando -> carrete normalizado
end


