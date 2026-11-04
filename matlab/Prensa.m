classdef Prensa < handle
%PRENSA  Acceso a la prensa servohidraulica UPH 50 desde MATLAB (cDAQ-9184).
%
%   Port de tools/planta.py + tools/daq.py. Un solo objeto lleva los numeros de
%   la planta (propiedades Constant), el acceso al chasis y las maniobras.
%
%   ####################  SEGURIDAD — LEER ANTES DE USAR  ####################
%   Esto puede MOVER un cilindro de 200 kN a 100 bar con amortiguamiento casi
%   nulo (delta_h = 1.4e-3). Un comando equivocado en el AO rompe algo.
%     * NADA escribe en el AO ni en el DO si no se pasa 'Armar',true. Sin eso
%       cada metodo hace un ENSAYO EN SECO (comprueba y avisa, no actua).
%     * diag, sensores, di y lee son de SOLO LECTURA. Empezar siempre por ahi.
%     * lazoAbierto y las maniobras vigilan posicion/fuerza/presion y abortan a 0.
%     * NO sustituye a la seta, la limitadora ni los finales de carrera.
%   ##########################################################################
%
%   SESION TIPICA (LabVIEW y NI MAX CERRADOS: reservan los modulos)
%
%       p = Prensa();                         % localiza modulos (solo lectura)
%       p.diag();                             % 4 AI + fuerza por 2 caminos
%       p.sensores(10);                       % lectura en vivo
%       p.hpu(true, 'Armar', true);           % arranca la UPH (AO a 0 antes)
%       pause(Prensa.CALENTAMIENTO);          % ESPERAR 3 MIN: transitorio (§5.9c)
%       p.jog(75, 'Armar', true);             % recolocar el vastago
%       p.escribe(Prensa.U_NULL, 'Armar', true);   % comando de velocidad nula
%       r = p.lazoAbierto(u, 'Ts', 1e-3, 'Etiqueta','train', 'Armar',true);
%       p.hpu(false, 'Armar', true);          % para la UPH, AO a 0
%
%   Todo en un solo proceso con arranque/parada garantizados:
%
%       p.conHpu(@() p.caracteriza('Armar',true), 'Armar', true);
%
%   LA PLANTA EN LAZO ABIERTO tiene dos puertas con la misma forma de salida:
%       real       r = p.lazoAbierto(u, ...)            (cDAQ)
%       simulada   r = Prensa.simula(u, Ts)             (simulink/prensa_fluidtek.slx)
%   Campos comunes: t, u, posicion [mm], fuerza [kN], presion_A/B [bar].
%   Diseno de la secuencia (sin hardware): Prensa.excitacion / informe /
%   validaCinematica / guardaCsv; Prensa.grafica dibuja cualquiera de los dos.
%
%   Datos que mandan (CLAUDE.md §5.0): velocidad nula en U_NULL = -0.370 V, NO
%   en 0; el null deriva con la temperatura del aceite (71 mV en 10 min);
%   esperar 3 min tras arrancar la bomba; poner el AO a 0 ANTES de energizarla.

    %% ====================================================================
    %  1. LOS NUMEROS DE LA PLANTA
    %  Fuente unica: simulink/params_prensa.m (geometria, masas, caudal de la
    %  bomba, escalas de sensor, ruido, Ts). Aqui solo se DERIVAN de P, salvo lo
    %  que params_prensa no tiene: la ley comando->velocidad MEDIDA, los limites
    %  de seguridad y los pines digitales.
    %% ====================================================================
    properties (Constant)
        P = Prensa.cargaParams();            % struct de params_prensa()

        % --- geometria y masas (de P) ---------------------------------------
        D_PISTON  = Prensa.P.D;              % [m]
        D_VASTAGO = Prensa.P.d;              % [m]
        L_CARRERA = Prensa.P.L;              % [m] carrera util
        A_A = Prensa.P.A_A;                  % [m2] camara SIN vastago
        A_B = Prensa.P.A_B;                  % [m2] camara CON vastago
        REL_AREAS = Prensa.P.A_A/Prensa.P.A_B;
        M_MOVIL = Prensa.P.m_movil;          % [kg]
        M_COLGANTE = Prensa.P.m_colgante;    % [kg] celda de carga
        G = Prensa.P.g;
        PESO = Prensa.P.m*Prensa.P.g;        % [N], hacia +

        % --- ley comando -> velocidad (MEDIDA 2026-08-13) -------------------
        % No esta en params_prensa (es un ajuste a un barrido, no un parametro
        % fisico); la usan el diseno de la excitacion y el comando de null.
        K_POS = 0.4459;  B_POS = 0.1139;    % [mm/s/V], [mm/s]  rama u > 0
        K_NEG = 0.3779;  B_NEG = 0.1397;    % rama u < 0
        U_NULL = -Prensa.B_NEG/Prensa.K_NEG;    % [V] velocidad nula: -0.370
        ASIMETRIA = Prensa.K_NEG/Prensa.K_POS;  % 0.847

        % --- tope de caudal (de P.Qp) ---------------------------------------
        Q_BOMBA = Prensa.P.Qp;                       % [m3/s]
        V_MAX_EXT = Prensa.Q_BOMBA/Prensa.A_A*1e3;   % [mm/s] techo al extender
        V_MAX_RET = Prensa.Q_BOMBA/Prensa.A_B*1e3;   % [mm/s] techo al retraer

        % --- lo que limita la medida (de P) ---------------------------------
        SIGMA_POSICION = Prensa.P.sig_x;     % [mm]
        LATENCIA_LAZO = Prensa.P.latencia;   % [s]
        TS_CONTROL = Prensa.P.Ts_ctrl;       % [s]
        CALENTAMIENTO = 180;      % [s] espera OBLIGATORIA tras arrancar la bomba

        % --- consignas de ensayos normados [mm/min] -------------------------
        V_ENSAYO_LOSA = 1.5;
        V_ENSAYO_VIGA = 0.1;

        % --- cadena de medida AI (NI 9222, DIFF +-10 V), escalas de P --------
        CANALES = struct( ...
            'id',     {'ai0','ai1','ai2','ai3'}, ...
            'nombre', {'posicion','fuerza','presion_A','presion_B'}, ...
            'unidad', {'mm','kN','bar','bar'}, ...
            'v_lo',   {0.0, 0.0, Prensa.P.pA_off, Prensa.P.pB_off}, ...
            'v_hi',   {10.0, 10.0, 10.0, 10.0}, ...
            'e_lo',   {0.0, 0.0, 0.0, 0.0}, ...
            'e_hi',   {10*Prensa.P.Kx_sens, 10*Prensa.P.KF_sens, ...
                       (10 - Prensa.P.pA_off)*Prensa.P.pA_esc, ...
                       (10 - Prensa.P.pB_off)*Prensa.P.pB_esc});
        CANAL_AO = 'ao0';

        % --- digitales (NI 9472 / 9421) -------------------------------------
        % DO line0 = arranque UPH · line5 = PERMISIVO, SIEMPRE alto (se escribe el
        % PUERTO ENTERO para sostenerlo) · DI line7 = motor encendido.
        LINEA_HPU = 0;  LINEA_PERMISIVO = 5;
        DO_SEGURO = Prensa.patronDO(false);   % permisivo alto, UPH parada
        DO_MARCHA = Prensa.patronDO(true);    % permisivo alto, UPH en marcha
        DI_REPOSO = [1 1 0 0 0 0 0 0];        % con la UPH apagada

        % --- limites de seguridad por software ------------------------------
        X_MIN_SEG = 20;  X_MAX_SEG = 130;     % [mm] ventana de TRABAJO
        X_MIN_JOG = 4;   X_MAX_JOG = 146;     % [mm] limites MECANICOS (recolocar)
        F_MAX_SEG = 50;                       % [kN]
        PA_MAX_SEG = 110;                     % [bar] sobre la limitadora (100)
        PB_MAX_SEG = 130;                     % [bar] bajo la saturacion del canal B
        U_MAX_SEG = 10;                       % [V]

        % Punto cero de presion (UPH parada, manometros a 0)
        V0_PRESION = [Prensa.P.pA_off Prensa.P.pB_off];
    end

    properties (SetAccess = private)
        Mods            % struct: ai, ao, di, do  (DeviceID por modelo de modulo)
    end

    properties (Access = private)
        AO              % sesion on-demand del AO (persistente)
        DO              % sesion del puerto DO completo (persistente)
        Cap             % estado de la captura en curso (callbacks)
    end

    %% ====================================================================
    %  2. FORMULAS (estaticas, sin hardware)
    %% ====================================================================
    methods (Static)
        function P = cargaParams()
        %CARGAPARAMS  params_prensa() de simulink/ (anade la carpeta al path).
            if isempty(which('params_prensa'))
                addpath(fullfile(fileparts(fileparts(mfilename('fullpath'))), 'simulink'));
            end
            P = params_prensa();
        end

        function p = patronDO(hpu)
            p = zeros(1,8);  p(Prensa.LINEA_PERMISIVO+1) = 1;  p(Prensa.LINEA_HPU+1) = hpu;
        end

        function v = velMedida(u)
        %VELMEDIDA  Velocidad [mm/s] que da el comando u [V], segun lo MEDIDO.
            u = double(u);
            v = (Prensa.K_POS*u + Prensa.B_POS).*(u >= 0) + ...
                (Prensa.K_NEG*u + Prensa.B_NEG).*(u < 0);
        end

        function u = uParaVel(v, umax)
        %UPARAVEL  Comando [V] que da la velocidad v [mm/s]. Inversa de velMedida.
            if nargin < 2, umax = 10; end
            if v >= Prensa.B_POS, u = (v - Prensa.B_POS)/Prensa.K_POS;
            else, u = (v - Prensa.B_NEG)/Prensa.K_NEG; end
            u = max(-umax, min(umax, u));
        end

        function F = fuerzaHidraulica(PA_bar, PB_bar)
        %FUERZAHIDRAULICA  F = P_A*A_A - P_B*A_B [kN] (cilindro ASIMETRICO).
        %   Fuerza INTERNA del cilindro; la celda mide la reaccion externa.
            F = (PA_bar*1e5*Prensa.A_A - PB_bar*1e5*Prensa.A_B)/1e3;
        end

        function r = ajustaDiametro(T)
        %AJUSTADIAMETRO  A_A y D a partir de escalones de fuerza contra un tope rigido.
        %   T: tabla con F_kN, VA_V, VB_V (medias de cada escalon, voltios).
        %   Modelo estatico: F = A_A*(P_A - P_B) + A_r*P_B + c, con A_r = A_A - A_B el
        %   area del vastago (d = 80 mm medido), asi solo queda A_A y el offset c.
        %   Se calcula con DOS juegos de escala de presion (hoja de datos y manometros)
        %   porque la escala del sensor es la mayor incertidumbre; tambien con tres
        %   parametros libres (A_A, A_B, c) y su numero de condicion: si P_A y P_B van
        %   muy correlacionadas, el ajuste libre no separa A_A de A_B.
            Ar = Prensa.A_A - Prensa.A_B;                           % [m2]
            esc = struct('nombre', {'hoja de datos', 'manometros 30/50'}, ...
                         'sA', {12.5, 11.07}, 'sB', {50, 54.52});    % [bar/V]
            V0A = Prensa.P.pA_off;  V0B = Prensa.P.pB_off;
            r = struct('nombre', {}, 'A_A_cm2', {}, 'D_mm', {}, 'c_kN', {}, 'rms_kN', {}, ...
                       'A_A_libre_cm2', {}, 'A_B_libre_cm2', {}, 'cond', {});
            fprintf('\n  AJUSTE DE A_A (vastago d = 80 mm: A_r = %.2f cm2) con %d escalones\n', Ar*1e4, height(T));
            fprintf('  %-18s %9s %8s %9s %8s | %9s %9s %6s\n', 'escala de presion', 'A_A cm2', 'D mm', 'c kN', 'rms kN', 'A_A libre', 'A_B libre', 'cond');
            for k = 1:numel(esc)
                PA = (T.VA_V - V0A)*esc(k).sA;  PB = (T.VB_V - V0B)*esc(k).sB;
                y = T.F_kN - 100*Ar*PB;
                X = [100*(PA - PB), ones(height(T), 1)];
                b = X\y;  AA = b(1);
                X3 = [PA, -PB, ones(height(T), 1)];
                b3 = X3\T.F_kN;
                r(k) = struct('nombre', esc(k).nombre, 'A_A_cm2', AA*1e4, ...
                    'D_mm', sqrt(4*AA/pi)*1e3, 'c_kN', b(2), 'rms_kN', sqrt(mean((y - X*b).^2)), ...
                    'A_A_libre_cm2', b3(1)*100, 'A_B_libre_cm2', b3(2)*100, 'cond', cond(X3(:, 1:2)));
                fprintf('  %-18s %9.1f %8.1f %9.2f %8.2f | %9.1f %9.1f %6.1f\n', r(k).nombre, ...
                    r(k).A_A_cm2, r(k).D_mm, r(k).c_kN, r(k).rms_kN, r(k).A_A_libre_cm2, r(k).A_B_libre_cm2, r(k).cond);
            end
            fprintf('  referencia: D = 125 mm -> A_A 122.7 cm2 ; D = 160 mm -> A_A 201.1 cm2\n');
        end

        function e = escala(v, c)
        %ESCALA  Voltios -> unidades de ingenieria (recta de dos puntos).
            e = c.e_lo + (v - c.v_lo)*(c.e_hi - c.e_lo)/(c.v_hi - c.v_lo);
        end

        function fallo = compruebaLimites(e, xlo, xhi)
        %COMPRUEBALIMITES  '' si el estado e esta dentro de todo; si no, el motivo.
            if nargin < 2, xlo = Prensa.X_MIN_SEG; xhi = Prensa.X_MAX_SEG; end
            fallo = '';
            if e.posicion < xlo || e.posicion > xhi
                fallo = sprintf('posicion %.1f mm fuera de [%g, %g]', e.posicion, xlo, xhi);
            elseif abs(e.fuerza) > Prensa.F_MAX_SEG
                fallo = sprintf('fuerza %.1f kN > %g', e.fuerza, Prensa.F_MAX_SEG);
            elseif e.presion_A > Prensa.PA_MAX_SEG
                fallo = sprintf('presion A %.1f bar > %g', e.presion_A, Prensa.PA_MAX_SEG);
            elseif e.presion_B > Prensa.PB_MAX_SEG
                fallo = sprintf('presion B %.1f bar > %g', e.presion_B, Prensa.PB_MAX_SEG);
            end
        end

        function ruta = resultados()
        %RESULTADOS  Carpeta results/ de la raiz del proyecto.
            ruta = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
            if ~isfolder(ruta), mkdir(ruta); end
        end
    end

    %% ====================================================================
    %  3. CONEXION Y LECTURA — SOLO LECTURA
    %% ====================================================================
    methods
        function obj = Prensa()
        %PRENSA  Localiza los modulos por NOMBRE de NI MAX (DeviceID): PrensaLEDI_AI/
        %   AO/DI/DO. Si se renombran, hay que cambiar los cuatro 'PrensaLEDI_*' de abajo.
        %   Solo lectura.
            dl = daqlist("ni");
            obj.Mods = struct('ai', '', 'ao', '', 'di', '', 'do', '');
            for k = 1:height(dl)
                m = char(dl.Model(k));  id = char(dl.DeviceID(k));
                if contains(id, 'PrensaLEDI_AI'), obj.Mods.ai = id;
                elseif contains(id, 'PrensaLEDI_AO'), obj.Mods.ao = id;
                elseif contains(id, 'PrensaLEDI_DI'), obj.Mods.di = id;
                elseif contains(id, 'PrensaLEDI_DO'), obj.Mods.do = id; end
            end
            falta = fieldnames(obj.Mods);  falta = falta(structfun(@isempty, obj.Mods));
            if any(ismember(falta, {'ai','ao'}))
                error('Prensa:modulos', 'Faltan modulos imprescindibles: %s. Revisar NI MAX.', ...
                    strjoin(falta, ', '));
            end
        end

        function delete(obj)
        %DELETE  Al destruir el objeto: AO a 0 y UPH parada (patron seguro).
            try, obj.liberaAO(); catch, end
            try, obj.liberaDO(); catch, end
        end

        function e = diag(obj)
        %DIAG  Enumera y lee 1000 muestras de los 4 AI. NO escribe nada.
            fprintf('%s\nDIAGNOSTICO DEL cDAQ — SOLO LECTURA\n%s\n', repmat('=',1,74), repmat('=',1,74));
            for f = fieldnames(obj.Mods).'
                id = obj.Mods.(f{1});
                if isempty(id), id = '-- NO ENCONTRADO --'; end
                fprintf('  %-4s : %s\n', f{1}, id);
            end
            try
                V = obj.leeBloque(1000, 1000);
            catch ME
                fprintf('  !! fallo la lectura: %s\n     Causa tipica: LabVIEW o NI MAX tienen el modulo RESERVADO.\n', ME.message);
                e = [];  return
            end
            fprintf('\n  %-6s %-10s %9s %9s %12s %5s\n', 'canal', 'senial', 'V medio', 'V ruido', 'ing. medio', 'unid');
            for i = 1:4
                c = Prensa.CANALES(i);
                ing = Prensa.escala(V(:,i), c);
                fprintf('  %-6s %-10s %9.4f %9.5f %12.3f %5s\n', c.id, c.nombre, mean(V(:,i)), std(V(:,i)), mean(ing), c.unidad);
            end
            e = obj.estado(V);
            F_hid = Prensa.fuerzaHidraulica(e.presion_A, e.presion_B);
            fprintf('\n  Fuerza por dos caminos independientes:\n');
            fprintf('    celda de carga     : %+7.3f kN\n', e.fuerza);
            fprintf('    P_A*A_A - P_B*A_B  : %+7.3f kN\n', F_hid);
            fprintf('    peso del movil     : %+7.3f kN (siempre hacia +)\n', Prensa.PESO/1e3);
            fprintf('  (sin probeta la celda marca ~0 y las presiones dan la fuerza interna)\n');
        end

        function e = lee(obj)
        %LEE  Estado instantaneo (20 ms a 1 kHz) en unidades de ingenieria. SOLO LECTURA.
            e = obj.estado(obj.leeBloque(20, 1000));
        end

        function sensores(obj, secs, fs)
        %SENSORES  Lectura en vivo. SOLO LECTURA.
            if nargin < 2, secs = 10; end
            if nargin < 3, fs = 100; end
            fprintf('\nLectura en vivo %.0f s a %.0f Hz (SOLO LECTURA)\n', secs, fs);
            tai = obj.abreAI(fs);  start(tai, "Continuous");
            c = onCleanup(@() stop(tai));
            t0 = tic;
            while toc(t0) < secs
                e = obj.leeEstado(tai, round(fs/5));
                fprintf('\r  t=%5.1fs  x=%8.3f mm  F=%8.3f kN  P_A=%7.2f  P_B=%7.2f bar', ...
                    toc(t0), e.posicion, e.fuerza, e.presion_A, e.presion_B);
            end
            fprintf('\n');
        end

        function bits = di(obj)
        %DI  Lee las 8 entradas digitales del NI 9421. SOLO LECTURA.
            d = daq("ni");  addinput(d, obj.Mods.di, "port0/line0:7", "Digital");
            bits = double(read(d, "OutputFormat", "Matrix"));
            fprintf('  Entradas digitales (NI 9421):\n');
            for i = 1:8
                marca = '';  if bits(i) ~= Prensa.DI_REPOSO(i), marca = '   <-- distinto del reposo'; end
                fprintf('    line%d = %d%s\n', i-1, bits(i), marca);
            end
        end
    end

    %% ====================================================================
    %  4. ESCRITURA BASICA: AO, DO, HPU  (todo exige 'Armar')
    %% ====================================================================
    methods
        function escribe(obj, u, o)
        %ESCRIBE  Fija el comando de la servovalvula [V] y lo MANTIENE.
        %   p.escribe(-0.37, 'Armar', true)
        %   SIN supervision: el vastago se mueve a la velocidad de la ley medida.
        %   Para vigilar limites usar lazoAbierto o las maniobras.
            arguments, obj, u (1,1) double, o.Armar (1,1) logical = false, end
            if abs(u) > Prensa.U_MAX_SEG
                error('Prensa:rango', '|u| = %.2f V supera %g V.', abs(u), Prensa.U_MAX_SEG);
            end
            if ~o.Armar
                fprintf('  [seco] escribiria %+.3f V en el AO. Anadir ''Armar'',true.\n', u); return
            end
            write(obj.sesionAO(), u);
            fprintf('[ok] AO a %+.3f V\n', u);
        end

        function cero(obj, o)
        %CERO  AO a 0 V (estado seguro).
            arguments, obj, o.Armar (1,1) logical = false, end
            obj.escribe(0, 'Armar', o.Armar);
        end

        function hpu(obj, encender, o)
        %HPU  Arranca o para la UPH y registra la deriva y las DI.
        %   A DIFERENCIA de daq.py, el puerto DO queda TOMADO al volver: la UPH
        %   sigue en marcha mientras exista el objeto (o hasta hpu(false)). Es lo
        %   que permite trabajar en lazo abierto de forma interactiva. Al destruir
        %   el objeto se escribe el patron seguro (UPH parada).
        %
        %   El AO se pone a 0 V ANTES de energizar: el NI 9263 conserva la ultima
        %   tension y presurizar da energia a lo que ya este pedido (§5.3d).
            arguments, obj, encender (1,1) logical, o.Armar (1,1) logical = false, o.Espera (1,1) double = 3, end
            fprintf('\n%s la UPH\n', ternario(encender, 'ARRANCANDO', 'PARANDO'));
            if ~o.Armar
                fprintf('  [seco] escribiria el puerto DO completo (%s/port0/line0:7). Anadir ''Armar'',true.\n', obj.Mods.do);
                return
            end
            obj.escribe(0, 'Armar', true);
            fprintf('  DI antes:\n');  antes = obj.di();
            obj.tomaDO();
            tv = obj.abreAI(200, {'ai0'});  start(tv, "Continuous");
            ts = [];  xs = [];  t0 = tic;
            patron = ternario(encender, Prensa.DO_MARCHA, Prensa.DO_SEGURO);
            write(obj.DO, patron);
            fprintf('  [DO] %s -> UPH %s\n', sprintf('%d', patron), ternario(encender, 'EN MARCHA', 'PARADA'));
            while toc(t0) < o.Espera
                b = read(tv, 100, "OutputFormat", "Matrix");
                ts(end+1) = toc(t0);  xs(end+1) = mean(Prensa.escala(b, Prensa.CANALES(1))); %#ok<AGROW>
            end
            stop(tv);  clear tv
            if numel(ts) >= 3
                pf = polyfit(ts, xs, 1);
                fprintf('  posicion durante la espera: %.2f -> %.2f mm en %.1f s\n', xs(1), xs(end), ts(end));
                fprintf('  DERIVA a comando 0 V: %+.4f mm/s (%+.3f mm/min)\n', pf(1), pf(1)*60);
            end
            fprintf('  DI despues:\n');  despues = obj.di();
            cambios = find(antes ~= despues) - 1;
            if isempty(cambios), fprintf('  -> NINGUNA DI ha cambiado: la UPH no arranco o el motor no esta cableado.\n');
            else, fprintf('  -> DI que han cambiado: %s (candidata a ''motor encendido'')\n', mat2str(cambios)); end
            if encender
                fprintf('\n  UPH EN MARCHA. Esperar %d s antes de medir (transitorio §5.9c).\n', Prensa.CALENTAMIENTO);
                fprintf('  Parar con p.hpu(false,''Armar'',true).\n');
            else
                obj.liberaDO();
            end
        end

        function conHpu(obj, fn, o)
        %CONHPU  Arranca la UPH, espera el calentamiento, ejecuta fn() y la para
        %   SIEMPRE, aunque fn falle. Equivale a --con-hpu: garantiza que el
        %   permisivo siga afirmado durante toda la maniobra.
        %   p.conHpu(@() p.caracteriza('Armar',true), 'Armar', true)
            arguments, obj, fn (1,1) function_handle, o.Armar (1,1) logical = false, o.Espera (1,1) double = Prensa.CALENTAMIENTO, end
            if ~o.Armar
                fprintf('  [seco] arrancaria la UPH, esperaria %.0f s, ejecutaria la maniobra y pararia. Anadir ''Armar'',true.\n', o.Espera);
                return
            end
            obj.escribe(0, 'Armar', true);
            obj.tomaDO();
            c = onCleanup(@() obj.paraSeguro());
            write(obj.DO, Prensa.DO_MARCHA);
            fprintf('  [DO] UPH EN MARCHA. Esperando %.0f s al transitorio de arranque (§5.9c)...\n', o.Espera);
            pause(o.Espera);
            fn();
            clear c      % dispara paraSeguro
        end
    end

    %% ====================================================================
    %  5. LATENCIA DEL LAZO DE SOFTWARE
    %% ====================================================================
    methods
        function lat = latencia(obj, o)
        %LATENCIA  Mide leer AI + escribir AO punto a punto. El AO se mantiene a
        %   0 V (la planta no se mueve). Decide el Ts de control alcanzable.
        %   Referencia con nidaqmx: 5.46 ms de mediana. Si sale ~200 ms, mirar el
        %   start/stop implicito por operacion (§6.1).
            arguments, obj, o.N (1,1) double = 2000, o.Fobj (1,1) double = 100, o.Armar (1,1) logical = false, end
            fprintf('\nMEDIDA DEL LAZO DE SOFTWARE (%d iteraciones, objetivo %.0f Hz)\n', o.N, o.Fobj);
            if ~o.Armar
                fprintf('  [seco] escribe 0 V en el AO. Anadir ''Armar'',true.\n'); lat = []; return
            end
            Ts = 1/o.Fobj;  tao = obj.sesionAO();
            tai = obj.abreAI([], {'ai0'});
            write(tao, 0);
            trd = zeros(o.N,1);  twr = trd;
            T0 = tic;  objetivo = 0;
            for k = 1:o.N
                objetivo = objetivo + Ts;
                a = toc(T0);  read(tai, "OutputFormat", "Matrix");
                b = toc(T0);  write(tao, 0);
                c = toc(T0);
                trd(k) = b - a;  twr(k) = c - b;
                while toc(T0) < objetivo, end        % espera activa: pause() redondea a ms
            end
            write(tao, 0);
            tot = (trd + twr)*1e3;
            fprintf('\n  %-14s %9s %9s %9s %9s\n', '', 'min', 'mediana', 'p95', 'max');
            for it = {'lectura AI', trd*1e3; 'escritura AO', twr*1e3; 'TOTAL', tot}.'
                v = it{2};
                fprintf('  %-14s %9.3f %9.3f %9.3f %9.3f   ms\n', it{1}, min(v), median(v), prctile(v,95), max(v));
            end
            Tsmin = prctile(tot, 99)*2;
            fprintf('  desviacion del total: %.3f ms\n', std(tot));
            fprintf('\n  -> Ts de control alcanzable: >= %.1f ms (%.0f Hz). Es tambien el retardo puro del modelo.\n', Tsmin, 1000/Tsmin);
            lat = struct('lectura_ms', trd*1e3, 'escritura_ms', twr*1e3, 'total_ms', tot);
        end

        function lat = latenciaAI(obj, n)
        %LATENCIAAI  Como latencia pero SOLO LECTURA (no escribe en el AO): mide la
        %   mitad AI del lazo para comprobar que la API no hace start/stop por read.
            if nargin < 2, n = 500; end
            tai = obj.abreAI([], {'ai0'});  t = zeros(n,1);
            for k = 1:n, a = tic; read(tai, "OutputFormat", "Matrix"); t(k) = toc(a)*1e3; end
            fprintf('  lectura AI: min %.2f  mediana %.2f  p95 %.2f  max %.2f ms\n', ...
                min(t), median(t), prctile(t,95), max(t));
            lat = t;
        end
    end

    %% ====================================================================
    %  6. CALIBRACION DE PRESION POR DOS PUNTOS
    %% ====================================================================
    methods
        function calibraPresion(obj, o)
        %CALIBRAPRESION  Segundo punto de la recta de los transductores.
        %   Mantiene la UPH en marcha con el AO en U_NULL (planta QUIETA) y registra
        %   ai2/ai3 mientras el operador lee los manometros.
        %   p.conHpu(@() p.calibraPresion('ManA',30,'ManB',50,'Armar',true), 'Armar',true)
        %   Sin 'ManA'/'ManB' imprime lo necesario para resolver la recta despues.
            arguments, obj, o.ManA = [], o.ManB = [], o.Secs (1,1) double = 90, o.Previa (1,1) double = 15, o.Armar (1,1) logical = false, end
            fprintf('%s\nCALIBRACION DE PRESION POR DOS PUNTOS\n%s\n', repmat('=',1,74), repmat('=',1,74));
            fprintf('  punto 0 (ya medido): A=%.4f V, B=%.4f V con manometros a 0 bar\n', Prensa.V0_PRESION);
            if ~o.Armar, fprintf('  ENSAYO EN SECO (falta ''Armar'').\n'); return, end
            if isempty(obj.DO), error('Prensa:hpu', 'La UPH debe estar en marcha: usar conHpu.'); end

            tao = obj.sesionAO();  write(tao, Prensa.U_NULL);
            fprintf('\n  AO a %+.3f V (velocidad nula). Se vigila la posicion.\n', Prensa.U_NULL);
            tai = obj.abreAI(200, {'ai2','ai3','ai0'});  start(tai, "Continuous");
            c = onCleanup(@() obj.finManiobra(tao, tai));
            t0 = tic;
            while toc(t0) < o.Previa
                d = read(tai, 100, "OutputFormat", "Matrix");
                x = mean(Prensa.escala(d(:,3), Prensa.CANALES(1)));
                obj.exigeVentana(tao, x);
                fprintf('\r    empieza el registro en %4.0f s   x=%6.2f mm', o.Previa - toc(t0), x);
            end
            fprintf('\n\n  >>> LEE LOS DOS MANOMETROS AHORA. Tienes %.0f segundos. <<<\n\n', o.Secs);
            vA = [];  vB = [];  xs = [];  t0 = tic;  ult = -1;
            while toc(t0) < o.Secs
                d = read(tai, 100, "OutputFormat", "Matrix");
                vA(end+1) = mean(d(:,1));  vB(end+1) = mean(d(:,2)); %#ok<AGROW>
                xs(end+1) = mean(Prensa.escala(d(:,3), Prensa.CANALES(1))); %#ok<AGROW>
                obj.exigeVentana(tao, xs(end));
                el = toc(t0);
                if el - ult >= 5
                    ult = el;
                    fprintf('    quedan %4.0f s   V_A=%7.4f V   V_B=%7.4f V   x=%6.2f mm\n', ...
                        o.Secs - el, vA(end), vB(end), xs(end));
                end
            end
            stop(tai);  write(tao, 0);

            fprintf('\n  POSICION durante la ventana: %.2f -> %.2f mm (%+.3f mm/min)\n', ...
                xs(1), xs(end), (xs(end) - xs(1))/o.Secs*60);
            V1A = mean(vA);  V1B = mean(vB);  sA = std(vA);  sB = std(vB);
            fprintf('\n  MEDIA DE LA VENTANA (%d bloques):\n    V_A = %.4f ± %.4f V\n    V_B = %.4f ± %.4f V\n', ...
                numel(vA), V1A, sA, V1B, sB);
            if max(sA, sB) > 0.02, fprintf('    !! la presion NO estaba estable: la recta saldra sucia.\n');
            else, fprintf('    [ok] estable durante toda la ventana\n'); end
            if isempty(o.ManA) || isempty(o.ManB)
                fprintf('\n  Faltan las lecturas de manometro. P = (V - V0)*P_man/(V1 - V0)\n');
                fprintf('    canal A: V0=%.4f V1=%.4f dV=%+.4f V\n', Prensa.V0_PRESION(1), V1A, V1A - Prensa.V0_PRESION(1));
                fprintf('    canal B: V0=%.4f V1=%.4f dV=%+.4f V\n', Prensa.V0_PRESION(2), V1B, V1B - Prensa.V0_PRESION(2));
                return
            end
            fprintf('\n  RECTA RESULTANTE (dos puntos):\n');
            datos = {'presion_A', Prensa.V0_PRESION(1), V1A, o.ManA; 'presion_B', Prensa.V0_PRESION(2), V1B, o.ManB};
            for k = 1:2
                [nom, V0, V1, Pman] = datos{k,:};
                if abs(V1 - V0) < 1e-3, fprintf('    %s: el canal no ha cambiado. Sin resolver.\n', nom); continue, end
                bpv = Pman/(V1 - V0);
                fprintf('    %s: %8.3f bar/V (fondo de escala a 10 V: %.1f bar)\n', nom, bpv, bpv*(10 - V0));
                fprintf('        para CANALES: v_lo=%.4f, v_hi=10, e_lo=0, e_hi=%.1f\n', V0, bpv*(10 - V0));
            end
        end
    end

    %% ====================================================================
    %  7. MANIOBRAS QUE MUEVEN EL ACTUADOR
    %% ====================================================================
    methods
        function res = caracteriza(obj, o)
        %CARACTERIZA  Curva comando -> velocidad en AMBOS sentidos (la medida de Fase 0).
        %   De aqui salen el SENTIDO, la ganancia por sentido, la asimetria (0.772
        %   del modelo, 0.847 medido) y la deriva de null (paso u = 0).
        %   Ajusta solo |u| <= 2.5 V: por encima satura el caudal y el acumulador
        %   falsea escalones cortos (§5.9).   Devuelve res = [u v].
            arguments, obj
                o.UList (1,:) double = [0 0.05 0.1 0.25 0.5 1 2 4 6 8 10]
                o.TPaso (1,1) double = 2.5, o.XObj (1,1) double = 75, o.Armar (1,1) logical = false
            end
            fprintf('%s\nCARACTERIZACION comando -> velocidad\n%s\n', repmat('=',1,74), repmat('=',1,74));
            fprintf('  amplitudes : %s V (y sus negativas)\n  t por paso : %.1f s, recolocacion a %.0f mm\n', ...
                mat2str(o.UList), o.TPaso, o.XObj);
            obj.imprimeLimites();
            if ~o.Armar, fprintf('\n  ENSAYO EN SECO (falta ''Armar''): no se escribe en el AO.\n'); res = []; return, end

            fs = 100;  res = zeros(0,2);
            tao = obj.sesionAO();  tai = obj.abreAI(fs);
            write(tao, 0);  start(tai, "Continuous");
            c = onCleanup(@() obj.finManiobra(tao, tai)); %#ok<NASGU>
            try
                e = obj.leeEstado(tai, 10);
                fprintf('\n  posicion inicial: %.2f mm\n', e.posicion);
                fallo = Prensa.compruebaLimites(e);
                if ~isempty(fallo), error('Prensa:aborta', 'ABORTA antes de empezar: %s', fallo); end
                signo = obj.descubreSentido(tao, tai);
                if abs(e.posicion - o.XObj) > 25         % solo si hace falta DE VERDAD
                    fprintf('\n  Recolocando a %.1f mm antes del barrido...\n', o.XObj);
                    if ~obj.recoloca(tao, tai, o.XObj, signo, 'UMax', 2, 'TMax', 180)
                        error('Prensa:aborta', 'ABORTA: no se pudo recolocar el vastago.');
                    end
                else
                    fprintf('  a %.1f mm del centro (banda ±25): se barre desde aqui\n', abs(e.posicion - o.XObj));
                end
                fprintf('\n');
                for u0 = o.UList
                    for sg = ternario(u0 == 0, 1, [1 -1])
                        u = sg*u0;
                        e = obj.leeEstado(tai, 10);
                        if abs(e.posicion - o.XObj) > 30      % cada tramo consume recorrido
                            obj.recoloca(tao, tai, o.XObj, signo, 'UMax', 2, 'TMax', 90, 'Verboso', false);
                            e = obj.leeEstado(tai, 10);
                        end
                        % Un limite rozado ANTES de un paso suele ser transitorio:
                        % se reintenta antes de rendirse.
                        for intento = 1:4
                            fallo = Prensa.compruebaLimites(e);
                            if isempty(fallo), break, end
                            fprintf('    (limite rozado antes de u=%+.2f V: %s — esperando, intento %d/4)\n', u, fallo, intento);
                            write(tao, 0);  pause(2);  flush(tai);  e = obj.leeEstado(tai, 10);
                        end
                        if ~isempty(fallo)
                            fprintf('  !! ABORTA antes de u=%+.2f V: %s\n     estado: %s\n', u, fallo, formatea(e));
                            error('Prensa:corte', 'corte');
                        end
                        write(tao, u);  flush(tai);
                        ts = [];  xs = [];  t0 = tic;
                        while toc(t0) < o.TPaso
                            e = obj.leeEstado(tai, 5);
                            ts(end+1) = toc(t0);  xs(end+1) = e.posicion; %#ok<AGROW>
                            fallo = Prensa.compruebaLimites(e);
                            if ~isempty(fallo)
                                write(tao, 0);  fprintf('  !! corte en u=%+.2f V: %s\n', u, fallo);  break
                            end
                        end
                        write(tao, 0);  pause(0.5);
                        m = ts >= 0.3;                         % descarta el transitorio del carrete
                        if nnz(m) >= 3, ts = ts(m); xs = xs(m); end
                        if numel(ts) >= 3
                            pf = polyfit(ts, xs, 1);
                            res(end+1,:) = [u pf(1)]; %#ok<AGROW>
                            fprintf('    u=%+6.2f V -> v=%+9.4f mm/s   (x %6.1f -> %6.1f mm, P_A=%6.1f P_B=%6.1f bar)\n', ...
                                u, pf(1), xs(1), xs(end), e.presion_A, e.presion_B);
                        end
                    end
                end
            catch ME
                if ~strcmp(ME.identifier, 'Prensa:corte'), rethrow(ME); end
                fprintf('  interrumpido\n');
            end
            if isempty(res), return, end
            obj.resumenCaracterizacion(res);
        end

        function satura(obj, o)
        %SATURA  Mide el TOPE DE CAUDAL de la bomba recorriendo la carrera a
        %   comando alto en los dos sentidos. El discriminador es la RELACION de
        %   mesetas retraer/extender: REL_AREAS (=A_A/A_B) si limita el CAUDAL, 0.847
        %   si limita el ORIFICIO. Escribe results/saturacion.csv.
            arguments, obj
                o.UTest (1,1) double = 10, o.Margen (1,1) double = 6, o.TMax (1,1) double = 120
                o.Fs (1,1) double = 200, o.SignoDado (1,1) double = 0
                o.Calienta (1,1) double = 180, o.Armar (1,1) logical = false
            end
            fprintf('%s\nPRUEBA DE SATURACION DE CAUDAL (|u| = %.1f V sostenido)\n%s\n', repmat('=',1,74), o.UTest, repmat('=',1,74));
            fprintf('  recorrido: %g..%g mm en los dos sentidos\n', Prensa.X_MIN_SEG + o.Margen, Prensa.X_MAX_SEG - o.Margen);
            fprintf('  relacion esperada si SATURA POR CAUDAL: %.3f ; por ORIFICIO: 0.847\n', Prensa.REL_AREAS);
            if ~o.Armar, fprintf('\n  ENSAYO EN SECO (falta ''Armar'').\n'); return, end

            xlo = Prensa.X_MIN_SEG + o.Margen;  xhi = Prensa.X_MAX_SEG - o.Margen;
            reg = struct();
            tao = obj.sesionAO();  tai = obj.abreAI(o.Fs);
            write(tao, 0);  start(tai, "Continuous");
            c = onCleanup(@() obj.finManiobra(tao, tai)); %#ok<NASGU>
            if o.SignoDado ~= 0
                signo = sign(o.SignoDado);
                fprintf('  sentido IMPUESTO: comando positivo %s la posicion\n', ternario(signo > 0, 'aumenta', 'disminuye'));
            else
                signo = obj.descubreSentido(tao, tai);
            end
            if o.Calienta > 0                                  % calentamiento + transitorio
                fprintf('\n  --- calentamiento: %.0f s sosteniendo %+.3f V ---\n', o.Calienta, Prensa.U_NULL);
                centro = 0.5*(xlo + xhi);  write(tao, Prensa.U_NULL);  flush(tai);
                tw = [];  xw = [];  pw = [];  t0 = tic;  ult = -30;
                while toc(t0) < o.Calienta
                    e = obj.leeEstado(tai, round(o.Fs/5));
                    tw(end+1) = toc(t0);  xw(end+1) = e.posicion;  pw(end+1) = e.presion_A; %#ok<AGROW>
                    if abs(e.posicion - centro) > 35
                        write(tao, 0);
                        obj.recoloca(tao, tai, centro, signo, 'UMax', 2, 'TMax', 120, 'Verboso', false);
                        write(tao, Prensa.U_NULL);
                    end
                    if toc(t0) - ult >= 30
                        ult = toc(t0);
                        fprintf('    t=%5.0fs  x=%7.2f mm  P_A=%5.1f bar\n', ult, e.posicion, e.presion_A);
                    end
                end
                write(tao, 0);  pause(1);
                w = max(5, round(10/median(diff(tw))));
                if numel(tw) - w > 10
                    vw = arrayfun(@(i) polyfit(tw(i:i+w), xw(i:i+w), 1)*[1;0], 1:numel(tw)-w);
                    fprintf('    deriva sostenida en %+.3f V: %+.4f -> %+.4f mm/s\n', Prensa.U_NULL, vw(1), vw(end));
                    q = pw(end - floor(numel(pw)/4));
                    fprintf('    presion A: %.1f -> %.1f bar (%s)\n', pw(1), pw(end), ...
                        ternario(abs(pw(end) - q) < 1, 'ASENTADA', 'AUN CAMBIANDO'));
                end
            end
            for sent = {'extendiendo', xhi, xlo; 'retrayendo', xlo, xhi}.'
                [nombre, destino, arranque] = sent{:};
                fprintf('\n  --- %s: colocando en %.0f mm ---\n', nombre, arranque);
                if ~obj.recoloca(tao, tai, arranque, signo, 'UMax', 2, 'TMax', 200, 'Verboso', false)
                    fprintf('  !! no se pudo colocar; se salta este sentido\n'); continue
                end
                u = o.UTest*signo*ternario(destino > arranque, 1, -1);
                fprintf('  aplicando u = %+.2f V hasta %.0f mm...\n', u, destino);
                D = zeros(0,4);  write(tao, u);  flush(tai);  t0 = tic;
                while toc(t0) < o.TMax
                    e = obj.leeEstado(tai, round(o.Fs/10));
                    D(end+1,:) = [toc(t0) e.posicion e.presion_A e.presion_B]; %#ok<AGROW>
                    if (destino > arranque && e.posicion >= destino) || (destino < arranque && e.posicion <= destino), break, end
                    if ~isempty(Prensa.compruebaLimites(e, Prensa.X_MIN_JOG, Prensa.X_MAX_JOG)), break, end
                end
                write(tao, 0);  pause(1);
                reg.(nombre) = D;
                fprintf('  %.1f -> %.1f mm en %.1f s\n', D(1,2), D(end,2), D(end,1));
            end

            fprintf('\n%s\nRESULTADO\n%s\n', repmat('=',1,74), repmat('=',1,74));
            mes = struct();
            for nombre = fieldnames(reg).'
                D = reg.(nombre{1});  ts = D(:,1);  xs = D(:,2);
                if numel(ts) < 20, fprintf('  %s: muy pocos datos\n', nombre{1}); continue, end
                w = max(5, round(3/median(diff(ts))));
                if numel(ts) - w < 10, fprintf('  %s: tramo demasiado corto\n', nombre{1}); continue, end
                vt = arrayfun(@(i) polyfit(ts(i:i+w), xs(i:i+w), 1)*[1;0], 1:numel(ts)-w);
                vini = median(vt(1:max(3, floor(numel(vt)/10))));
                vfin = median(vt(end-max(3, floor(numel(vt)/5))+1:end));
                mes.(nombre{1}) = abs(vfin);
                A = ternario(strcmp(nombre{1}, 'extendiendo'), Prensa.A_A, Prensa.A_B);
                fprintf('  %-12s: v inicial %6.3f -> meseta %6.3f mm/s (caida %+5.1f %%)\n', nombre{1}, ...
                    abs(vini), abs(vfin), (1 - abs(vfin/vini))*100);
                fprintf('                caudal implicito = %.2f L/min   P_A %5.1f  P_B %5.1f bar\n', ...
                    abs(vfin)*1e-3*A*60000, mean(D(:,3)), mean(D(:,4)));
            end
            if isfield(mes, 'extendiendo') && isfield(mes, 'retrayendo')
                rel = mes.retrayendo/mes.extendiendo;
                fprintf('\n  RELACION retraer/extender = %.3f  (caudal %.3f, orificio 0.847)\n', rel, Prensa.REL_AREAS);
                fprintf('  -> compatible con limitacion por %s\n', ...
                    ternario(abs(rel - Prensa.REL_AREAS) < abs(rel - 0.847), 'CAUDAL (la bomba)', 'ORIFICIO (la servovalvula)'));
            end
            T = table();
            for nombre = fieldnames(reg).'
                D = reg.(nombre{1});
                T = [T; table(repmat(string(nombre{1}), size(D,1), 1), D(:,1), D(:,2), D(:,3), D(:,4), ...
                    'VariableNames', {'sentido','t_s','posicion_mm','presion_A_bar','presion_B_bar'})]; %#ok<AGROW>
            end
            ruta = fullfile(Prensa.resultados(), 'saturacion.csv');
            writetable(T, ruta);  fprintf('\n[ok] %s\n', ruta);
        end

        function jog(obj, destino, o)
        %JOG  Lleva el vastago a `destino` [mm] con un lazo P lento y acotado
        %   (con compensacion del null). No es un controlador: es posicionamiento.
            arguments, obj, destino (1,1) double, o.UMax (1,1) double = 2, o.Tol (1,1) double = 1, o.TMax (1,1) double = 240, o.Armar (1,1) logical = false, end
            fprintf('%s\nRECOLOCACION a %.1f mm (|u| <= %g V, tol %g mm)\n%s\n', repmat('=',1,74), destino, o.UMax, o.Tol, repmat('=',1,74));
            if destino < Prensa.X_MIN_SEG || destino > Prensa.X_MAX_SEG
                error('Prensa:aborta', 'ABORTA: destino fuera de [%g, %g] mm', Prensa.X_MIN_SEG, Prensa.X_MAX_SEG);
            end
            if ~o.Armar, fprintf('  ENSAYO EN SECO (falta ''Armar'').\n'); return, end
            tao = obj.sesionAO();  tai = obj.abreAI(100);
            write(tao, 0);  start(tai, "Continuous");
            c = onCleanup(@() obj.finManiobra(tao, tai)); %#ok<NASGU>
            e0 = obj.leeEstado(tai, 10);
            fprintf('  posicion inicial: %.2f mm\n', e0.posicion);
            signo = obj.descubreSentido(tao, tai);
            obj.recoloca(tao, tai, destino, signo, 'UMax', o.UMax, 'Tol', o.Tol, 'TMax', o.TMax);
            fprintf('\n  AO a 0 V.\n');
        end

        function res = escalonesFuerza(obj, o)
        %ESCALONESFUERZA  Escalones de fuerza contra un tope rigido (viga de acero) para
        %   fijar A_A (y con ella el diametro) con la celda como referencia.
        %
        %   p.conHpu(@() p.escalonesFuerza('Armar',true), 'Armar', true)
        %
        %   Fases: sale (extiende desde x bajo hasta XAprox) -> aprox (lento hasta el
        %   contacto, F > 1.5 kN) -> carga (PI de fuerza con rampa; escalones de subida
        %   y de bajada) -> descarga (retrae hasta F < 0.5 kN) -> separa. Cualquier
        %   limite (F, P_A, P_B, ventana de posicion, F negativa, tiempo) lleva a la
        %   descarga controlada. Al terminar deja el AO en U_NULL: PARAR LA UPH enseguida
        %   (a 0 V el vastago se extiende ~8 mm/min y volveria a cargar la viga); conHpu
        %   lo hace. Presiones en bar con la escala de params_prensa (PROVISIONAL) y
        %   tambien en voltios, para reajustar la escala despues.
            arguments, obj
                o.Fobj (1,:) double = [5 10 20 30 40]   % [kN] escalones de subida
                o.Baja (1,1) logical = true             % repite bajando
                o.XAprox (1,1) double = 40              % [mm] hasta aqui se sube sin buscar contacto
                o.XLento (1,1) double = 48              % [mm] desde aqui se aproxima despacio
                o.XMaxContacto (1,1) double = 100       % [mm] sin contacto hasta aqui: aborta
                o.Rampa (1,1) double = 1                % [kN/s]
                o.Mantiene (1,1) double = 15            % [s] por escalon
                o.Kp (1,1) double = 0.04                % [V/kN]
                o.Ki (1,1) double = 0.03                % [V/(kN s)]
                o.DuMax (1,1) double = 0.25             % [V] excursion maxima sobre U_NULL
                o.PAmax (1,1) double = 60               % [bar]
                o.PBmax (1,1) double = 75               % [bar] (en reposo P_B ~46 bar)
                o.TMax (1,1) double = 900               % [s]
                o.Etiqueta (1,1) string = "escalones"
                o.Armar (1,1) logical = false
            end
            fprintf('%s\nESCALONES DE FUERZA contra tope rigido\n%s\n', repmat('=',1,74), repmat('=',1,74));
            if any(o.Fobj <= 0) || max(o.Fobj) > 0.8*Prensa.F_MAX_SEG
                error('Prensa:rango', 'Fobj debe estar en (0, %g] kN', 0.8*Prensa.F_MAX_SEG);
            end
            objetivos = o.Fobj;
            if o.Baja, objetivos = [o.Fobj, fliplr(o.Fobj(1:end-1))]; end
            Fabort = min(1.25*max(o.Fobj) + 3, Prensa.F_MAX_SEG);
            fprintf('  escalones [kN]: %s\n  rampa %.1f kN/s, %.0f s por escalon, PI Kp=%.3f V/kN Ki=%.3f V/(kN s), |du|<=%.2f V\n', ...
                mat2str(objetivos), o.Rampa, o.Mantiene, o.Kp, o.Ki, o.DuMax);
            fprintf('  aborta si |F| > %.1f kN, P_A > %g, P_B > %g bar, x fuera de [%g, %g] mm (desde XAprox)\n', ...
                Fabort, o.PAmax, o.PBmax, Prensa.X_MIN_SEG, Prensa.X_MAX_SEG);
            fprintf('  contacto esperado a ~53-63 mm; aproximacion lenta desde %g mm, sin contacto a %g mm: aborta\n', o.XLento, o.XMaxContacto);
            if ~o.Armar, fprintf('  ENSAYO EN SECO (falta ''Armar'').\n'); res = []; return, end
            if isempty(obj.DO), error('Prensa:hpu', 'La UPH debe estar en marcha: usar conHpu.'); end

            U0 = Prensa.U_NULL;
            tao = obj.sesionAO();  tai = obj.abreAI(200);
            write(tao, U0);  start(tai, "Continuous");
            c = onCleanup(@() obj.finManiobra(tao, tai)); %#ok<NASGU>
            e0 = obj.leeEstado(tai, 20);
            fprintf('  inicio: x=%.2f mm  F=%.2f kN  P_A=%.1f  P_B=%.1f bar\n', e0.posicion, e0.fuerza, e0.presion_A, e0.presion_B);
            if e0.posicion < 1 || e0.posicion > o.XAprox, error('Prensa:aborta', 'x inicial %.1f mm fuera de [1, %g]', e0.posicion, o.XAprox); end
            if abs(e0.fuerza) > 2, error('Prensa:aborta', 'F inicial %.2f kN: debe ser ~0 antes de empezar', e0.fuerza); end

            nombres = {'sale','aprox','carga','descarga','separa','fin'};
            cA = Prensa.CANALES(3);  cB = Prensa.CANALES(4);
            L = zeros(0, 10);                 % t fase x F Fsp PA PB u VA VB
            pasos = zeros(0, 9);              % Fobj F PA PB VA VB x sube t
            fase = 'sale';  abortado = '';  xIni = e0.posicion;  xc = NaN;
            nCont = 0;  integ = 0;  Fsp = 0;  k = 1;  tHold = NaN;  tFase = 0;  tPrev = 0;  ult = -2;
            xSep0 = e0.posicion;
            T0 = tic;
            while true
                e = obj.leeEstado(tai, 4);
                t = toc(T0);  dt = max(t - tPrev, 1e-3);  tPrev = t;
                x = e.posicion;  F = e.fuerza;  PA = e.presion_A;  PB = e.presion_B;

                % ---- vigilancia ------------------------------------------------
                if ~ismember(fase, {'descarga','separa','fin'}) && isempty(abortado)
                    if abs(F) > Fabort, abortado = sprintf('fuerza %.1f kN > %.1f', F, Fabort);
                    elseif PA > o.PAmax, abortado = sprintf('presion A %.1f bar > %g', PA, o.PAmax);
                    elseif PB > o.PBmax, abortado = sprintf('presion B %.1f bar > %g', PB, o.PBmax);
                    elseif x > Prensa.X_MAX_SEG, abortado = sprintf('posicion %.1f mm > %g', x, Prensa.X_MAX_SEG);
                    elseif ~strcmp(fase, 'sale') && x < Prensa.X_MIN_SEG, abortado = sprintf('posicion %.1f mm < %g', x, Prensa.X_MIN_SEG);
                    elseif F < -3, abortado = sprintf('fuerza %.1f kN negativa (signo de celda o tiron)', F);
                    elseif t > o.TMax, abortado = sprintf('tiempo > %g s', o.TMax);
                    end
                    if ~isempty(abortado)
                        fprintf('\n  !! ABORTA: %s -> descarga controlada\n', abortado);
                        fase = 'descarga';  tFase = t;
                    end
                end

                % ---- fases -----------------------------------------------------
                u = U0;
                switch fase
                    case 'sale'
                        u = Prensa.uParaVel(0.6, 3);
                        if t > 10 && (x - xIni) < 0.5
                            abortado = 'el vastago no sube: sentido o sensor'; fase = 'fin';
                        elseif x >= o.XAprox
                            fase = 'aprox';  tFase = t;
                            fprintf('\n  [aprox] x=%.1f mm: aproximacion al contacto\n', x);
                        end
                    case 'aprox'
                        if x < o.XLento, v = 0.6; else, v = 0.15; end
                        u = Prensa.uParaVel(v, 3);
                        if F > 1.5, nCont = nCont + 1; else, nCont = 0; end
                        if nCont >= 3
                            xc = x;  fase = 'carga';  Fsp = F;  integ = 0;  tFase = t;
                            fprintf('\n  [contacto] x=%.2f mm F=%.2f kN -> carga\n', x, F);
                        elseif x > o.XMaxContacto
                            abortado = sprintf('sin contacto hasta %g mm', o.XMaxContacto); fase = 'fin';
                        end
                    case 'carga'
                        objF = objetivos(k);
                        Fsp = Fsp + max(-o.Rampa*dt, min(o.Rampa*dt, objF - Fsp));
                        err = Fsp - F;
                        integ = max(-o.DuMax/o.Ki, min(o.DuMax/o.Ki, integ + err*dt));
                        u = U0 + max(-o.DuMax, min(o.DuMax, o.Kp*err + o.Ki*integ));
                        if isnan(tHold) && abs(objF - Fsp) < 1e-6, tHold = t; end
                        if ~isnan(tHold) && t - tHold >= o.Mantiene
                            w = L(:,2) == 3 & L(:,1) >= tHold + 0.4*o.Mantiene;
                            m = mean(L(w, [4 6 7 9 10 3]), 1);
                            pasos(end+1,:) = [objF, m, k <= numel(o.Fobj), t]; %#ok<AGROW>
                            fprintf('\n  [paso %d/%d] Fobj %.0f kN: F=%.2f  P_A=%.2f  P_B=%.2f bar  x=%.2f mm\n', ...
                                k, numel(objetivos), objF, m(1), m(2), m(3), m(6));
                            k = k + 1;  tHold = NaN;
                            if k > numel(objetivos), fase = 'descarga';  tFase = t; end
                        end
                    case 'descarga'
                        u = U0 - 0.35;
                        if F < 0.5 || t - tFase > 60, fase = 'separa';  tFase = t;  xSep0 = x; end
                    case 'separa'
                        % 15 mm de holgura: al parar la UPH el acumulador sigue cargado y con
                        % AO = 0 V (+0.37 V sobre el null) empuja el vastago hacia la viga
                        % (2026-11-04: 17 kN con solo 3 mm de separacion).
                        u = U0 - 2;
                        if x <= xSep0 - 15 || x <= Prensa.X_MIN_SEG + 1 || t - tFase > 40, fase = 'fin'; end
                end
                u = max(-3, min(3, u));
                write(tao, u);

                VA = cA.v_lo + (PA - cA.e_lo)*(cA.v_hi - cA.v_lo)/(cA.e_hi - cA.e_lo);
                VB = cB.v_lo + (PB - cB.e_lo)*(cB.v_hi - cB.v_lo)/(cB.e_hi - cB.e_lo);
                L(end+1,:) = [t, find(strcmp(fase, nombres), 1, 'first'), x, F, Fsp, PA, PB, u, VA, VB]; %#ok<AGROW>
                if strcmp(fase, 'fin'), break, end
                if t - ult >= 2
                    ult = t;
                    fprintf('\r  t=%5.0fs %-8s x=%6.2f mm F=%6.2f (sp %5.1f) kN  P_A=%5.1f P_B=%5.1f bar  u=%+.3f V ', ...
                        t, fase, x, F, Fsp, PA, PB, u);
                end
            end
            write(tao, U0);
            fprintf('\n\n  fin: AO en %+.3f V (PARAR LA UPH ahora).  %s\n', U0, ternario(isempty(abortado), 'completo', ['ABORTADO: ' abortado]));

            T = array2table(pasos, 'VariableNames', {'Fobj_kN','F_kN','PA_bar','PB_bar','VA_V','VB_V','x_mm','sube','t_s'});
            Lt = array2table(L, 'VariableNames', {'t_s','fase','x_mm','F_kN','Fsp_kN','PA_bar','PB_bar','u_V','VA_V','VB_V'});
            res = struct('pasos', T, 'registro', Lt, 'xContacto', xc, 'abortado', abortado);
            try
                writetable(T,  fullfile(Prensa.resultados(), o.Etiqueta + "_pasos.csv"));
                writetable(Lt, fullfile(Prensa.resultados(), o.Etiqueta + "_registro.csv"));
                fprintf('  guardado: results/%s_pasos.csv y _registro.csv\n', o.Etiqueta);
            catch ME
                fprintf('  (no se pudo guardar: %s)\n', ME.message);
            end
            if height(T) >= 3, res.ajuste = Prensa.ajustaDiametro(T); end
        end
    end

    %% ====================================================================
    %  8. LAZO ABIERTO: EXCITACION + REGISTRO, TEMPORIZADO POR HARDWARE
    %% ====================================================================
    methods
        function r = lazoAbierto(obj, u, o)
        %LAZOABIERTO  Emite una secuencia de comando y registra los 4 AI.
        %
        %   r = p.lazoAbierto(u, 'Ts', 1e-3, 'Etiqueta', 'train', 'Armar', true)
        %   r = p.lazoAbierto('results/excitacion_train.csv', 'Etiqueta','train', ...)
        %
        %   u: vector de voltios (una muestra por Ts) o ruta de un CSV t_s,u_V.
        %   AO y AI salen del MISMO objeto DataAcquisition (un reloj de chasis), asi
        %   que comando y medida quedan alineados por hardware y el Ts es EXACTO.
        %   El AO se rellena por trozos de BufferS segundos y se vigilan posicion,
        %   fuerza y presiones en cada trozo: si algo se sale, AO a 0 V y se
        %   guardan los datos hasta ese punto. Ctrl-C tambien deja el AO a 0.
        %
        %   La UPH debe estar en marcha (hpu / conHpu) y haber pasado el
        %   calentamiento. Sin probeta (el limite de fuerza lo exige).
        %   Devuelve: t, u, posicion [mm], fuerza [kN], presion_A/B [bar],
        %   abortado ('' si completo). Guarda results/captura_<Etiqueta>.csv.
            arguments, obj, u
                o.Ts (1,1) double = 1e-3, o.Etiqueta (1,1) string = "captura"
                o.BufferS (1,1) double = 0.3, o.Guardar (1,1) logical = true
                o.Armar (1,1) logical = false
            end
            if ischar(u) || isstring(u)
                tb = readtable(u);  o.Ts = tb.t_s(2) - tb.t_s(1);  u = tb.u_V;
            end
            u = u(:);  fs = 1/o.Ts;  nTot = numel(u);  nBuf = max(1, round(o.BufferS*fs));
            if max(abs(u)) > Prensa.U_MAX_SEG
                error('Prensa:rango', 'ABORTA: la secuencia pide %.2f V (limite %g V)', max(abs(u)), Prensa.U_MAX_SEG);
            end
            fprintf('%s\nLAZO ABIERTO ''%s''\n%s\n', repmat('=',1,74), o.Etiqueta, repmat('=',1,74));
            fprintf('  muestras : %d (%.1f s a %.0f Hz)\n  comando  : %+.3f .. %+.3f V\n', nTot, nTot/fs, fs, min(u), max(u));
            fprintf('  buffer   : %d muestras (%.0f ms de latencia de aborto)\n', nBuf, o.BufferS*1e3);
            obj.imprimeLimites();

            fprintf('\n  Comprobacion previa (solo lectura):\n');
            e = obj.estado(obj.leeBloque(500, 1000));
            fprintf('    %-10s = %9.3f\n', 'posicion', e.posicion);
            if e.posicion <= Prensa.X_MIN_SEG || e.posicion >= Prensa.X_MAX_SEG
                error('Prensa:aborta', 'ABORTA: posicion inicial %.1f mm fuera de [%g, %g]. Recolocar (jog).', ...
                    e.posicion, Prensa.X_MIN_SEG, Prensa.X_MAX_SEG);
            end
            if e.fuerza > Prensa.F_MAX_SEG
                error('Prensa:aborta', 'ABORTA: hay %.1f kN de carga. La identificacion se hace SIN probeta.', e.fuerza);
            end
            fprintf('    [ok] posicion inicial %.1f mm, sin carga\n', e.posicion);
            if ~o.Armar
                fprintf('\n  ENSAYO EN SECO: no se escribe nada. Anadir ''Armar'',true.\n'); r = []; return
            end

            obj.liberaAO();          % el AO on-demand y el sincronizado no pueden coexistir
            fprintf('\n  ARMADO. Emitiendo... (Ctrl-C aborta a 0 V)\n');
            d = daq("ni");
            for c = Prensa.CANALES
                ch = addinput(d, obj.Mods.ai, c.id, "Voltage");
                ch.TerminalConfig = "Differential";  ch.Range = [-10 10];
            end
            ao = addoutput(d, obj.Mods.ao, Prensa.CANAL_AO, "Voltage");  ao.Range = [-10 10];
            d.Rate = fs;
            d.ScansAvailableFcnCount = nBuf;
            d.ScansRequiredFcnCount = nBuf;
            obj.Cap = struct('u', u, 'nTot', nTot, 'nBuf', nBuf, 'iEsc', 0, 'nLeidas', 0, ...
                'reg', zeros(nTot, 4), 'abortado', '', 't0', tic);
            d.ScansAvailableFcn = @(src, ~) obj.alLeer(src);
            d.ScansRequiredFcn  = @(src, ~) obj.alEscribir(src);
            d.ErrorOccurredFcn  = @(src, evt) obj.alError(src, evt);

            seguro = onCleanup(@() obj.cierraCaptura(d));
            preload(d, obj.siguienteTrozo(2*nBuf));
            start(d, "Continuous");
            while d.Running, pause(0.05); end
            if isempty(obj.Cap.abortado) && obj.Cap.nLeidas < nTot
                obj.Cap.abortado = 'la adquisicion se detuvo antes de tiempo';
            end
            clear seguro                  % stop + AO a 0 V
            fprintf('\n');

            C = obj.Cap;  n = C.nLeidas;
            if ~isempty(C.abortado)
                fprintf('  !! ABORTADO: %s\n     El AO se ha llevado a 0 V. Los datos hasta el aborto se guardan.\n', C.abortado);
            end
            E = zeros(n, 4);
            for i = 1:4, E(:,i) = Prensa.escala(C.reg(1:n,i), Prensa.CANALES(i)); end
            r = struct('t', (0:n-1).'/fs, 'u', u(1:n), 'posicion', E(:,1), 'fuerza', E(:,2), ...
                'presion_A', E(:,3), 'presion_B', E(:,4), 'abortado', C.abortado);
            if o.Guardar
                ruta = fullfile(Prensa.resultados(), "captura_" + o.Etiqueta + ".csv");
                writetable(table(r.t, r.u, r.posicion, r.fuerza, r.presion_A, r.presion_B, 'VariableNames', ...
                    {'t_s','u_V','posicion_mm','fuerza_kN','presion_A_bar','presion_B_bar'}), ruta);
                fprintf('[ok] %s (%d muestras, %.1f s)\n', ruta, n, n/fs);
            end
            fprintf('  Comprobar antes de dar la captura por buena: recorrido previsto, sin tramos\n');
            fprintf('  planos (sensor saturado) y temperatura del aceite al inicio y al final (§6.4).\n');
        end
    end

    %% ====================================================================
    %  9. PLANTA SIMULADA (simulink/prensa_fluidtek.slx) Y EXCITACION
    %     Sin hardware. Port de gen_excitacion.py; la planta simulada es el
    %     modelo de Jelali & Kroll del .slx, no un integrador propio.
    %% ====================================================================
    methods (Static)
        function r = simula(u_V, Ts, o)
        %SIMULA  Planta SIMULADA en lazo abierto: corre simulink/prensa_fluidtek.slx.
        %
        %   r = Prensa.simula(u, Ts)                       u en VOLTIOS, ZOH cada Ts
        %   r = Prensa.simula(u, Ts, 'X0', 75, 'Sobrescribe', {'Qp', 3.18/60000})
        %
        %   'Sobrescribe' se pasa a params_prensa (cualquier campo). Devuelve los
        %   MISMOS campos que lazoAbierto (t, u, posicion [mm], fuerza [kN],
        %   presion_A/B [bar]) mas v [mm/s], Ps [bar], fuerza_hid [kN] y xv.
                    arguments
                u_V (:,1) double, Ts (1,1) double
                o.X0 (1,1) double = 75              % [mm]
                o.Sobrescribe (1,:) cell = {}
            end
            mdl = 'prensa_fluidtek';
            carpeta = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'simulink');
            if ~isfile(fullfile(carpeta, [mdl '.slx']))
                error('Prensa:simula', 'No existe %s.slx en %s.', mdl, carpeta);
            end
            if isempty(which('params_prensa')), addpath(carpeta); end
            n = numel(u_V);  t = (0:n-1).'*Ts;
            uc = min(max(u_V, -Prensa.U_MAX_SEG), Prensa.U_MAX_SEG);
            P = params_prensa(o.Sobrescribe{:});
            P.x_ini = o.X0*1e-3;  P.t_fin = n*Ts;
            in = Simulink.SimulationInput(fullfile(carpeta, [mdl '.slx']));
            in = in.setVariable('P', P);
            in = in.setVariable('cmd', [t uc]);          % escalones, sin interpolar
            in = in.setModelParameter('StopTime', num2str(n*Ts, '%.10g'));
            out = sim(in);
            s = out.yout.getElement(1);
            tt = s.Values.Time;  y = squeeze(s.Values.Data);
            if size(y,1) ~= numel(tt), y = y.'; end
            yi = interp1(tt, y, t, 'linear', 'extrap');   % el solver es de paso variable
            r = struct('t', t, 'u', uc, 'posicion', yi(:,1), 'v', yi(:,2), ...
                'presion_A', yi(:,3), 'presion_B', yi(:,4), 'Ps', yi(:,5), ...
                'fuerza', yi(:,6), 'fuerza_hid', yi(:,7), 'xv', yi(:,8));
        end

        function R = validaModelo(varargin)
        %VALIDAMODELO  Curva estatica comando->velocidad del .slx frente a medidas.
        %   R = Prensa.validaModelo('results/caracterizacion_2026-08-12.csv', ...)
        %   Cada CSV: columna 1 = u [V], columna 2 = v [mm/s]. Simula cada nivel
        %   2.5 s y ajusta la pendiente de x en [1.5, 2.5] s. Devuelve u, v
        %   simulada y el RMS [mm/s] contra cada fichero.
            M = cellfun(@readmatrix, varargin, 'UniformOutput', false);
            us = unique(cell2mat(cellfun(@(m) m(:,1), M(:), 'UniformOutput', false)));
            vs = nan(size(us));  Ts = 0.005;
            for k = 1:numel(us)
                r = Prensa.simula(repmat(us(k), round(2.5/Ts), 1), Ts);
                w = r.t >= 1.5;
                pf = polyfit(r.t(w), r.posicion(w), 1);  vs(k) = pf(1);
                fprintf('u = %+6.2f V   v = %+8.4f mm/s   PA = %5.1f  PB = %5.1f bar\n', ...
                    us(k), vs(k), mean(r.presion_A(w)), mean(r.presion_B(w)));
            end
            rms_ = cellfun(@(m) sqrt(mean((interp1(us, vs, m(:,1)) - m(:,2)).^2)), M);
            fprintf('RMS error de velocidad: %s mm/s\n', mat2str(rms_, 4));
            R = struct('u', us, 'v', vs, 'rms', rms_);
        end

        function sec = excitacion(varargin)
        %EXCITACION  Secuencia APRBS multinivel plegada en la ventana de posicion.
        %
        %   sec = Prensa.excitacion('Seed',1,'DurTotal',600, ...)
        %
        %   Opciones (defecto): DurTotal 600 s, Ts 1 ms, UMax 10 V,
        %   VMin 0.05/60 mm/s, VMax 4 mm/s, X0 75, XLo 25, XHi 125 mm,
        %   DxObj 8 mm, UNull = U_NULL, Seed 1.
        %
        %   Tres clases de tramo: RAPIDO (dinamica), REGIMEN (mapa estatico,
        %   duracion por presupuesto de recorrido) y CERO (deriva de null). La
        %   velocidad se sortea en escala LOGARITMICA y se despeja el comando
        %   (uParaVel): asi las bandas de los ensayos normados quedan pobladas.
        %   La semilla es reproducible en MATLAB pero NO coincide con numpy.
            ap = inputParser;
            ap.addParameter('DurTotal', 600);  ap.addParameter('Ts', 0.001);
            ap.addParameter('UMax', 10);       ap.addParameter('VMin', 0.05/60);
            ap.addParameter('VMax', 4);        ap.addParameter('X0', 75);
            ap.addParameter('XLo', 25);        ap.addParameter('XHi', 125);
            ap.addParameter('DxObj', 8);
            ap.addParameter('TRapMin', 0.05);  ap.addParameter('TRapMax', 0.60);
            ap.addParameter('TRegMin', 1.0);   ap.addParameter('TRegMax', 25.0);
            ap.addParameter('PCero', 0.10);    ap.addParameter('PRapido', 0.55);
            ap.addParameter('UNull', Prensa.U_NULL);
            ap.addParameter('Seed', 1);
            ap.parse(varargin{:});  o = ap.Results;

            rs = RandStream('mt19937ar', 'Seed', o.Seed);
            tramos = zeros(0,2);  clases = strings(0,1);   % [u T], clase
            x = o.X0;  xc = 0.5*(o.XLo + o.XHi);
            tAcum = 0; tPos = 0; tNeg = 0; iter = 0;

            while tAcum < o.DurTotal && iter < 1e6
                iter = iter + 1;
                r = rand(rs);
                if r < o.PCero
                    clase = "cero";  vObj = NaN;
                else
                    if r < o.PCero + o.PRapido, clase = "rapido"; else, clase = "regimen"; end
                    vObj = 10^(log10(o.VMin) + rand(rs)*(log10(o.VMax) - log10(o.VMin)));
                end

                % signo: aqui se PLIEGA la secuencia (volver al centro + equilibrar sentidos)
                margen = 0.25*(o.XHi - o.XLo);
                if x > o.XHi - margen, signo = -1;
                elseif x < o.XLo + margen, signo = +1;
                else
                    sesgo = 0.5 - 0.35*(x - xc)/(0.5*(o.XHi - o.XLo));
                    if tPos + tNeg > 0
                        sesgo = sesgo - 0.30*(tPos - tNeg)/(tPos + tNeg);
                    end
                    signo = 2*(rand(rs) < min(max(sesgo, 0.05), 0.95)) - 1;
                end

                % de velocidad objetivo a comando (cero = 0 V EXACTOS)
                if clase == "cero"
                    u = 0;
                else
                    u = Prensa.uParaVel(signo*vObj, o.UMax) + (o.UNull - Prensa.U_NULL);
                    u = min(max(u, -o.UMax), o.UMax);
                end

                vPred = Prensa.velMedida(u);
                if clase == "cero",       T = 0.3 + 1.7*rand(rs);
                elseif clase == "rapido", T = o.TRapMin + (o.TRapMax - o.TRapMin)*rand(rs);
                else,  T = min(max(o.DxObj/max(abs(vPred), 1e-4), o.TRegMin), o.TRegMax);
                end

                % recorte: nunca predecir una salida de la ventana
                if vPred > 0 && x + vPred*T > o.XHi
                    T = max(0, (o.XHi - x)/vPred);
                elseif vPred < 0 && x + vPred*T < o.XLo
                    T = max(0, (o.XLo - x)/vPred);
                end
                if T < o.Ts, continue; end

                x = x + vPred*T;
                tramos(end+1,:) = [u T];  clases(end+1,1) = clase; %#ok<AGROW>
                tAcum = tAcum + T;
                if vPred > 0, tPos = tPos + T; elseif vPred < 0, tNeg = tNeg + T; end
            end

            us = repelem(tramos(:,1), max(1, round(tramos(:,2)/o.Ts)));
            sec = struct('t', (0:numel(us)-1).' * o.Ts, 'u', us, ...
                'tramos', tramos, 'clases', clases, 'Ts', o.Ts, 'x0', o.X0, ...
                'ventana', [o.XLo o.XHi], 'seed', o.Seed);
        end

        function informe(sec)
        %INFORME  Cobertura de la secuencia, por decada de VELOCIDAD (§6.1.1).
            u = sec.u;  Ts = sec.Ts;  n = numel(u);
            fprintf('%s\nSECUENCIA DE EXCITACION (semilla %d)\n%s\n', repmat('=',1,72), sec.seed, repmat('=',1,72));
            fprintf('  duracion       : %8.1f s (%d muestras a Ts=%.1f ms)\n', n*Ts, n, Ts*1e3);
            fprintf('  tramos         : %d\n', size(sec.tramos,1));
            fprintf('  inversiones    : %d\n', sum(diff(sign(u(u ~= 0))) ~= 0));
            fprintf('  comando        : %+.3f .. %+.3f V\n', min(u), max(u));
            for cl = ["rapido" "regimen" "cero"]
                m = sec.clases == cl;
                fprintf('  tramos %-8s: %4d (%3.0f%%)  tiempo %7.1f s (%3.0f%%)\n', cl, sum(m), ...
                    100*sum(m)/numel(m), sum(sec.tramos(m,2)), 100*sum(sec.tramos(m,2))/(n*Ts));
            end
            fprintf('  muestras signo -: %.0f%%   signo +: %.0f%%\n', 100*mean(u < 0), 100*mean(u > 0));
            v = abs(Prensa.velMedida(u))*60;
            fprintf('\n  COBERTURA POR DECADA DE VELOCIDAD (lo critico: §6.1.1)\n');
            fprintf('  %26s %10s %8s\n', 'banda de |v| [mm/min]', 'muestras', '%');
            bordes = [0 0.05 0.2 1 2 5 20 100 1e9];
            for k = 1:numel(bordes)-1
                lo = bordes(k); hi = bordes(k+1);  m = sum(v >= lo & v < hi);  marca = '';
                if lo <= 0.1 && 0.1 < hi, marca = '  <-- ensayo de VIGA (0.1)'; end
                if lo <= 1.5 && 1.5 < hi, marca = '  <-- ensayo de LOSA (1.5)'; end
                fprintf('  %8.2f-%-8.2f %10d %7.1f%%%s\n', lo, min(hi,9999), m, 100*m/n, marca);
            end
            vp = Prensa.velMedida(u);
            fprintf('\n  velocidad predicha: %+.3f .. %+.3f mm/s\n', min(vp), max(vp));
        end

        function r = validaCinematica(sec)
        %VALIDACINEMATICA  Integra v = ley medida y comprueba la ventana segura.
        %   Barato (ms). Para presiones y transitorios usar Prensa.simula.
            v = Prensa.velMedida(sec.u);
            x = min(max(sec.x0 + cumsum(v)*sec.Ts, 0), Prensa.L_CARRERA*1e3);
            fprintf('\n  VALIDACION CINEMATICA (ganancias MEDIDAS)\n');
            fprintf('    rama u>0 : v = %.4f*u %+.4f\n    rama u<0 : v = %.4f*u %+.4f (asimetria %.3f)\n', ...
                Prensa.K_POS, Prensa.B_POS, Prensa.K_NEG, Prensa.B_NEG, Prensa.ASIMETRIA);
            fprintf('    comando de velocidad CERO: %+.3f V <-- el null NO esta en 0\n', Prensa.U_NULL);
            fprintf('    posicion recorrida : %7.1f .. %7.1f mm (ventana %g..%g)\n', min(x), max(x), sec.ventana);
            fuera = x < sec.ventana(1) | x > sec.ventana(2);
            if any(fuera), fprintf('    !! %d muestras (%.1f%%) FUERA de la ventana\n', sum(fuera), 100*mean(fuera));
            else, fprintf('    [ok] la secuencia se mantiene dentro de la ventana\n'); end
            r = struct('t', sec.t, 'posicion', x, 'v', v);
        end

        function guardaCsv(sec, ruta)
        %GUARDACSV  CSV t_s,u_V: el formato que consume lazoAbierto.
            d = fileparts(ruta);  if ~isempty(d) && ~isfolder(d), mkdir(d); end
            writetable(table(sec.t, sec.u, 'VariableNames', {'t_s','u_V'}), ruta);
            fprintf('[ok] %s (%d muestras)\n', ruta, numel(sec.u));
        end

        function f = grafica(r, titulo)
        %GRAFICA  Comando, posicion, fuerza y presiones de un registro (real o simulado).
            if nargin < 2, titulo = ''; end
            f = figure('Name', 'Prensa', 'Position', [100 100 900 750]);
            tl = tiledlayout(4, 1, 'TileSpacing', 'compact');  title(tl, titulo);
            ax = nexttile; plot(r.t, r.u); ylabel('u [V]'); grid on
            ax(2) = nexttile; plot(r.t, r.posicion); ylabel('x [mm]'); grid on
            ax(3) = nexttile; plot(r.t, r.fuerza); ylabel('F [kN]'); grid on
            ax(4) = nexttile; plot(r.t, r.presion_A, r.t, r.presion_B);
            ylabel('P [bar]'); xlabel('t [s]'); legend('P_A','P_B'); grid on
            linkaxes(ax, 'x');
        end
    end

    %% ====================================================================
    %  10. INTERNOS
    %% ====================================================================
    methods (Access = private)
        % ---- sesiones ----------------------------------------------------
        function d = abreAI(obj, rate, canales)
            if nargin < 3 || isempty(canales), canales = {Prensa.CANALES.id}; end
            d = daq("ni");
            for k = 1:numel(canales)
                ch = addinput(d, obj.Mods.ai, canales{k}, "Voltage");
                ch.TerminalConfig = "Differential";  ch.Range = [-10 10];
            end
            if nargin > 1 && ~isempty(rate), d.Rate = rate; end
        end

        function tao = sesionAO(obj)
            if isempty(obj.AO) || ~isvalid(obj.AO)
                obj.AO = daq("ni");
                ch = addoutput(obj.AO, obj.Mods.ao, Prensa.CANAL_AO, "Voltage");
                ch.Range = [-10 10];
            end
            tao = obj.AO;
        end

        function liberaAO(obj)
            if ~isempty(obj.AO) && isvalid(obj.AO)
                try, write(obj.AO, 0); catch, end
            end
            obj.AO = [];
        end

        function tomaDO(obj)
            if ~isempty(obj.DO) && isvalid(obj.DO), return, end
            obj.DO = daq("ni");
            addoutput(obj.DO, obj.Mods.do, "port0/line0:7", "Digital");
            write(obj.DO, Prensa.DO_SEGURO);
            fprintf('  [DO] puerto tomado, patron seguro: %s (permisivo line%d alto, UPH parada)\n', ...
                num2str(Prensa.DO_SEGURO, '%d'), Prensa.LINEA_PERMISIVO);
        end

        function liberaDO(obj)
            if ~isempty(obj.DO) && isvalid(obj.DO)
                try, write(obj.DO, Prensa.DO_SEGURO); catch, end
                fprintf('  [DO] patron seguro restaurado (UPH parada, permisivo alto)\n');
            end
            obj.DO = [];
        end

        function paraSeguro(obj)
            try, write(obj.sesionAO(), 0); catch, end
            obj.liberaDO();
        end

        function finManiobra(obj, tao, tai)
            try, write(tao, 0); catch, end
            try, stop(tai); catch, end
            obj.AO = tao;
        end

        % ---- lectura -----------------------------------------------------
        function V = leeBloque(obj, n, fs)
            d = obj.abreAI(fs);
            V = read(d, n, "OutputFormat", "Matrix");       % [n x 4] voltios
        end

        function e = estado(~, V)
            e = struct();
            for i = 1:size(V,2)
                c = Prensa.CANALES(i);
                e.(c.nombre) = mean(Prensa.escala(V(:,i), c));
            end
        end

        function e = leeEstado(obj, tai, nmin)
        %LEEESTADO  Medias de lo ACUMULADO (min nmin scans) en una AI continua.
        %   Si hay mas de 2*nmin (atraso por una pausa), solo cuentan los ultimos:
        %   los limites de seguridad se comprueban con datos recientes.
            while tai.NumScansAvailable < nmin, pause(0.002); end
            V = read(tai, tai.NumScansAvailable, "OutputFormat", "Matrix");
            V = V(max(1, end - 2*nmin + 1):end, :);
            e = obj.estado(V);
        end

        % ---- seguridad ---------------------------------------------------
        function exigeVentana(~, tao, x)
            if x < Prensa.X_MIN_SEG || x > Prensa.X_MAX_SEG
                write(tao, 0);
                error('Prensa:aborta', 'ABORTA: posicion %.1f mm fuera de [%g, %g] mm', x, Prensa.X_MIN_SEG, Prensa.X_MAX_SEG);
            end
        end

        function imprimeLimites(~)
            fprintf('  limites    : x en [%g, %g] mm · F < %g kN · P_A < %g bar · P_B < %g bar\n', ...
                Prensa.X_MIN_SEG, Prensa.X_MAX_SEG, Prensa.F_MAX_SEG, Prensa.PA_MAX_SEG, Prensa.PB_MAX_SEG);
        end

        function signo = descubreSentido(obj, tao, tai, uTanteo, tTanteo, dxMin)
        %DESCUBRESENTIDO  Que signo de comando hace CRECER la posicion medida.
        %   No se puede dar por sabido: un signo equivocado en un lazo P sobre un
        %   integrador no da error, da una carrera contra el tope.
            if nargin < 4, uTanteo = 1; tTanteo = 3; dxMin = 0.15; end
            flush(tai);  e0 = obj.leeEstado(tai, 10);
            write(tao, uTanteo);  pause(tTanteo);
            e1 = obj.leeEstado(tai, 10);
            write(tao, 0);  pause(0.5);  flush(tai);
            dx = e1.posicion - e0.posicion;
            fprintf('  tanteo %+.2f V durante %.1f s -> dx = %+.3f mm\n', uTanteo, tTanteo, dx);
            if abs(dx) < dxMin
                error('Prensa:aborta', ['ABORTA: el vastago apenas se mueve (%+.3f mm en %.1f s con %+.2f V). ' ...
                    'Comprobar: UPH encendida? amplificador alimentado? presion? seta liberada?'], dx, tTanteo, uTanteo);
            end
            signo = sign(dx);
            fprintf('  -> un comando POSITIVO %s la posicion medida\n', ternario(signo > 0, 'AUMENTA', 'DISMINUYE'));
        end

        function ok = recoloca(obj, tao, tai, destino, signo, o)
        %RECOLOCA  Lazo P lento sobre tareas ya abiertas. Lleva COMPENSACION DEL
        %   NULL (feedforward): u = U_NULL + Kp*error*signo. Sin ese termino el lazo
        %   pide tensiones cercanas a 0 que en realidad ordenan BAJAR a 0.14 mm/s.
        %   Mandan los limites MECANICOS, no la ventana: el jog existe para volver
        %   a entrar en ella. Tolerancia 1 mm: afinar mas exige accion integral.
            arguments, obj, tao, tai, destino, signo
                o.UMax (1,1) double = 1, o.Tol (1,1) double = 1, o.TMax (1,1) double = 60, o.Verboso (1,1) logical = true
            end
            Kp = 0.08;  t0 = tic;  ok = false;
            while toc(t0) < o.TMax
                e = obj.leeEstado(tai, 10);
                fallo = Prensa.compruebaLimites(e, Prensa.X_MIN_JOG, Prensa.X_MAX_JOG);
                if ~isempty(fallo)
                    write(tao, 0);  fprintf('\n  !! ABORTA recolocacion: %s\n', fallo);  return
                end
                err = destino - e.posicion;
                if abs(err) <= o.Tol
                    write(tao, 0);  ok = true;
                    if o.Verboso, fprintf('\r  [ok] en %.2f mm (error %+.2f mm)%s\n', e.posicion, err, repmat(' ',1,20)); end
                    return
                end
                u = min(max(Prensa.U_NULL + Kp*err*signo, -o.UMax), o.UMax);
                write(tao, u);
                if o.Verboso, fprintf('\r    recolocando: x=%7.2f mm  err=%+7.2f mm  u=%+5.2f V', e.posicion, err, u); end
            end
            write(tao, 0);
            fprintf('\n  !! recolocacion sin converger en %g s\n', o.TMax);
        end

        function resumenCaracterizacion(~, res)
            UL = 2.5;                                   % zona lineal
            fprintf('\n  RESUMEN\n');
            cero = res(res(:,1) == 0, 2);
            if ~isempty(cero), fprintf('    deriva de null (u=0)      : %+.4f mm/s\n', cero(1)); end
            aj = struct();
            for k = {'positivo', [0 UL]; 'negativo', [-UL 0]}.'
                m = res(:,1) > min(k{2}) & res(:,1) <= max(k{2});
                if strcmp(k{1}, 'negativo'), m = res(:,1) >= -UL & res(:,1) < 0; end
                if nnz(m) >= 2
                    pf = polyfit(res(m,1), res(m,2), 1);  aj.(k{1}) = pf;
                    fprintf('    K comando %-9s: %+.4f mm/s por V (v = %.4fu %+.4f, n=%d)\n', k{1}, pf(1), pf(1), pf(2), nnz(m));
                end
            end
            if isfield(aj, 'positivo') && isfield(aj, 'negativo')
                fprintf('    asimetria |K-/K+|         : %.3f (el modelo de 2 camaras predice 0.772)\n', abs(aj.negativo(1)/aj.positivo(1)));
                fprintf('    velocidad nula en u       : %+.3f V\n', -aj.negativo(2)/aj.negativo(1));
                fprintf('\n  -> ajustado SOLO con |u| <= %.1f V. Para adoptarlo, cambiar K_POS/K_NEG/B_POS/B_NEG.\n', UL);
            end
            altos = res(abs(res(:,1)) > UL, :);
            if ~isempty(altos)
                fprintf('\n  Puntos por encima de %.1f V, FUERA del ajuste:\n', UL);
                for k = 1:size(altos,1)
                    aviso = '';
                    if altos(k,1) > 0 && altos(k,2) > Prensa.V_MAX_EXT*1.05
                        aviso = sprintf('  <- por encima del techo de caudal (%.2f)', Prensa.V_MAX_EXT);
                    end
                    fprintf('    u=%+6.2f V -> v=%+8.4f mm/s%s\n', altos(k,1), altos(k,2), aviso);
                end
                fprintf('    Un escalon corto NO mide el regimen a comando alto (acumulador). Usar satura.\n');
            end
        end

        % ---- callbacks de la captura -------------------------------------
        function trozo = siguienteTrozo(obj, n)
            C = obj.Cap;  i = C.iEsc;  j = min(i + n, C.nTot);
            trozo = zeros(n, 1);                       % rellena con 0 V al acabar
            trozo(1:j-i) = C.u(i+1:j);
            obj.Cap.iEsc = j;
        end

        function alEscribir(obj, src)
            write(src, obj.siguienteTrozo(obj.Cap.nBuf));
        end

        function alLeer(obj, src)
            % Se escribe en obj.Cap CAMPO A CAMPO (nunca la struct entera): asi no
            % se pisa el iEsc que mantiene alEscribir.
            blq = read(src, src.ScansAvailableFcnCount, "OutputFormat", "Matrix");
            n0 = obj.Cap.nLeidas;
            m = min(size(blq,1), obj.Cap.nTot - n0);
            obj.Cap.reg(n0+1:n0+m, :) = blq(1:m, :);
            obj.Cap.nLeidas = n0 + m;
            x = Prensa.escala(blq(:,1), Prensa.CANALES(1));  F = Prensa.escala(blq(:,2), Prensa.CANALES(2));
            PA = Prensa.escala(blq(:,3), Prensa.CANALES(3)); PB = Prensa.escala(blq(:,4), Prensa.CANALES(4));
            if min(x) < Prensa.X_MIN_SEG || max(x) > Prensa.X_MAX_SEG
                obj.Cap.abortado = sprintf('posicion fuera de rango (%.1f..%.1f mm)', min(x), max(x));
            elseif max(F) > Prensa.F_MAX_SEG
                obj.Cap.abortado = sprintf('fuerza %.1f kN > %g kN', max(F), Prensa.F_MAX_SEG);
            elseif max(PA) > Prensa.PA_MAX_SEG
                obj.Cap.abortado = sprintf('presion A %.1f bar > %g bar', max(PA), Prensa.PA_MAX_SEG);
            elseif max(PB) > Prensa.PB_MAX_SEG
                obj.Cap.abortado = sprintf('presion B %.1f bar > %g bar', max(PB), Prensa.PB_MAX_SEG);
            end
            if ~isempty(obj.Cap.abortado) || obj.Cap.nLeidas >= obj.Cap.nTot
                stop(src);  return
            end
            fprintf('\r    %5.1f%%  x=%7.2f mm  F=%6.2f kN  P_A=%6.1f  P_B=%6.1f bar', ...
                obj.Cap.nLeidas/obj.Cap.nTot*100, mean(x), mean(F), mean(PA), mean(PB));
        end

        function alError(obj, src, evt)
            obj.Cap.abortado = ['error de adquisicion: ' evt.Error.message];
            try, stop(src); catch, end
        end

        function cierraCaptura(obj, d)
            try, stop(d); catch, end
            try, flush(d); catch, end
            delete(d);
            try, write(obj.sesionAO(), 0); catch, end    % estado seguro
        end
    end
end

function r = ternario(c, a, b)
    if c, r = a; else, r = b; end
end

function s = formatea(e)
    s = strjoin(cellfun(@(k) sprintf('%s=%.3f', k, e.(k)), fieldnames(e), 'UniformOutput', false), '  ');
end
