/* ===========================================================================
   CONTROLADOR PRENSA FLUIDTEK v17 - cascada fuerza -> posicion
   Formula Node de FluidtekPrensaCONidentLEDI_v16.vi  (un paso del lazo, ~20 ms)
   ===========================================================================

   QUE HACE
     1. Genera la consigna sp segun el estado st (maquina de estados).
     2. Lazo externo: convierte sp (posicion o fuerza) en consigna de posicion
        x_ref. En fuerza se divide por la rigidez estimada rig [kN/mm].
     3. Lazo interno: PI de posicion -> comando u [V] a la servovalvula.
     4. Protecciones: celda (F_corte), sobrecarga, caida de carga, limite de
        posicion, paro de emergencia, presiones (desactivado) y NaN.

   ESTADOS  st      0 reposo/aproximacion   1 carga   2 mantener   3 descarga
                    4 retorno/fin           5 FALLA
   CAUSAS   causa   0 ninguna  1 caida de carga  2 sobrecarga  3 limite de posicion
                    4 paro de emergencia  5 limite de la celda (F_corte)
                    7 limite por presiones (pa >= pa_max o F_pres >= F_corte)
   MODOS    modo    0 control de desplazamiento [mm]    1 control de fuerza [kN]

   TERMINALES (el nombre es el del terminal en la VI)
     Entradas  x        posicion del vastago [mm]
               F        fuerza de la celda [kN]
               modo     ver arriba
               t, tr    tiempo del ensayo [s] y reloj de alta resolucion [s]
               pa       presion camara A, SIN vastago [bar]  (TR44.2, ai2)
               pb       presion camara B, CON vastago [bar]  (TR49.2, ai3)
                        (llegan en bar reales; el escalado esta en el diagrama:
                         pa = (V_ai2 - 1.9826)*12.5   pb = (V_ai3 - 1.9682)*50)
               vjog     velocidad de aproximacion fuera de ensayo [mm/min]
               Flib     limite de fuerza fuera de ensayo [kN]
               sinprot  1 = sin proteccion de carga fuera de ensayo
               ini      INICIAR (>= 5 activo)      inip   valor anterior de ini
               spm      consigna manual fuera de ensayo [mm o kN]
               lim      consigna final del ensayo  rate   rampa [unidad/min]
               tc       tiempo de mantenimiento [s]   xlim limite de posicion [mm]
               paro     paro de emergencia
               stp, f0p, tsp   st, f0 y ts del paso anterior
               P        parametros avanzados (arreglo, >= 8)
               S        estado persistente (arreglo de 12)
               tkq, tqx SIN USO (vigilancia del QuantumX, retirada en v16)
     Salidas   st, f0, ts, inio, sp, prot, prog, trest, falla, causa,
               Fpko, xspo, kho, u, So (estado, 12), Lg (registro, 15)

   PARAMETROS P[]  los entrega el Formula Node ParamPID (ParamPID.c), que es el
                   unico lugar con valores por defecto y rangos. Si P trae menos de
                   8 elementos el paso termina en FALLA con u = 0 (seccion 12).
     P[0] Kx  ganancia del PI de posicion [V/mm]    P[1] Tix tiempo integral [s]
     P[2] Kf  correccion del lazo de fuerza [1/s]   P[3] rigidez inicial [kN/mm]
     P[4] caida de carga que dispara FALLA 1, fraccion del pico
     P[5] sobrecarga que dispara FALLA 2, fraccion de lim
     P[6] velocidad de retorno [mm/min]             P[7] variacion maxima de u [V/s]

   ESTADO PERSISTENTE S[] -> So[]  (llega como 12 ceros en la primera iteracion)
     0 x_ref  1 rig   2 F_pico   3 integ   4 sp_prev   5 t_prev
     6 F_ref  7 x_ref_rig  8 x_inicio  9 u_prev  10 modo_prev  11 causa_act

   REGISTRO Lg[]
     0 t  1 st  2 sp  3 F  4 x  5 x_ref  6 u  7 rig  8 dt  9 falla  10 causa
     11 prot  12 pa  13 pb  14 F_pres

   PARO: salida 0 V, controlador en seguimiento (sin salto al soltarlo).
   El historial de versiones esta en git (git log -- labview_control_software/PID.c).
   v16: misma logica que v15, verificada paso a paso con 320 000+ pasos
        aleatorios; solo cambian nombres, orden de la cabecera y se retira el
        codigo muerto del QuantumX (causa 6).
   v17: los valores por defecto de P viven solo en ParamPID.c; si P llega
        incompleto ya no se inventan defaults: FALLA con u = 0.
   =========================================================================== */


/* ---------------------------------------------------------------------------
   0. DECLARACIONES Y CONSTANTES
   (el Formula Node no tiene #define: las constantes son variables)
   --------------------------------------------------------------------------- */

/* parametros de P */
float64 Kx, Tix, Kf, rig_ini, frac_caida, frac_sobre, v_retorno, du_max, par_ok;
/* estado persistente (S) */
float64 x_ref, rig, F_pico, integ, sp_prev, t_prev, F_ref, x_ref_rig, x_inicio;
float64 u_prev, modo_prev, causa_act;
/* variables de trabajo */
float64 med, dt, v_aprox, F_lib;
float64 dif, vel_sp, paso_sp, dF_min, rig_nueva;
float64 x_ref_ini, x_ref_prev, err, u_pi;
float64 F_pres, lim_pres;
/* arreglos de salida */
float64 So[12];
float64 Lg[15];
/* constantes de seguridad y de ajuste */
float64 F_corte, pa_max, prot_pres_on, v_des, u_max, x_win, v_lenta;
float64 rig_min, rig_max, alfa_rig, frac_freno, frac_pico_min, margen_xlim;
float64 dF_min_rig, dx_min_rig, dx_sin_contacto;
float64 cFpA, cFpB;
/* codigos de estado y de causa */
float64 ST_REPOSO, ST_CARGA, ST_MANTIENE, ST_DESCARGA, ST_RETORNO, ST_FALLA;
float64 CAU_NINGUNA, CAU_CAIDA, CAU_SOBRECARGA, CAU_LIM_POS, CAU_PARO, CAU_FCORTE, CAU_PRESION;

F_corte  = 160;     /* kN: corte por la celda. 160 kN ~ 80 bar con el cilindro 160/80: BAJAR (CLAUDE.md 6) */
pa_max   = 95;      /* bar: corte por presion de la camara A (el TR44.2 satura a 100 bar) */
prot_pres_on = 0;   /* 0 = corte por presiones DESACTIVADO (desde 2026-11-03); 1 = activo */
v_des    = 40;      /* kN/min: carga/descarga en fuerza fuera de ensayo y descarga en ensayo */
u_max    = 10;      /* V: saturacion del comando (tambien umbral del anti-windup) */
x_win    = 2;       /* mm: x_ref no se separa de x mas que esto */
v_lenta  = 2;       /* mm/min: avance lento fuera de ensayo con compresion */
rig_min  = 0.02;    /* kN/mm: rigidez estimada minima */
rig_max  = 50;      /* kN/mm: rigidez estimada maxima */
alfa_rig = 0.3;     /* filtro de la rigidez estimada */
frac_freno    = 0.25;   /* avance lento si F > frac_freno*F_lib (fuera de ensayo) */
frac_pico_min = 0.2;    /* la caida de carga solo cuenta si el pico fue > 20 % de lim */
margen_xlim   = 0.5;    /* mm: tolerancia sobre el limite de posicion xlim */
dF_min_rig      = 0.05; /* kN: cambio minimo de fuerza para actualizar la rigidez */
dx_min_rig      = 0.002;/* mm: cambio minimo de posicion para actualizar la rigidez */
dx_sin_contacto = 5;    /* mm: mas que esto sin cambio de F = sin contacto */

/* Fp = A_A*pa - A_B*pb con el cilindro 160/80 mm (kN/bar = A[m2]*100):
   A_A = 201.06 cm2 (camara A, sin vastago), A_B = 150.80 cm2 (camara B, con vastago) */
cFpA = 2.0106;
cFpB = 1.5080;

ST_REPOSO = 0;  ST_CARGA = 1;  ST_MANTIENE = 2;  ST_DESCARGA = 3;  ST_RETORNO = 4;  ST_FALLA = 5;
CAU_NINGUNA = 0;  CAU_CAIDA = 1;  CAU_SOBRECARGA = 2;  CAU_LIM_POS = 3;
CAU_PARO = 4;  CAU_FCORTE = 5;  CAU_PRESION = 7;

/* parametros de ParamPID. Si P llega incompleto los valores son inertes y la
   seccion 12 deja el paso en FALLA con u = 0 */
par_ok = sizeOfDim(P,0)>=8;
Kx=0; Tix=1; Kf=0; rig_ini=1; frac_caida=1; frac_sobre=1; v_retorno=0; du_max=0;
if (par_ok) {
    Kx=P[0]; Tix=P[1]; Kf=P[2]; rig_ini=P[3];
    frac_caida=P[4]; frac_sobre=P[5]; v_retorno=P[6]; du_max=P[7];
}

/* ---------------------------------------------------------------------------
   1. MEDICION ACTUAL Y ESTADO PERSISTENTE
   --------------------------------------------------------------------------- */

/* medicion del modo activo: posicion [mm] o fuerza [kN] */
med = x;
if (modo>0.5) med = F;

/* el estado llega como 12 ceros en la primera iteracion: rig>0 indica estado valido */
if (sizeOfDim(S,0)>=12 && S[1]>0) {
    x_ref=S[0]; rig=S[1]; F_pico=S[2]; integ=S[3]; sp_prev=S[4]; t_prev=S[5];
    F_ref=S[6]; x_ref_rig=S[7]; x_inicio=S[8]; u_prev=S[9]; modo_prev=S[10]; causa_act=S[11];
} else {
    x_ref=x; rig=rig_ini; F_pico=0; integ=0; sp_prev=med; t_prev=tr;
    F_ref=F; x_ref_rig=x; x_inicio=x; u_prev=0; modo_prev=modo; causa_act=0;
}

dt = tr - t_prev;
if (dt<=0 || dt>1) dt = 0.02;   /* lectura de tiempo invalida: usa el periodo nominal */

x_ref_ini = x_ref;   /* x_ref al empezar el paso (la proteccion de fuerza lo restituye) */
prot = 0;
st = stp;  f0 = f0p;  ts = tsp;

/* ---------------------------------------------------------------------------
   2. MAGNITUDES DERIVADAS Y LIMITES
   --------------------------------------------------------------------------- */

/* fuerza estimada por las presiones [kN] y corte por presiones (hoy desactivado) */
F_pres = cFpA*pa - cFpB*pb;
lim_pres = 0;
if (prot_pres_on>0.5 && (pa>=pa_max || F_pres>=F_corte)) lim_pres = 1;

/* velocidad de aproximacion [mm/min] y limite de fuerza fuera de ensayo [kN] */
v_aprox = vjog;
if (v_aprox<=0) v_aprox = 25;
v_aprox = min(50,v_aprox);

F_lib = abs(Flib);
if (F_lib<=0) F_lib = 4;
F_lib = max(0.2,min(50,F_lib));

/* ---------------------------------------------------------------------------
   3. INICIO DEL ENSAYO
   El ensayo arranca en el flanco de subida de INICIAR (ini >= 5, inip < 5).
   inio devuelve ini para que el diagrama lo realimente como inip.
   --------------------------------------------------------------------------- */
inio = ini;

/* reinicio sin salto al iniciar o al cambiar el tipo de control */
if ((ini>=5 && inip<5) || modo!=modo_prev) {
    x_ref = x;
    integ = 0;
    sp_prev = med;
}

/* ---------------------------------------------------------------------------
   4. GENERADOR DE CONSIGNA sp (maquina de estados)
   --------------------------------------------------------------------------- */
if (ini<5) {
    /* FUERA DE ENSAYO: avance a v_aprox, retorno a v_retorno. Lento solo si se
       avanza con compresion (F > frac_freno*F_lib); el retorno nunca se frena.
       En fuerza, la consigna spm se limita a +-F_lib (o +-F_corte si sinprot). */
    st = ST_REPOSO;
    causa_act = CAU_NINGUNA;
    if (modo>0.5) {
        if (sinprot>0.5) {
            dif = max(-F_corte,min(F_corte,spm)) - sp_prev;
        } else {
            dif = max(-F_lib,min(F_lib,spm)) - sp_prev;
        }
        vel_sp = v_des;
        paso_sp = vel_sp*dt/60;
        if (paso_sp<=0) paso_sp = 1e9;
    } else {
        dif = spm - sp_prev;
        vel_sp = v_aprox;
        if (dif<0) vel_sp = v_retorno;
        if (sinprot<0.5 && dif>0 && F>frac_freno*F_lib) vel_sp = min(v_aprox,v_lenta);
        paso_sp = vel_sp*dt/60;
    }
    /* la consigna sigue a spm con pendiente limitada a paso_sp por paso */
    sp = sp_prev + max(-paso_sp,min(paso_sp,dif));
} else {
    /* EN ENSAYO */
    if (inip<5) {
        /* primer paso del ensayo: arranca la carga desde la medicion actual */
        st = ST_CARGA;
        f0 = med;
        ts = t;
        F_pico = 0;
        F_ref = F;
        x_ref_rig = x;
        x_inicio = x;
        causa_act = CAU_NINGUNA;
    }
    dif = lim - f0;
    if (st==ST_CARGA) {
        /* carga: rampa hacia lim */
        sp = f0 + sign(dif)*abs(rate)*(t-ts)/60;
        if (dif*(sp-lim)>=0) { sp=lim; st=ST_MANTIENE; ts=t; }
    } else if (st==ST_MANTIENE) {
        /* mantener lim durante tc segundos */
        sp = lim;
        if (t-ts>=tc) { st=ST_DESCARGA; ts=t; }
    } else if (st==ST_DESCARGA) {
        /* descarga: a v_retorno [mm/min] en desplazamiento, a v_des [kN/min] en fuerza */
        vel_sp = v_des;
        if (modo<0.5) vel_sp = v_retorno;
        sp = lim - sign(dif)*vel_sp*(t-ts)/60;
        if (dif*(f0-sp)>=0) { sp=f0; st=ST_RETORNO; ts=t; }
    } else {
        /* retorno/fin (4) o FALLA (5): consigna en el valor inicial */
        sp = f0;
    }
}

/* ---------------------------------------------------------------------------
   5. RIGIDEZ ESTIMADA rig = dF/dx
   En linea en todo momento salvo retorno y falla (reposo, aproximacion, carga,
   mantenimiento y descarga, en ambos modos): asi el lazo de fuerza usa la
   rigidez real.
   --------------------------------------------------------------------------- */
dF_min = dF_min_rig;
if (ini>=5) dF_min = max(dF_min_rig,dF_min_rig*abs(lim-f0));
if (st<=ST_DESCARGA && abs(F-F_ref)>dF_min && abs(x-x_ref_rig)>dx_min_rig) {
    rig_nueva = (F-F_ref)/(x-x_ref_rig);
    if (rig_nueva>rig_min) rig = rig + alfa_rig*(rig_nueva-rig);
    F_ref = F;
    x_ref_rig = x;
}
if (abs(x-x_ref_rig)>dx_sin_contacto) { F_ref=F; x_ref_rig=x; }   /* sin contacto: reinicia la referencia */
rig = max(rig_min,min(rig_max,rig));


/* ---------------------------------------------------------------------------
   6. LAZO EXTERNO: consigna de posicion x_ref
   --------------------------------------------------------------------------- */
if (modo>0.5) {
    /* control de fuerza */
    if (st==ST_RETORNO) {
        /* retorno a la posicion inicial x_inicio a v_retorno */
        x_ref = x_ref + sign(x_inicio-x_ref)*min(abs(x_inicio-x_ref),v_retorno*dt/60);
    } else if (st!=ST_FALLA) {
        /* prealimentacion de la rampa + correccion normalizada por rigidez */
        x_ref_prev = x_ref;
        x_ref = x_ref + (sp-sp_prev)/rig + Kf*(sp-F)/rig*dt;
        /* velocidad maxima del piston = v_aprox (evita embalarse sin contacto) */
        if (st<=ST_DESCARGA) x_ref = x_ref_prev + max(-v_aprox*dt/60,min(v_aprox*dt/60,x_ref-x_ref_prev));
    }
} else {
    /* control de desplazamiento */
    if (st!=ST_FALLA) x_ref = sp;
}


/* ---------------------------------------------------------------------------
   7. PROTECCION DE FUERZA FUERA DE ENSAYO
   Con compresion F > F_lib no deja avanzar; retroceder (alejarse) siempre esta
   permitido.
   --------------------------------------------------------------------------- */
if (st==ST_REPOSO && sinprot<0.5) {
    if (F>F_lib) { x_ref=min(x_ref,x_ref_ini); integ=min(integ,0); prot=1; }
    if (prot>0.5 && modo<0.5) sp = x_ref;
}


/* ---------------------------------------------------------------------------
   8. DETECCION DE FALLA, LIMITES Y PARO DE EMERGENCIA
   --------------------------------------------------------------------------- */
if (st>=ST_CARGA && st<=ST_DESCARGA) {
    F_pico = max(F_pico,F);
    if (modo>0.5 && st<=ST_MANTIENE && F_pico>frac_pico_min*abs(lim) && F<(1-frac_caida)*F_pico) {
        st=ST_FALLA; causa_act=CAU_CAIDA; x_ref=x;           /* caida de carga */
    } else if (modo>0.5 && F>frac_sobre*abs(lim)) {
        st=ST_FALLA; causa_act=CAU_SOBRECARGA; x_ref=x;      /* sobrecarga */
    } else if (x>xlim+margen_xlim) {
        st=ST_FALLA; causa_act=CAU_LIM_POS; x_ref=x;         /* limite de posicion */
    }
}
/* limite del sistema: en ensayo se declara FALLA */
if (st>=ST_CARGA && st<=ST_RETORNO && abs(F)>=F_corte) { st=ST_FALLA; causa_act=CAU_FCORTE;  x_ref=x; }
if (st>=ST_CARGA && st<=ST_RETORNO && lim_pres>0.5)    { st=ST_FALLA; causa_act=CAU_PRESION; x_ref=x; }
if (paro>0.5 && ini>=5)  { st=ST_FALLA; causa_act=CAU_PARO; }
if (st==ST_FALLA) sp = med;
if (modo>0.5) sp = max(-F_corte,min(F_corte,sp));   /* consigna de fuerza nunca sobre el corte */


/* ---------------------------------------------------------------------------
   9. LAZO INTERNO DE POSICION (PI con anti-windup) Y SEGURIDADES DE SALIDA
   --------------------------------------------------------------------------- */
x_ref = max(x-x_win,min(x+x_win,x_ref));
err = x_ref - x;
u_pi = Kx*(err+integ/Tix);
if (abs(u_pi)<u_max) integ = integ + err*dt;                 /* integra solo si no satura */
u = max(-u_max,min(u_max,u_pi));
u = max(u_prev-du_max*dt,min(u_prev+du_max*dt,u));           /* limite de variacion (V/s) */

/* LIMITE DEL SISTEMA: desde F_corte no se permite empujar mas, en cualquier estado y modo */
if (F>=F_corte)  { u=min(u,0); integ=min(integ,0); x_ref=min(x_ref,x); prot=1; }
if (F<=-F_corte) { u=max(u,0); integ=max(integ,0); x_ref=max(x_ref,x); prot=1; }
/* corte por presiones (solo si prot_pres_on = 1) */
if (lim_pres>0.5) { u=min(u,0); integ=min(integ,0); x_ref=min(x_ref,x); prot=1; }
/* PARO: 0 V y seguimiento, para que al soltarlo no haya salto */
if (paro>0.5) { u=0; integ=0; x_ref=x; sp=med; }


/* ---------------------------------------------------------------------------
   10. INDICADORES PARA LA INTERFAZ
   --------------------------------------------------------------------------- */
prog = 0;
trest = 0;
if (st==ST_MANTIENE) {
    if (tc>0) prog = 100*min(1,(t-ts)/tc);
    trest = max(0,tc-(t-ts));
}
if (st==ST_DESCARGA || st==ST_RETORNO) prog = 100;
falla = 0;
if (st==ST_FALLA) falla = 1;
causa = causa_act;
if (paro>0.5) { causa=CAU_PARO; falla=1; }   /* alarma de PARO tambien fuera de ensayo */
Fpko = F_pico;


/* ---------------------------------------------------------------------------
   11. ESTADO PARA EL SIGUIENTE PASO Y REGISTRO
   --------------------------------------------------------------------------- */
sp_prev = sp;
u_prev = u;
t_prev = tr;
modo_prev = modo;

So[0]=x_ref;  So[1]=rig;       So[2]=F_pico;  So[3]=integ;     So[4]=sp_prev; So[5]=t_prev;
So[6]=F_ref;  So[7]=x_ref_rig; So[8]=x_inicio; So[9]=u_prev;   So[10]=modo_prev; So[11]=causa_act;
xspo = x_ref;
kho = rig;

Lg[0]=t;    Lg[1]=st;   Lg[2]=sp;   Lg[3]=F;   Lg[4]=x;   Lg[5]=x_ref;
Lg[6]=u;    Lg[7]=rig;  Lg[8]=dt;   Lg[9]=falla; Lg[10]=causa; Lg[11]=prot;
Lg[12]=pa;  Lg[13]=pb;  Lg[14]=F_pres;


/* ---------------------------------------------------------------------------
   12. PROTECCION ANTE DATOS INVALIDOS
   Lectura invalida del cDAQ (NaN en posicion o fuerza) o P incompleto: salida
   0 V, FALLA en ensayo y reinicio del estado (So[1]=0 hace que el proximo paso
   lo descarte).
   --------------------------------------------------------------------------- */
if (!(x==x) || !(F==F) || par_ok<0.5) {
    u = 0;
    falla = 1;
    So[1] = 0;
    if (st>=ST_CARGA && st<=ST_RETORNO) st = ST_FALLA;
    Lg[6] = 0;
    Lg[9] = 1;
}
