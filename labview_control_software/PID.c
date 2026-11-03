/* ===========================================================================
   CONTROLADOR PRENSA FLUIDTEK v15 - cascada fuerza -> posicion
   Codigo del Formula Node de la VI FluidtekPrensaCONidentLEDI_v15.vi
   ===========================================================================

   QUE HACE (un paso del lazo, cada ~20 ms)
     1. Genera la consigna sp segun el estado st (maquina de estados).
     2. Lazo externo: convierte la consigna (posicion o fuerza) en una consigna
        de posicion xsp. En fuerza se divide por la rigidez estimada kh [kN/mm].
     3. Lazo interno: PI de posicion -> comando u [V] a la servovalvula.
     4. Protecciones: limite del sistema, presiones, paro de emergencia, NaN.
   El lazo esta temporizado por el cDAQ (4000 muestras a 200 kS/s = 20 ms);
   dt se mide con el reloj de alta resolucion tr.

   ESTADOS  st:    0 reposo/aproximacion   1 carga   2 mantener   3 descarga
                   4 retorno/fin           5 FALLA
   CAUSAS   causa: 0 ninguna               1 caida de carga     2 sobrecarga
                   3 limite de posicion    4 paro de emergencia
                   5 limite del sistema (Fcorte 160 / 170 kN)
                   6 sin lectura de fuerza del QuantumX (desactivada desde v14)
                   7 limite por presiones (pB >= PBmax o Fp >= Fcorte)
   MODOS    modo:  0 control de desplazamiento (mm)   1 control de fuerza (kN)
   PARO:    salida 0 V, controlador en seguimiento (sin salto al soltarlo),
            alarma visible en todo estado y la UPH se apaga (logica en el
            lazo de la UPH).

   TERMINALES (descripcion deducida del uso en este codigo)
     Entradas   x       posicion del vastago [mm]
                F       fuerza de la celda (Krenel, MEAN x 20 kN/V) [kN]
                modo    ver arriba
                t       tiempo del ensayo [s]
                tr      reloj de alta resolucion [s]   (para medir dt)
                pa, pb  presiones del panel [bar] (pa camara con vastago, pb sin vastago)
                tkq,tqx sin uso desde v14 (vigilancia del QuantumX desactivada)
                vjog    velocidad de aproximacion fuera de ensayo [mm/min]
                Flib    limite de fuerza fuera de ensayo [kN]
                sinprot 1 = sin proteccion de carga fuera de ensayo
                ini     INICIAR (>= 5 activo)       inip  valor anterior de ini
                conf    'Confirmar parametros'
                spm     consigna manual fuera de ensayo (mm o kN segun modo)
                lim     consigna final del ensayo       rate  rampa [unid/min]
                tc      tiempo de mantenimiento [s]     xlim  limite de posicion [mm]
                paro    paro de emergencia
                stp,f0p,tsp   st, f0 y ts del paso anterior
                P       parametros de 'Parametros avanzados' (arreglo, >= 8)
                S       estado persistente (arreglo de 12, ver mas abajo)
     Salidas    st, f0, ts, inio, sp, prot, prog, trest, falla, causa,
                Fpko, xspo, kho, u, So (estado, 12), Lg (registro, 15)

   PARAMETROS P[] (si P trae menos de 8 elementos se usan los valores por defecto)
     P[0] Kx    ganancia del PI de posicion [V/mm]        defecto 30
     P[1] Tix   tiempo integral [s]                       defecto 1.5
     P[2] Kf    ganancia de correccion del lazo de fuerza defecto 0.5
     P[3] k0    rigidez inicial estimada [kN/mm]          defecto 1
     P[4] caida caida de carga que dispara FALLA 1 (fraccion del pico)  0.3
     P[5] sobre sobrecarga que dispara FALLA 2 (fraccion de lim)        1.2
     P[6] vret  velocidad de retorno [mm/min]             defecto 50
     P[7] dumax variacion maxima de u [V/s]               defecto 40

   ESTADO PERSISTENTE S[] / So[] (llega como 12 ceros en la primera iteracion)
     0 xsp  1 kh   2 Fpk  3 xi   4 spo  5 tq
     6 F1   7 x1   8 x0   9 up  10 mp  11 cau

   HISTORIAL (mas reciente primero)
     v15 rev. 2026-11-02: presiones con la calibracion de CLAUDE.md 2.4 (pbc 32.293
          bar/V, pac 18.163 bar/V) y Fp con el cilindro 125/80 mm (A_A 122.72 cm2,
          A_B 72.45 cm2). Antes: escalas de la hoja de datos y cilindro 160x80, con
          lo que pbc leia ~2.7 veces menos presion que el manometro y el corte por
          presiones no actuaba a tiempo.
     v15: reserva automatica del cDAQ al iniciar (30 reintentos cada 2 s) y
          proteccion ante NaN al final del codigo.
     v14: la fuerza de control vuelve a ser la celda via modulo Krenel en el cDAQ
          (MEAN x 20 kN/V); el QuantumX solo registra. La vigilancia del dato del
          QuantumX (causa 6) queda desactivada.
     v13: F venia de la celda en el QuantumX (canal 1) via global F_QX; tqx = Tick
          Count de la ultima muestra. Sin dato nuevo en 0.3 s: salida 0 V y, en
          ensayo, FALLA causa 6.
     v13: chequeo independiente por presiones (pb camara sin vastago TR44.2
          0-100 bar, pa camara con vastago TR49.2 0-400 bar, ambos 4-20 mA). Las
          presiones del panel se reescalan a la hoja de datos: pac=pa*0.915,
          pbc=pb*1.0618. Fp = 2.0106*pbc - 1.5276*pac (kN, cilindro 160x80).
          Limite: pbc >= PBmax (95 bar, el TR44 satura en 100 bar) o Fp >= Fcorte:
          no se permite avanzar y en ensayo FALLA causa 7.
     v13: fuera de ensayo el retroceso nunca se bloquea por la proteccion de fuerza.
     v12: en control de fuerza, fuera de ensayo carga y descarga a vdes = 15
          kN/min; en ensayo la carga usa la Rampa del panel y la descarga va a vdes.
     v11: retorno a vret (P[6], 50 mm/min por defecto) en todos los casos y modos.
     v11: LIMITE DEL SISTEMA HIDRAULICO 170 kN (no configurable): desde
          Fcorte = 160 kN no se permite avanzar (u <= 0) en ningun estado ni modo;
          en ensayo pasa a FALLA (causa 5). Consigna de fuerza limitada a Fcorte.
     v11: sinprot = 1 desactiva la proteccion de carga fuera de ensayo (Flib y
          freno a 2 mm/min): el vastago va a vjog/vret sin importar la carga, pero
          el limite del sistema sigue activo.
   =========================================================================== */


/* ---------------------------------------------------------------------------
   0. DECLARACIONES Y CONSTANTES
   (el Formula Node no tiene #define: las constantes son variables)
   --------------------------------------------------------------------------- */

/* parametros de P */
float64 Kx, Tix, Kf, k0, caida, sobre, vret, dumax;
/* estado persistente (S) */
float64 xsp, kh, Fpk, xi, spo, tq, F1, x1, x0, up, mp, cau;
/* generador de consigna y lazo externo */
float64 d, dsp0, dsp, dFm, knew, xsa, xpr, pm, iniE, vj, Fl;
/* lazo interno */
float64 ex, ui, dt;
/* chequeo por presiones */
float64 pac, pbc, Fp, plim;
/* vigilancia del QuantumX (desactivada) y validez de la lectura */
float64 qxok, okm;
/* arreglos de salida */
float64 So[12];
float64 Lg[15];
/* constantes */
float64 Fcorte, vdes, PBmax, kmin, kmax, umax, xwin, vlenta, alfaK;
float64 escPA, escPB, V0DS, sDsA, sDsB, V0A, kA, V0B, kB, cFpB, cFpA;
float64 vA, vB;

Fcorte = 160;      /* kN: umbral de corte (el limite del sistema hidraulico es 170, margen 10) */
vdes   = 15;       /* kN/min: carga/descarga en fuerza fuera de ensayo y descarga en ensayo */
PBmax  = 95;       /* bar: corte por presion de la camara sin vastago (limitadora a 100 bar) */
kmin   = 0.02;     /* kN/mm: rigidez estimada minima */
kmax   = 50;       /* kN/mm: rigidez estimada maxima */
umax   = 10;       /* V: saturacion del comando (tambien umbral del anti-windup) */
xwin   = 2;        /* mm: xsp no se separa de x mas que esto */
vlenta = 2;        /* mm/min: avance lento fuera de ensayo con compresion */
alfaK  = 0.3;      /* filtro de la rigidez estimada */

/* Presiones: pa y pb llegan del panel y se convierten en dos pasos.
   (1) Panel -> tension equivalente del canal. v13 reescala el panel a la hoja de datos
       (escPA, escPB): 50 bar/V la camara con vastago (pa, ai3) y 12.5 bar/V la camara
       sin vastago (pb, ai2), cero en 2 V (4-20 mA sobre 500 ohm).
   (2) Tension -> bar con la CALIBRACION por dos puntos contra manometros
       (CLAUDE.md 2.4, 2026-08-12), no con la hoja de datos. */
escPA  = 0.915;    /* reescalado de pa a la hoja de datos */
escPB  = 1.0618;   /* reescalado de pb a la hoja de datos */
V0DS   = 2;        /* V: cero de la hoja de datos (4 mA) */
sDsB   = 50;       /* bar/V de la hoja de datos, camara con vastago (pa) */
sDsA   = 12.5;     /* bar/V de la hoja de datos, camara sin vastago (pb) */
V0B    = 1.9801;   /* V: cero calibrado de ai3, camara B (con vastago) */
kB     = 18.163;   /* bar/V calibrado de ai3 */
V0A    = 1.9697;   /* V: cero calibrado de ai2, camara A (sin vastago) */
kA     = 32.293;   /* bar/V calibrado de ai2 */

/* Fp = A_A*P_A - A_B*P_B, con el cilindro 125/80 mm (CLAUDE.md 2.3, PROVISIONAL hasta
   confirmar el diametro interior): A_A = 122.72 cm2, A_B = 72.45 cm2 (kN/bar = A[m2]*100) */
cFpB   = 1.22718;  /* kN/bar de pbc: area de la camara sin vastago */
cFpA   = 0.72453;  /* kN/bar de pac: area anular, camara con vastago */

if (sizeOfDim(P,0)>=8) {
    Kx=P[0]; Tix=P[1]; Kf=P[2]; k0=P[3]; caida=P[4]; sobre=P[5]; vret=P[6]; dumax=P[7];
} else {
    Kx=30; Tix=1.5; Kf=0.5; k0=1; caida=0.3; sobre=1.2; vret=50; dumax=40;
}


/* ---------------------------------------------------------------------------
   1. MEDICION ACTUAL Y ESTADO PERSISTENTE
   --------------------------------------------------------------------------- */

/* medicion del modo activo: posicion (mm) o fuerza (kN) */
pm = x;
if (modo>0.5) pm = F;

/* el estado llega como 12 ceros en la primera iteracion: kh>0 indica estado valido */
if (sizeOfDim(S,0)>=12 && S[1]>0) {
    xsp=S[0]; kh=S[1]; Fpk=S[2]; xi=S[3]; spo=S[4]; tq=S[5];
    F1=S[6]; x1=S[7]; x0=S[8]; up=S[9]; mp=S[10]; cau=S[11];
} else {
    xsp=x; kh=k0; Fpk=0; xi=0; spo=pm; tq=tr;
    F1=F; x1=x; x0=x; up=0; mp=modo; cau=0;
}

dt = tr - tq;
if (dt<=0 || dt>1) dt = 0.02;   /* lectura de tiempo invalida: usa el periodo nominal */

xpr = xsp;      /* xsp al empezar el paso (la proteccion de fuerza lo restituye) */
prot = 0;
st = stp;  f0 = f0p;  ts = tsp;


/* ---------------------------------------------------------------------------
   2. MAGNITUDES DERIVADAS Y LIMITES
   --------------------------------------------------------------------------- */

/* chequeo independiente por presiones */
vB = V0DS + pa*escPA/sDsB;     /* tension equivalente de ai3 (camara B, con vastago) */
vA = V0DS + pb*escPB/sDsA;     /* tension equivalente de ai2 (camara A, sin vastago) */
pac = (vB - V0B)*kB;           /* presion real, camara con vastago [bar] */
pbc = (vA - V0A)*kA;           /* presion real, camara sin vastago [bar] */
Fp = cFpB*pbc - cFpA*pac;      /* fuerza estimada por presiones [kN] */
plim = 0;
if (pbc>=PBmax || Fp>=Fcorte) plim = 1;

/* v14: F viene del Krenel (cDAQ); sin vigilancia del QuantumX */
qxok = 1;

/* velocidad de aproximacion (mm/min) y limite de fuerza fuera de ensayo (kN) */
vj = vjog;
if (vj<=0) vj = 25;
vj = min(50,vj);

Fl = abs(Flib);
if (Fl<=0) Fl = 4;
Fl = max(0.2,min(50,Fl));


/* ---------------------------------------------------------------------------
   3. INICIO DEL ENSAYO
   INICIAR solo arranca el ensayo si 'Confirmar parametros' esta activo al
   pulsarlo; si se rechaza, queda bloqueado (-1) hasta soltar INICIAR.
   --------------------------------------------------------------------------- */
iniE = ini;
if (ini>=5 && inip<5 && conf<0.5) iniE = -1;
if (ini>=5 && inip<0) iniE = -1;
inio = iniE;

/* reinicio sin salto al iniciar o al cambiar el tipo de control */
if ((iniE>=5 && inip<5) || modo!=mp) {
    xsp = x;
    xi = 0;
    spo = pm;
}


/* ---------------------------------------------------------------------------
   4. GENERADOR DE CONSIGNA sp (maquina de estados)
   --------------------------------------------------------------------------- */
if (iniE<5) {
    /* FUERA DE ENSAYO: avance a vjog, retorno a vret. Lento (2 mm/min) solo si se
       avanza con compresion (F > 0.25*Flib); el retorno nunca se frena.
       En fuerza, la consigna spm se limita a +-Flib (o +-Fcorte si sinprot). */
    st = 0;
    cau = 0;
    if (modo>0.5) {
        if (sinprot>0.5) {
            d = max(-Fcorte,min(Fcorte,spm)) - spo;
        } else {
            d = max(-Fl,min(Fl,spm)) - spo;
        }
        dsp0 = vdes;
        dsp = dsp0*dt/60;
        if (dsp<=0) dsp = 1e9;
    } else {
        d = spm - spo;
        dsp0 = vj;
        if (d<0) dsp0 = vret;
        if (sinprot<0.5 && d>0 && F>0.25*Fl) dsp0 = min(vj,vlenta);
        dsp = dsp0*dt/60;
    }
    /* la consigna sigue a spm con pendiente limitada a dsp por paso */
    sp = spo + max(-dsp,min(dsp,d));
} else {
    /* EN ENSAYO */
    if (inip<5) {
        /* primer paso del ensayo: arranca la carga desde la medicion actual */
        st = 1;
        f0 = pm;
        ts = t;
        Fpk = 0;
        F1 = F;
        x1 = x;
        x0 = x;
        cau = 0;
    }
    d = lim - f0;
    if (st==1) {
        /* carga: rampa hacia lim */
        sp = f0 + sign(d)*abs(rate)*(t-ts)/60;
        if (d*(sp-lim)>=0) { sp=lim; st=2; ts=t; }
    } else if (st==2) {
        /* mantener lim durante tc segundos */
        sp = lim;
        if (t-ts>=tc) { st=3; ts=t; }
    } else if (st==3) {
        /* descarga: a vret (mm/min) en desplazamiento, a vdes (kN/min) en fuerza */
        dsp0 = vdes;
        if (modo<0.5) dsp0 = vret;
        sp = lim - sign(d)*dsp0*(t-ts)/60;
        if (d*(f0-sp)>=0) { sp=f0; st=4; ts=t; }
    } else {
        /* retorno/fin (4) o FALLA (5): consigna en el valor inicial */
        sp = f0;
    }
}


/* ---------------------------------------------------------------------------
   5. RIGIDEZ ESTIMADA kh = dF/dx
   En linea en todo momento salvo retorno y falla (reposo, aproximacion, carga,
   mantenimiento y descarga, en ambos modos): asi el lazo de fuerza usa la
   rigidez real.
   --------------------------------------------------------------------------- */
dFm = 0.05;
if (iniE>=5) dFm = max(0.05,0.05*abs(lim-f0));
if (st<=3 && abs(F-F1)>dFm && abs(x-x1)>0.002) {
    knew = (F-F1)/(x-x1);
    if (knew>kmin) kh = kh + alfaK*(knew-kh);
    F1 = F;
    x1 = x;
}
if (abs(x-x1)>5) { F1=F; x1=x; }   /* sin contacto: reinicia la referencia */
kh = max(kmin,min(kmax,kh));


/* ---------------------------------------------------------------------------
   6. LAZO EXTERNO: consigna de posicion xsp
   --------------------------------------------------------------------------- */
if (modo>0.5) {
    /* control de fuerza */
    if (st==4) {
        /* retorno a la posicion inicial x0 a vret */
        xsp = xsp + sign(x0-xsp)*min(abs(x0-xsp),vret*dt/60);
    } else if (st!=5) {
        /* prealimentacion de la rampa + correccion normalizada por rigidez */
        xsa = xsp;
        xsp = xsp + (sp-spo)/kh + Kf*(sp-F)/kh*dt;
        /* velocidad maxima del piston = vjog (evita embalarse sin contacto) */
        if (st<=3) xsp = xsa + max(-vj*dt/60,min(vj*dt/60,xsp-xsa));
    }
} else {
    /* control de desplazamiento */
    if (st!=5) xsp = sp;
}


/* ---------------------------------------------------------------------------
   7. PROTECCION DE FUERZA FUERA DE ENSAYO
   Con compresion F > Flib no deja avanzar; retroceder (alejarse) siempre esta
   permitido (v13: ya no se bloquea el retroceso con F < -Flib).
   --------------------------------------------------------------------------- */
if (st==0 && sinprot<0.5) {
    if (F>Fl) { xsp=min(xsp,xpr); xi=min(xi,0); prot=1; }
    if (prot>0.5 && modo<0.5) sp = xsp;
}


/* ---------------------------------------------------------------------------
   8. DETECCION DE FALLA, LIMITES Y PARO DE EMERGENCIA
   --------------------------------------------------------------------------- */
if (st>=1 && st<=3) {
    Fpk = max(Fpk,F);
    if (modo>0.5 && st<=2 && Fpk>0.2*abs(lim) && F<(1-caida)*Fpk) {
        st=5; cau=1; xsp=x;                  /* caida de carga */
    } else if (modo>0.5 && F>sobre*abs(lim)) {
        st=5; cau=2; xsp=x;                  /* sobrecarga */
    } else if (x>xlim+0.5) {
        st=5; cau=3; xsp=x;                  /* limite de posicion */
    }
}
/* limite del sistema: en ensayo se declara FALLA */
if (st>=1 && st<=4 && abs(F)>=Fcorte) { st=5; cau=5; xsp=x; }
if (st>=1 && st<=4 && plim>0.5)       { st=5; cau=7; xsp=x; }
if (qxok<0.5 && st>=1 && st<=4)       { st=5; cau=6; xsp=x; }
if (paro>0.5 && iniE>=5)              { st=5; cau=4; }
if (st==5) sp = pm;
if (modo>0.5) sp = max(-Fcorte,min(Fcorte,sp));   /* consigna de fuerza nunca sobre el corte */


/* ---------------------------------------------------------------------------
   9. LAZO INTERNO DE POSICION (PI con anti-windup) Y SEGURIDADES DE SALIDA
   --------------------------------------------------------------------------- */
xsp = max(x-xwin,min(x+xwin,xsp));
ex = xsp - x;
ui = Kx*(ex+xi/Tix);
if (abs(ui)<umax) xi = xi + ex*dt;                 /* integra solo si no satura */
u = max(-umax,min(umax,ui));
u = max(up-dumax*dt,min(up+dumax*dt,u));           /* limite de variacion (V/s) */

/* LIMITE DEL SISTEMA (170 kN): desde Fcorte no se permite empujar mas, en
   cualquier estado y modo */
if (F>=Fcorte)  { u=min(u,0); xi=min(xi,0); xsp=min(xsp,x); prot=1; }
if (F<=-Fcorte) { u=max(u,0); xi=max(xi,0); xsp=max(xsp,x); prot=1; }
/* chequeo independiente por presiones */
if (plim>0.5)   { u=min(u,0); xi=min(xi,0); xsp=min(xsp,x); prot=1; }
/* seguimiento: al soltar PARO no hay salto */
if (paro>0.5)   { u=0; xi=0; xsp=x; sp=pm; }
/* sin dato de fuerza: 0 V, sin movimiento */
if (qxok<0.5)   { u=0; xi=0; xsp=x; sp=pm; }


/* ---------------------------------------------------------------------------
   10. INDICADORES PARA LA INTERFAZ
   --------------------------------------------------------------------------- */
prog = 0;
trest = 0;
if (st==2) {
    if (tc>0) prog = 100*min(1,(t-ts)/tc);
    trest = max(0,tc-(t-ts));
}
if (st==3 || st==4) prog = 100;
falla = 0;
if (st==5) falla = 1;
causa = cau;
if (qxok<0.5) { causa=6; falla=1; }
if (paro>0.5) { causa=4; falla=1; }   /* alarma de PARO tambien fuera de ensayo */
Fpko = Fpk;


/* ---------------------------------------------------------------------------
   11. ESTADO PARA EL SIGUIENTE PASO Y REGISTRO
   --------------------------------------------------------------------------- */
spo = sp;
up = u;
tq = tr;
mp = modo;

So[0]=xsp; So[1]=kh; So[2]=Fpk; So[3]=xi; So[4]=spo; So[5]=tq;
So[6]=F1;  So[7]=x1; So[8]=x0;  So[9]=up; So[10]=mp; So[11]=cau;
xspo = xsp;
kho = kh;

Lg[0]=t;    Lg[1]=st;   Lg[2]=sp;   Lg[3]=F;   Lg[4]=x;   Lg[5]=xsp;
Lg[6]=u;    Lg[7]=kh;   Lg[8]=dt;   Lg[9]=falla; Lg[10]=causa; Lg[11]=prot;
Lg[12]=pac; Lg[13]=pbc; Lg[14]=Fp;


/* ---------------------------------------------------------------------------
   12. v15: PROTECCION ANTE NaN
   Lectura invalida del cDAQ (NaN en posicion o fuerza): salida 0 V, FALLA en
   ensayo y reinicio del estado (So[1]=0 hace que el proximo paso lo descarte).
   --------------------------------------------------------------------------- */
okm = 1;
if (!(x==x) || !(F==F)) okm = 0;
if (okm<0.5) {
    u = 0;
    falla = 1;
    So[1] = 0;
    if (st>=1 && st<=4) st = 5;
    Lg[6] = 0;
    Lg[9] = 1;
}
