import sys; sys.path.insert(0,'/root/gen')
from cade2 import Sheet
from lay import *
import lay; lay.DX=24
from power import motor_branch
O='/mnt/user-data/outputs/cadesimu/v8/'
import os; os.makedirs(O,exist_ok=True)
def NO(n,t=('13','14')): return E('7000','-'+n,list(t))
def NC(n,t=('21','22')): return E('7001','-'+n,list(t))
def coil(n): return E('9000','-'+n)
def lamp(n,c='0'): return E('9008','-'+n,['X1','X2',c])
def SW(n,t=('13','14')): return E('8008','-'+n,list(t))
def SWc(n,t=('11','12')): return E('8009','-'+n,list(t))
def ev(n): return E('9003','-'+n)
def detc(n): return E('7532','-'+n,['1','2'])
def cyl(s,x,y,n,vp='10',vn='10'):
    d='-'+n
    s.comp('10501',x,y,d,fields={2:vp,3:vn,4:d+'0',6:d+'1',7:d+'0',9:d+'1',10:d+'+'},suffix='#'+d+'-')
s=Sheet('FLUIDTEK UPH50'); s.flag=3; s.size=(400,287)
motor_branch(s,24,40,[('2004','-QA1'),('6009','-GM1'),('2001','-KM1')],'-M1')
motor_branch(s,62,40,[('6009','-GM2'),('2001','-KV1')],'-M2')
control_aligned(s,98,40,[
  S(SW('DO0'), coil('K1')),
  S(SW('DO1'), coil('K2')),
  S(SW('DO5'), coil('K6')),
  S(NO('K1',('11','14')), coil('KM1')),
  S(NO('K2',('11','14')), coil('KV1')),
  S(NO('K6',('11','14')), lamp('PT')),
])
control_aligned(s,98,140,[
  S(SWc('S0',('21','22')), lamp('DI0')),
  S(SWc('SL1',('1','2')), lamp('DI1')),
  S(SWc('FL1',('1','2')), lamp('DI2')),
  S(NC('GM1'), lamp('DI5')),
  S(NC('GM2'), lamp('DI6')),
  S(NO('KM1'), lamp('DI7')),
  S(SW('UEXT'), detc('A1'), ev('A+')),
  S(SW('URET'), detc('A0'), ev('A-')),
])
cyl(s,270,50,'A')
s.save(O+'FLUIDTEK_UPH50.cad')
print('ok')
