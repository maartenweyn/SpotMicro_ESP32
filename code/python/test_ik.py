from math import *


L1=60.5
L2=10
L3=100.7
L4=118.5

x = L1
y = -(L3+L4)
z = 0

L1L2 = L1**2+L2**2
LL = 2*L1*L2

Rxy = sqrt(x**2+y**2-L1**2)
t1 =  degrees(-atan2(y, x) - atan2(Rxy,-L1))

print ("Ryz", Rxy)
print ("t1", t1)

Dxy = Rxy - L2
Dxyz = sqrt(Dxy*Dxy+x*x)
print ("Dxy", Dxy)
print ("Dxyz", Dxyz)

    
D=(Dxyz**2-L2**2-L4**2)/(2*L3*L4)

print ("D", D)
t3=degrees(atan2(-sqrt(1-D*D),D))

print ("t3", t3)

t2=degrees(atan2(z,Dxy)-atan2(L4*sin(t3),L3+L4*cos(t3)))

print ("t2", t2)