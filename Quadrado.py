import RDelta
from time import sleep

print "Esperando 6 segundos para estabilizar o sistema"
sleep(5)
RDelta.inversa(0,0,-250)
print "Posicionando efetuador"
sleep(1)
RDelta.inversa(0,0,-260)
sleep(0.25)

passo=4

for i in range(0,120+passo,passo):
	RDelta.inversa(i,0,-260)
	sleep(0.15)

for i in range(0,120+passo,passo):
	RDelta.inversa(120,i,-260)
	sleep(0.15)
	
for i in range(100,0-passo,-passo):
	RDelta.inversa(i,120,-260)
	sleep(0.15)
	
for i in range(120,0-passo,-passo):
	RDelta.inversa(0,i,-260)
	sleep(0.15)