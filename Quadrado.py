import RDelta
from time import sleep

print "Esperando 6 segundos para estabilizar o sistema"
sleep(5)
RDelta.inversa(0,0,-250)
print "Posicionando efetuador"
sleep(1)
RDelta.inversa(0,0,-260)
sleep(0.25)

passo=3

for i in range(0,100+passo,passo):
	RDelta.inversa(i,0,-260)
	sleep(0.1)

for i in range(0,100+passo,passo):
	RDelta.inversa(100,i,-260)
	sleep(0.1)
	
for i in range(100,0-passo,-passo):
	RDelta.inversa(i,100,-260)
	sleep(0.1)
	
for i in range(100,0-passo,-passo):
	RDelta.inversa(0,i,-260)
	sleep(0.1)