# -*- coding: utf-8 -*-

from math import pi, cos, sqrt, atan, sin
from serial import Serial

# Parâmetros geométricos do robo
f = 264.85
e = 126.22
re = 280.0
rf = 70.0

# Constantes trigonométricas
sin120 = sqrt(3)/2.0
cos120 = -0.5
tan60 = sqrt(3)
sin30 = 0.5
tan30 = 1/sqrt(3)
t = (f-e)*tan30/2.0
dtr = pi/180.0
usbport = '/dev/tty.usbserial-A700fkJk'
ser = Serial(usbport, 9600, timeout=1)

def move(servo, angulo):
	""" Move servo para angulo: move(servo,angulo)"""

	if (0 <= angulo <= 180):
		ser.write(chr(255))
		ser.write(chr(servo))
		ser.write(chr(int(angulo)))
		return 0
	else:
		return -999

def direta(theta1, theta2, theta3):
	""" Cinemática direta: (theta1, theta2, theta3) -> (x0, y0, z0)
	Status do retorno: -999 = posição fora do espaço de trabalho"""
	
	theta1 *= dtr
	theta2 *= dtr
	theta3 *= dtr
	y1 = -(t + rf*cos(theta1)) #y junta esférica braço-haste 1, J1, alinhado com eixo Y
	z1 = -rf*sin(theta1) #junta esférica braço-haste 1, J1, alinhado com eixo Y
	y2 = (t + rf*cos(theta2))*sin30 #y junta esférica braço-haste 2, J2, girando anti-horário de J1
	x2 = y2*tan60 #x junta esférica braço-haste 2, J2, girando anti-horário de J1
	z2 = -rf*sin(theta2) #z junta esférica braço-haste 2, J2, girando anti-horário de J1
	y3 = (t + rf*cos(theta3))*sin30 #y junta esférica braço-haste 3, J3, girando horário de J1
	x3 = -y3*tan60 #x junta esférica braço-haste 3, J3, girando horário de J1
	z3 = -rf*sin(theta3) #z junta esférica braço-haste 3, J3, girando horário de J1
	dnm = (y2-y1)*x3-(y3-y1)*x2
	w1 = y1*y1 + z1*z1
	w2 = x2*x2 + y2*y2 + z2*z2
	w3 = x3*x3 + y3*y3 + z3*z3
	a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1)
	b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0
	a2 = -(z2-z1)*x3+(z3-z1)*x2
	b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0
	a = a1*a1 + a2*a2 + dnm*dnm
	b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm)
	c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

	#discriminante
	d = b**2 - 4.0*a*c
	if (d < 0):
		return -999 #Ponto fora da área de trabalho
	else:
		z0 = -0.5*(b+sqrt(d))/a #Devolve a coordenada Z do ponto do manipulador
		x0 = (a1*z0 + b1)/dnm #Devolve a coordenada X do ponto do manipulador
		y0 = (a2*z0 + b2)/dnm #Devolve a coordenada Y do ponto do manipulador
		return (x0,y0,z0)

def CalculaAngulo(x0,y0,z0):
	"""Funções de apoio, calcula anglo theta1 (para o plano YZ)"""

	y1 = -0.5*0.57735*f #f/2 * tg 30
	y0 -= 0.5*0.57735*e #realiza projeção da junta
	a = (x0**2 + y0**2 + z0**2 +rf**2 - re**2 - y1**2)/(2*(z0+1e-3))
	b = (y1-y0)/(z0+1e-3)
	#discriminant
	d = -(a+b*y1)*(a+b*y1)+rf*((b**2)*rf+rf)
	if (d < 0):
		return (0,-999) #Ponto fora do espaço de trabalho
	else:
		yj = (y1 - a*b - sqrt(d))/(b*b + 1) #Escolhendo ponto externo
		zj = a + b*yj
		return ((180.0*atan(-zj/(y1 - yj))/pi + (180.0 if yj>y1 else 0.0)),0)

def inversa(x0,y0,z0):
	"""Cinemática inversa: inversa(x0, y0, z0) -> (theta1, theta2, theta3)
	Status do retorno: -999 = posição fora do espaço de trabalho"""

	status = CalculaAngulo(x0, y0, z0)
	if (status[1] == 0):
		move(1,136-status[0])
		status = CalculaAngulo(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0) #rotaciona +120 deg
		if (status[1] == 0):
			move(2,133-status[0])
			status = CalculaAngulo(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0) #rotaciona -120 deg
			if (status[1] == 0):
				move(3,status[0]+27)
	return status[1]