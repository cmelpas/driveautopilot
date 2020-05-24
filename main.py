import serial
import pynmea2
import io
import time
import threading
import utm
import numpy as np

class BackInput(threading.Thread):
        def __init__(self):
                super(BackInput, self).__init__()

        def run(self):
                self.input = raw_input()



port = "/dev/ttyACM0"

def parseGPS(gnss):
	if (gnss.find("GGA") > 0):
		msg = pynmea2.parse(gnss)
		#print ("%s,%s,%s,%s,%s,%s" % (msg.timestamp,msg.latitude,msg.longitude,msg.altitude,msg.num_sats,msg.geo_sep))
		nmea = str(msg.timestamp)+","+str(msg.latitude)+","+str(msg.longitude)+","+str(msg.altitude)+","+str(msg.num_sats)+","+str(msg.geo_sep)
	else:
		nmea = ""
	return nmea

def input_thread(a_list):
	raw_input()
	a_list.append(True)


def paralela(m, d):

        c1 = -np.sqrt(d*d*(1-(m*m/(1+m*m))))-m*np.sqrt(d*d*(m*m/(1+m*m)))
        c2 = np.sqrt(d*d*(1-(m*m/(1+m*m))))+m*np.sqrt(d*d*(m*m/(1+m*m)))

        return c1,c2


def distpoint(distX,distY,distPend,distC):
        j = 1
	PD = []
	PD.append(999999999999999)
	#print distC
        while (j < len(distC)+1):
                PD.append((distX+distPend*distY+distC[j-1])/np.sqrt(1+distPend*distPend))
		#Paramos de comparar si empeoramos
		if (abs(PD[j])>abs(PD[j-1])):
			break
                j+=1

        return PD




serialPort = serial.Serial(port, baudrate = 9600)
sio = io.TextIOWrapper(io.BufferedRWPair(serialPort, serialPort))
#print("Time,Latitud,Longitud,Altura,Satelites,Direccion\n")

#Seleccinoamos punto A

total = 100 #numero de paralelas
cont = 0
distancia = 10
C = []
errortodo = []

while True:
	gnss = str(sio.readline())
	#print (gnss.find("GGA"))
	con = parseGPS(gnss)
	if(con ==""):
		#print("Sin posicion GPS")
		continue

	else:
		if (cont == 0):
			print("GPS sincronizado ya puede registrar puntos A-B")

	        threading1 = BackInput()
	        threading1.start()
	        threading1.join()
	        if (threading1.input == "s"):
	                print ("Salimos")
	                break
		elif (threading1.input == "a"):
			print ("Posicion A")
			A = con.split(",")

		elif (threading1.input == "b"):
			print ("Posicion B")
			B = con.split(",")
		else:
	        	print ('Pulse: s, a , b')
	
		cont += 1



print("A:  %s y B:  %s" % (A, B))


utmA = utm.from_latlon(float(A[1]),float(A[2]))
utmB = utm.from_latlon(float(B[1]),float(B[2]))

print("\nUTM: A %s y B  %s" % (utmA, utmB))

#UTMA = utmA.split(",")
#UTMB = utmB.split(",")

#Calculamos la ecuacion de la recta como referencia A

xA = 0
yA = 0

xB = utmB[0]-utmA[0]
yB = utmB[1]-utmA[1]

#(X - Xa)*(yB-yA) = (Y - yA)*(xB-xA)
#en nuestro caso A 0,0  X*yB  -Y*xB  = 0

pendParal = xB/yB
pendPerpen = yB/xB

print ("Ecuacion de la recta: %f*X - %f*Y + %d = 0") % (yB, xB,0)
print ("Ecuacion de la recta simplificada: %f*X - %f*Y + %d = 0") % (1, pendParal,0)
print ("Ecuacion de la recta perpendicular: %f*X - %f*Y + %d = 0") % (1, pendPerpen,0)


#Teniendo de referencia 0,0 calcular la distancia: d = raiz((ax-bx)2+(ay-by)2) -> d2 = bx2 + by2
# Intersecion de circunferencia origen 0,0 y radio distancia -> raiz((x-a)2+(y-b)2)=r -> raiz(x2+y2) = r -> x2 + y2 = r2 -> x2 = r2 - y2
#x = raiz(r2 - y2) //////// X + (yB/xB)*Y = 0
#Todo: raiz(r2 - y2) + (yB/xB)Y = 0 -> r2 - y2 = (yB*Y)2/xB2 -> y2(1 - yB2/xB2) = r2

#y = raiz(distancia2 -raiz(distancia2/

i = 0

C.append(0)

while i< total:

	temp1, temp2 = paralela(pendParal, distancia)
	C.append(temp1)
	C.insert(0,temp2)

	print ("Ecuacion de la recta paralela %d con distancia %d: %f*X - %f*Y + %f = 0") % (i,distancia,1, pendParal,temp1)
	distancia+=10
	i+=1

#Rectas
#print C

#Calculamos la distancia a la recta mas proxima

lat = 100
lon = 100
print ("Distancia de %f,%f a recta paralela mas proxima")%(lat,lon)

errortodo = distpoint(5,5,pendParal,C)
error = errortodo[len(errortodo)-2]

print ("Error minimo encontrado %f en recta %d")% (error,len(errortodo)-1)
print (C[len(errortodo)-1])

if(abs(error) < 0.5):
	if(error>0):
		print ("00000 Giro izquierda X grados")
	else:
                print ("10000 Giro derecha X grados")

elif(abs(error) < 1):
        if(error>0):
                print ("00001 Giro izquierda X grados")
        else:
                print ("10001 Giro derecha X grados")

elif(abs(error)< 1.5):
        if(error>0):
                print ("00010 Giro izquierda X grados")
        else:
                print ("10010 Giro derecha X grados")

elif(abs(error) < 2):
        if(error>0):
                print ("00011 Giro izquierda X grados")
        else:
                print ("10011 Giro derecha X grados")

elif(abs(error)< 2.5):
        if(error>0):
                print ("00100 Giro izquierda X grados")
        else:
                print ("10100 Giro derecha X grados")


elif(abs(error) < 3):
        if(error>0):
                print ("00101 Giro izquierda X grados")
        else:
                print ("10101 Giro derecha X grados")

elif(abs(error)< 3.5):
        if(error>0):
                print ("00111 Giro izquierda X grados")
        else:
                print ("10111 Giro derecha X grados")


elif(abs(error) < 4):
        if(error>0):
                print ("01000 Giro izquierda X grados")
        else:
                print ("10100 Giro derecha X grados")

elif(abs(error)< 4.5):
        if(error>0):
                print ("01001 Giro izquierda X grados")
        else:
                print ("11001 Giro derecha X grados")


elif(abs(error) < 5):
        if(error>0):
                print ("01011 Giro izquierda X grados")
        else:
                print ("11011 Giro derecha X grados")

elif(abs(error)< 5.5):
        if(error>0):
                print ("0111 Giro izquierda X grados")
        else:
                print ("1111 Giro derecha X grados")



else:
	print ("Demasiado error para autoguiado")
