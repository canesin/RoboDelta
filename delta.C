#include <Servo.h>

/*
 * Nunchuck -- Use a Wii Nunchuck
 * Tim Hirzel http://www.growdown.com
 * 
 notes on Wii Nunchuck Behavior.
 This library provides an improved derivation of rotation angles from the nunchuck accelerometer data.
 The biggest different over existing libraries (that I know of ) is the full 360 degrees of Roll data
 from teh combination of the x and z axis accelerometer data using the math library atan2. 

 It is accurate with 360 degrees of roll (rotation around axis coming out of the c button, the front of the wii),
 and about 180 degrees of pitch (rotation about the axis coming out of the side of the wii).  (read more below)

 In terms of mapping the wii position to angles, its important to note that while the Nunchuck
 sense Pitch, and Roll, it does not sense Yaw, or the compass direction.  This creates an important
 disparity where the nunchuck only works within one hemisphere.  At a result, when the pitch values are 
 less than about 10, and greater than about 170, the Roll data gets very unstable.  essentially, the roll
 data flips over 180 degrees very quickly.   To understand this property better, rotate the wii around the
 axis of the joystick.  You see the sensor data stays constant (with noise).  Because of this, it cant know
 the difference between arriving upside via 180 degree Roll, or 180 degree pitch.  It just assumes its always
 180 roll.


 * 
 * This file is an adaptation of the code by these authors:
 * Tod E. Kurt, http://todbot.com/blog/
 *
 * The Wii Nunchuck reading code is taken from Windmeadow Labs
 * http://www.windmeadow.com/node/42
 */

#ifndef WiiChuck_h
#define WiiChuck_h

#include "WProgram.h" 
#include <Wire.h>
#include <math.h>


// these may need to be adjusted for each nunchuck for calibration
#define ZEROX 510  
#define ZEROY 490
#define ZEROZ 460
#define RADIUS 210  // probably pretty universal

#define DEFAULT_ZERO_JOY_X 124
#define DEFAULT_ZERO_JOY_Y 132



class WiiChuck {
    private:
        byte cnt;
        uint8_t status[6];		// array to store wiichuck output
        byte averageCounter;
        //int accelArray[3][AVERAGE_N];  // X,Y,Z
        int i;
        int total;
        uint8_t zeroJoyX;   // these are about where mine are
        uint8_t zeroJoyY; // use calibrateJoy when the stick is at zero to correct
        int lastJoyX;
        int lastJoyY;
        int angles[3];

        boolean lastZ, lastC;


    public:

        byte joyX;
        byte joyY;
        boolean buttonZ;
        boolean buttonC;
        void begin() 
        {
            Wire.begin();
            cnt = 0;
            averageCounter = 0;
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.send (0x40);		// sends memory address
            Wire.send (0x00);		// sends memory address
            Wire.endTransmission ();	// stop transmitting
            update();            
            for (i = 0; i<3;i++) {
                angles[i] = 0;
            }
            zeroJoyX = DEFAULT_ZERO_JOY_X;
            zeroJoyY = DEFAULT_ZERO_JOY_Y;
        }


        void calibrateJoy() {
            zeroJoyX = joyX;
            zeroJoyY = joyY;
        }

        void update() {

            Wire.requestFrom (0x52, 6);	// request data from nunchuck
            while (Wire.available ()) {
                // receive byte as an integer
                status[cnt] = _nunchuk_decode_byte (Wire.receive()); //
                cnt++;
            }
            if (cnt > 5) {
                lastZ = buttonZ;
                lastC = buttonC;
                lastJoyX = readJoyX();
                lastJoyY = readJoyY();
                //averageCounter ++;
                //if (averageCounter >= AVERAGE_N)
                //    averageCounter = 0;

                cnt = 0;
                joyX = (status[0]);
                joyY = (status[1]);
                for (i = 0; i < 3; i++) 
                    //accelArray[i][averageCounter] = ((int)status[i+2] << 2) + ((status[5] & (B00000011 << ((i+1)*2) ) >> ((i+1)*2))); 
                    angles[i] = (status[i+2] << 2) + ((status[5] & (B00000011 << ((i+1)*2) ) >> ((i+1)*2))); 

                //accelYArray[averageCounter] = ((int)status[3] << 2) + ((status[5] & B00110000) >> 4); 
                //accelZArray[averageCounter] = ((int)status[4] << 2) + ((status[5] & B11000000) >> 6); 

                buttonZ = !( status[5] & B00000001);
                buttonC = !((status[5] & B00000010) >> 1);
                _send_zero(); // send the request for next bytes

            }
        }


    // UNCOMMENT FOR DEBUGGING
    //byte * getStatus() {
    //    return status;
    //}

    float readAccelX() {
       // total = 0; // accelArray[xyz][averageCounter] * FAST_WEIGHT;
        return (float)angles[0] - ZEROX;
    }
    float readAccelY() {
        // total = 0; // accelArray[xyz][averageCounter] * FAST_WEIGHT;
        return (float)angles[1] - ZEROY;
    }
    float readAccelZ() {
        // total = 0; // accelArray[xyz][averageCounter] * FAST_WEIGHT;
        return (float)angles[2] - ZEROZ;
    }

    boolean zPressed(boolean raw = false) {
        return (buttonZ && (raw || ! lastZ));
    }
    boolean cPressed(boolean raw = false) {
        return (buttonC && (raw || ! lastC));
    }

    // for using the joystick like a directional button
    boolean rightJoy(int thresh=60) {
        return (readJoyX() > thresh and lastJoyX <= thresh);
    }

    // for using the joystick like a directional button
    boolean leftJoy(int thresh=60) {
        return (readJoyX() < -thresh and lastJoyX >= -thresh);
    }


    int readJoyX() {
        return (int) joyX - zeroJoyX;
    }

    int readJoyY() {
        return (int)joyY - zeroJoyY;
    }


    // R, the radius, generally hovers around 210 (at least it does with mine)
   // int R() {
   //     return sqrt(readAccelX() * readAccelX() +readAccelY() * readAccelY() + readAccelZ() * readAccelZ());  
   // }


    // returns roll degrees
    int readRoll() {
        return (int)(atan2(readAccelX(),readAccelZ())/ M_PI * 180.0);
    }

    // returns pitch in degrees
    int readPitch() {        
        return (int) (acos(readAccelY()/RADIUS)/ M_PI * 180.0);  // optionally swap 'RADIUS' for 'R()'
    }

    private:
        byte _nunchuk_decode_byte (byte x)
        {
            x = (x ^ 0x17) + 0x17;
            return x;
        }

        void _send_zero()
        {
            Wire.beginTransmission (0x52);	// transmit to device 0x52
            Wire.send (0x00);		// sends one byte
            Wire.endTransmission ();	// stop transmitting
        }

};

#endif

 // -------------------------------- Cinemática direta -----------------------------------------
 // Cinemática direta: (theta1, theta2, theta3) -> (x0, y0, z0)
 // Status do retorno: 0=OK, -1=posição fora do espaço de trabalho
 int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1)); // y junta esférica braço-haste 1, J1, alinhado com eixo Y
     float z1 = -rf*sin(theta1); // z junta esférica braço-haste 1, J1, alinhado com eixo Y
 
     float y2 = (t + rf*cos(theta2))*sin30; // y junta esférica braço-haste 2, J2, girando anti-horário de J1
     float x2 = y2*tan60; // x junta esférica braço-haste 2, J2, girando anti-horário de J1
     float z2 = -rf*sin(theta2); // z junta esférica braço-haste 2, J2, girando anti-horário de J1
 
     float y3 = (t + rf*cos(theta3))*sin30; // y junta esférica braço-haste 3, J3, girando horário de J1
     float x3 = -y3*tan60; // x junta esférica braço-haste 3, J3, girando horário de J1
     float z3 = -rf*sin(theta3); // z junta esférica braço-haste 3, J3, girando horário de J1
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // Ponto fora da área de trabalho
 
     z0 = -(float)0.5*(b+sqrt(d))/a; // Devolve a coordenada Z do ponto do manipulador
     x0 = (a1*z0 + b1)/dnm; // Devolve a coordenada X do ponto do manipulador
     y0 = (a2*z0 + b2)/dnm; // Devolve a coordenada Y do ponto do manipulador
     return 0;
 }
 // --------------------------------Fim Cinemática direta ---------------------------------------

 // -------------------------------- Cinemática inversa -----------------------------------------
 // Funções de apoio, calcula anglo theta1 (para o plano YZ)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // realiza projeção da junta
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // Ponto fora do espaço de trabalho
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // Escolhendo ponto externo
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 // Cinemática inversa: (x0, y0, z0) -> (theta1, theta2, theta3)
 // Status do retorno: 0=OK, -1=posição fora do espaço de trabalho
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotaciona +120 deg
     if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotaciona -120 deg
     return status;
 }
 // -------------------------------- Fim Cinemática inversa ---------------------------------------


// CODE by prutsers.wordpress.com

Servo servo1;
Servo servo2;
Servo servo3;

float xPos = 0;  // x axis position
float yPos = 0; // y axis position
float zPos = -200; // z axis position, negative is up, positive is down. Delta bots are usually used up side down...

float t1;  //servo angle t for 'theta', 1 for servo 1
float t2;
float t3;
float oldT1;
float oldT2;
float oldT3;

int result;

#define MAXANGLE 90
#define MINANGLE -90
WiiChuck chuck = WiiChuck();

void setup(){
  servo1.attach(10);
  servo2.attach(11);
  servo3.attach(12);
  
  Serial.begin(38400);

  chuck.begin();
  chuck.update();
}

float updateRate = 5.0;
float updateRateZ = 10.0;
float threshold = 3.0;

float zDevSq = 5;

float joyAmplify = 1.5;

void loop(){
  chuck.update();
  xPos = xPos*(updateRate - 1.0) / updateRate + joyAmplify * chuck.readJoyX()/updateRate;
  yPos = yPos*(updateRate - 1.0) / updateRate + joyAmplify * chuck.readJoyY()/updateRate;
  
  // a simple statistical filter for removing pitch sensor noise
  int z = -120 - chuck.readPitch();
  float dZSq = (zPos - z)*(zPos - z);
  // update moving standard deviation
  if (dZSq < 9*zDevSq)  // 3 stdevs from mean, both sides squared
  {
    zDevSq = zDevSq * (updateRateZ - 1) + dZSq;
    zPos = zPos*(updateRateZ - 1.0) / updateRateZ + z / updateRateZ;
  }
  
  //if (chuck.zPressed(true)) zPos += 1.0;
  //if (chuck.cPressed(true)) zPos -= 1.0;
  
  result = delta_calcInverse(xPos, yPos, zPos, t1, t2, t3);

  Serial.print(result == 0 ? "ok" : "no");
  char buf[10];
  dtostrf(xPos, 4, 0, buf);
  Serial.print(" X");
  Serial.print(buf);
  dtostrf(yPos, 4, 0, buf);
  Serial.print(" Y");
  Serial.print(buf);
  dtostrf(zPos, 4, 0, buf);
  Serial.print(" Z");
  Serial.print(buf);

  dtostrf(t1, 6, 2, buf);
  Serial.print(" T1");
  Serial.print(buf);
  dtostrf(t2, 6, 2, buf);
  Serial.print(" T2");
  Serial.print(buf);
  dtostrf(t3, 6, 2, buf);
  Serial.print(" T3");
  Serial.print(buf);

  Serial.println("");
  
  if (result == 0) {
    if (abs(oldT1 - t1) > threshold || abs(oldT2 - t2) > threshold || abs(oldT3 - t3) > threshold)
    {
      servo1.write(t1+90.0-14.0); //-14 is the correction for the angle for my serve
      oldT1 = t1;
      servo2.write(t2+90.0-10.0);
      oldT2 = t2;
      servo3.write(t3+90.0-00.0);
      oldT3 = t3;
    }
  }
  //delay(5);
}