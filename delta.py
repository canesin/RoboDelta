def direta(theta1, theta2, theta3):
	

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
