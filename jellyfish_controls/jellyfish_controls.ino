#include <Servo.h>
#include <Math.h>

Servo stroke;  // for controlling stroke motion (symmetrical)
Servo fin1;
Servo fin2;
double beta_1;
double beta_2;
double alpha_1 = 0;
double alpha_2 = 0;
int theta = 0;

double fin_len = 2.15;
double rudder_len = 0.48;
double motor_len = 0.56;
double steer_len = 3.05;

char receivedChar;
boolean startLoop = false;
boolean newData = false;

void setup() {
  // put your setup code here, to run once:
  stroke.attach(9);  // attaches the servo on pin 9 to the servo object
  fin1.attach(10);
  fin2.attach(11);
  stroke.write(0);
  fin1.write(0);
  fin2.write(0);
  Serial.begin(9600);
}

void loop() {
  recvOneChar();
  //go to max stroke diameter (180) gently
  for(theta = 0; theta < 180; theta++){
    
  }
  
  // TODO: add arrow controls of fins
  // stroke continuously scans from 0 to 180 degrees
  while(startLoop){
    for(theta = 10; theta < 180; theta++)  
    {                      
      recvOneChar();
      showNewData();            
      stroke.write(theta);
      update_rudders(theta, alpha_1, alpha_2);
      fin1.write(beta_1);
      fin2.write(beta_2);                
      delay(15);                   
    } 
    for(theta = 180; theta > 10; theta--)    
    {                                
      stroke.write(theta);
      update_rudders(theta, alpha_1, alpha_2);
      fin1.write(beta_1);
      fin2.write(beta_2);           
      delay(15);       
    }
  }
}

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        startLoop = true;
        newData = true;
    }
}

void showNewData() {
  int threshold = 90;
  if (newData == true) {
      Serial.print("This just in ... ");
      Serial.println(receivedChar);
      if (receivedChar == 's'){
        alpha_1 -= 10;
      } else if (receivedChar == 'w')
      {
        alpha_1 += 10;
      } else if (receivedChar == 'i')
      {
        alpha_2 += 10;
      } else if (receivedChar == 'k')
      {
        alpha_2 -= 10;
      }
      if (abs(alpha_1) >= threshold) {
        int sign = (alpha_1 > 0) - (alpha_1 < 0);
        alpha_1 = threshold * sign;
      }
      if (abs(alpha_2) >= threshold) {
        int sign = (alpha_2 > 0) - (alpha_2 < 0);
        alpha_2 = threshold * sign;
      }
      newData = false;
  }
}

void update_rudders(double theta, double alpha_1, double alpha_2){
  //TODO: calculate necessary beta values given gamma
  beta_1 = beta_calc(alpha_1, theta);
  beta_2 = beta_calc(alpha_2, theta);
}

double beta_calc(double alpha, double theta){
  double l = 3.05;
  double d = 1.0;
  double m_1 = 0.5;
  double m_2 = 0.5;
  //given a desired rudder angle (alpha) calculate beta
  double x_1 = fin_len*sin(theta);
  double y_1 = fin_len*-cos(theta);
  //TODO: check signs
  double x_2 = rudder_len*sin(alpha - theta) + x_1;
  double y_2 = rudder_len*cos(alpha - theta) + y_1;

  //TODO: get m_1 and m_2 offsets
  double top = 0.5*sqrt(pow(4*l*x_2 - 4*l*m_1, 2) - \ 
  4*(pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 + 2*m_1*x_2 + 2*m_2*y_2 - pow(m_1, 2) - pow(m_2, 2) - pow(x_2, 2) - pow(y_2, 2))* \
  (pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 + 2*m_1*x_2 + 2*m_2*y_2 - pow(m_1, 2) - pow(m_2, 2) - pow(x_2, 2) - pow(y_2, 2))) + \
  2*l*m_1 - 2*l*x_2; 
  double bottom = pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 + 2*m_1*x_2 + 2*m_2*y_2 - pow(m_1, 2) - pow(m_2, 2) - pow(x_2, 2) - pow(y_2, 2);

  double beta = 2*(atan(top/bottom));
  return beta;
}

double gamma_calc(double theta, double beta){
  double m_1 = 0.5;
  double m_2 = 0.5;
  double x_1 = fin_len*sin(theta);
  double y_1 = fin_len*-cos(theta);
  double c_1 = motor_len*sin(beta) + m_1;
  double c_2 = motor_len*-cos(beta) + m_2;

  double f = c_1 - x_1;
  double h = c_2 - y_1;
  double g = pow(rudder_len, 2) - pow(f, 2) - pow(h, 2);
  double l = steer_len;

  double gamma = 2*atan((sqrt(pow(l, 2)*(4*pow(f, 2) + 4*pow(h, 2) - pow(l, 2)) - pow(g, 2) + 2*g*pow(l, 2)) + 2*f*l) /
  (g - l*(2*h + l))); 
  return gamma;
}

double alpha_calc(double gamma, double theta, double beta){
  double m_1 = 0.5;
  double m_2 = 0.5;
  double c_1 = motor_len*sin(beta) + m_1;
  double c_2 = motor_len*-cos(beta) + m_2;
  double x_1 = fin_len*sin(theta);
  double y_1 = fin_len*-cos(theta);
  double x_2 = steer_len * sin(gamma) + c_1;
  double y_2 = steer_len * -cos(gamma) + c_2;

  double alpha = -asin((x_2 - x_1)/rudder_len) + theta;
  return alpha;
}
