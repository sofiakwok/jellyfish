#include <Servo.h>
#include <Math.h>

Servo stroke;  // for controlling stroke motion (symmetrical)
Servo fin1;
Servo fin2;

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
  // put your main code here, to run repeatedly:
  delay(1000);
  // TODO: add arrow controls of stroke & fins
  // scan from 0 to 180 degrees
  for(theta = 10; theta < 180; theta++)  
  {                                  
    stroke.write(angle);               
    delay(15);                   
  } 
  // now scan back from 180 to 0 degrees
  for(theta = 180; theta > 10; theta--)    
  {                                
    stroke.write(angle);           
    delay(15);       
  }
}

double gamma_calc(double theta, double beta){
  double fin_len = 2.15;
  double rudder_len = 0.48;
  double motor_len = 0.56;
  double steer_len = 3.05;

  double x_1 = fin_len*sin(theta);
  double x_2 = fin_len*-cos(theta);
  double c_1 = motor_len*sin(beta) + m_1;
  double c_2 = motor_len*-cos(beta) + m_2;

  double f = c_1 - x_1;
  double h = c_2 - y_1;
  double g = pow(rudder_len, 2) - pow(f, 2) - pow(h, 2);
  double l = steer_len;

  gamma = 2*atan((sqrt(pow(l, 2)*(4*pow(f, 2) + 4*pow(h, 2) - pow(l, 2)) - pow(g, 2) + 2*g*pow(l, 2)) + 2*f*l) /
  (g - l*(2*h + l))); 
  return gamma;
}

double alpha_calc(double gamma, double theta, double beta){
  double fin_len = 2.15;
  double steer_len = 3.05;
  double rudder_len = 0.48;

  double c_1 = motor_len*sin(beta) + m_1;
  double c_2 = motor_len*-cos(beta) + m_2;
  double x_1 = fin_len*sin(theta);
  double x_2 = fin_len*-cos(theta);
  double x_2 = steer_len * sin(gamma) + c_1;
  double y_2 = steer_len * -cos(gamma) + c_2;

  double alpha = -asin((x_2 - x_1)/rudder_len) + theta;
  return alpha;
}

