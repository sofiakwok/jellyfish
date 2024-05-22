#include <Arduino.h>
#include <Servo.h>
#include <Math.h>
#include <Complex.h>
#include <String.h>
#include <HardwareSerial.h>

// Arduino code to control stroke and rudder angles.
// Takes in a command from python using serial and uses that to determine controls. 

Servo stroke;  // for controlling stroke motion (symmetrical)
Servo fin1;
Servo fin2;
//beta_1 is left (looking from below), beta_2 is right 
double beta_1;
double beta_2;
//alpha: angle offset of rudders from fins (radians)
//left = negative, right = positive
double alpha_1 = 0;
double alpha_2 = 0;
double beta_1_offset = 4;
double beta_2_offset = 9;
//double k_air = 1.5;
//theta: angle of middle servo, controls fin angles
double theta = 0;

bool startLoop = false;
bool newData = false;
double delay_time = 3;

const int BUFFER_SIZE = 15;
char buf[BUFFER_SIZE];
char *command[16]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;

double starting_angle = 170;

void setup() {
  // put your setup code here, to run once:
  stroke.attach(9); // for controlling theta
  fin1.attach(10); // for controlling rudder 1
  fin2.attach(11); // for controlling rudder 2
  stroke.write(180);
  fin1.write(180 - starting_angle - beta_1_offset); //because fin1 is flipped
  fin2.write(starting_angle + beta_2_offset);
  Serial.begin(9600);
}

void loop() {
  // wait until a new command is sent
  while (!Serial.available()){}
  while (Serial.available()){
    int message = Serial.readBytesUntil('\n', buf, BUFFER_SIZE);
  }
  // split command into separate angle commands using comma
  ptr = strtok(buf, ","); 
  int index = 0;
  while (ptr != NULL)
  {
    command[index] = ptr;
    index++;
    ptr = strtok(NULL, ",");
  }
  
  theta = atof(command[0]);
  alpha_1 = atof(command[1]);
  alpha_2 = atof(command[2]);

  stroke.write(theta);
  update_rudders(180 - theta, alpha_1, alpha_2);
  fin1.write(180 - beta_1 - beta_1_offset);
  fin2.write(beta_2 + beta_2_offset);  
}

void old_loop() {
    // stroke continuously scans from 0 to 180 degrees
    // fins actually go from 0 to 90 because of the 2:1 gear ratio
    // jellyfish can do 60 deg in 0.1 sec
  while(true){
    //Serial.print("\n Loop 1 \n");
    for(theta = 180; theta > 0; theta--)  
    {              
      stroke.write(theta);
      update_rudders(180 - theta, alpha_1, alpha_2);
      //Serial.print(beta_2);
      //Serial.print(" ");
      fin1.write(180 - beta_1 - beta_1_offset);
      fin2.write(beta_2 + beta_2_offset);                
      delay(delay_time);                   
    } 
    //Serial.print("\n Loop 2 \n");
    for(theta = 0; theta < 180; theta++)    
    {                           
      stroke.write(theta);
      update_rudders(180 - theta, alpha_1, alpha_2);
      //Serial.print(beta_2);
      //Serial.print(" ");
      fin1.write(180 - beta_1 - beta_1_offset);
      fin2.write(beta_2 + beta_2_offset);           
      delay(delay_time);       
    }
  }
}

void update_rudders(double theta, double alpha_1, double alpha_2){
  bool left;
  beta_1 = beta_calc(alpha_1, theta, left=true);
  //Serial.print((String)"(alpha_1: " + alpha_1 + " theta: " + theta + " beta_1: " + beta_1 + ")");
  beta_2 = beta_calc(alpha_2, theta, left=false);
}

double beta_calc(double alpha_deg, double theta_deg, bool left){
  //convert degrees to radians and account for theta gear ratio
  double theta = theta_deg * 3.1415/180/2;
  double alpha = alpha_deg * 3.1415/180;
  //Serial.print((String)"(theta: " + theta + " alpha: " + alpha + ")");
  //all measurements in inches and taken from Solidworks
  double d = 2.45; // length of outer servo attachment to steer rudders 
  double l = 0.568898; // length of servo arm
  double r = 0.2481; // length of arm used for changing alpha from rotation axis
  double fin_len = 2.15178;
  
  // offset of servo from fin rotational axis (m_1 = x, m_2 = y)
  // assumes fin rotational axis is at (0, 0)
  double m_1 = 0;
  double m_2 = 0;
  //given a desired rudder angle (alpha) calculate beta (fin servo angle)
  double x_1 = 0;
  double y_1 = 0;
  double x_2 = 0;
  double y_2 = 0;

  double a = 0;
  double b = 0;
  double c = 0;
  double root = 0;
  Complex top(0, 0);
  Complex bottom(0, 0);

  if (left){ //for fin1 math
    m_1 = -0.311024;
    m_2 = 1.165354;
    x_1 = fin_len*sin(theta);
    y_1 = -fin_len*cos(theta);
    x_2 = r*sin(alpha + theta + 3.1415/2) + x_1;
    y_2 = -r*cos(alpha + theta + 3.1415/2) + y_1;
    a = pow(4*l*x_2 - 4*l*m_1, 2);
    b = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    c = pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    root = a - 4*b*c;
    Complex c(root, 0);
    top = c.c_sqrt();
    top *= 0.5;
    top += 2*l*m_1 - 2*l*x_2; 
    bottom.set(pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2), 0);
  } else { // for fin 2
    m_1 = 0.311024;
    m_2 = 1.165354;
    x_1 = -fin_len*sin(theta);
    y_1 = -fin_len*cos(theta);
    x_2 = -r*sin(alpha + theta + 3.1415/2) + x_1;
    y_2 = -r*cos(alpha + theta + 3.1415/2) + y_1;
    a = pow(4*l*m_1 - 4*l*x_2, 2);
    b = pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    c = pow(d, 2) - pow(l, 2) - 2*l*m_2 + 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2);
    root = a - 4*b*c;
    Complex c(root, 0);
    top = c.c_sqrt();
    top *= 0.5;
    top += -2*l*m_1 + 2*l*x_2; 
    bottom.set(pow(d, 2) - pow(l, 2) + 2*l*m_2 - 2*l*y_2 - pow(m_1, 2) + 2*m_1*x_2 - pow(m_2, 2) + 2*m_2*y_2 - pow(x_2, 2) - pow(y_2, 2), 0);
  }
  Complex fraction = top/bottom;
  double beta = 2*fraction.c_atan().real();
  //convert back to degrees 
  double beta_deg = 180/3.1415*beta;
  return beta_deg;
}
