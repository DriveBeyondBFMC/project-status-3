
#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

const uint8_t PIN_RST = 9; // reset pin
const uint8_t PIN_IRQ = 2; // irq pin
const uint8_t PIN_SS = SS;   // spi select pin

char tag_addr[] = "7D:00:22:EA:82:60:3B:9C";
#define MA_WINDOW 5
#define N_ANCHORS 3
#define ANCHOR_DISTANCE_EXPIRED 5000   //measurements older than this are ignore (milliseconds)

float kalman_x = 0, kalman_y = 0;
float p_x = 1, p_y = 1;  
float q = 0.01;  
float r = 0.1;   

float ma_x[MA_WINDOW] = {0}, ma_y[MA_WINDOW] = {0};
int ma_index = 0;


float anchor_matrix[N_ANCHORS][3] = { //list of anchor coordinates, relative to chosen origin.
  {0.00, 5, 0.30}, //Anchor labeled #1
  {3.9, 2.5, 0.30},  //Anchor labeled #2
  {0.00, 0.0, 0.30}, //Anchor labeled #3
};  //Z values are ignored in this code

uint32_t last_anchor_update[N_ANCHORS] = {0}; //millis() value last time anchor was seen
float last_anchor_distance[N_ANCHORS] = {0.0}; //most recent distance reports

float current_tag_position[2] = {0.0, 0.0}; //global current position (meters with respect to anchor origin)
float current_distance_rmse = 0.0;  //rms error in distance calc => crude measure of position error (meters).  Needs to be better characterized


float kalmanUpdate(float &x, float &p, float measurement) {
    p += q;  
    float k = p / (p + r);  
    x += k * (measurement - x);
    p *= (1 - k);
    return x;
}

float movingAverage(float *buffer, int size, float new_value) {
    buffer[ma_index] = new_value;  
    ma_index = (ma_index + 1) % size;  

    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  //initialize configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin

  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);

  // start as tag, do not assign random short address

  DW1000Ranging.startAsTag(tag_addr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);
}

void loop()
{
  DW1000Ranging.loop();
}


void newRange()
{
  int i; //indices, expecting values 1 to 4
  //index of this anchor, expecting values 1,2,3
  int index = DW1000Ranging.getDistantDevice()->getShortAddress() & 0x03; 
  
  if (index > 0) {
    last_anchor_update[index - 1] = millis();  //decrement for array index
    float range = DW1000Ranging.getDistantDevice()->getRange();
    last_anchor_distance[index-1] = range;
    if (range < 0.0 || range > 30.0)     last_anchor_update[index - 1] = 0;  //sanity check, ignore this measurement
  }
  
  int detected = 0;

  for (i = 0; i < N_ANCHORS; i++) {
    if (millis() - last_anchor_update[i] > ANCHOR_DISTANCE_EXPIRED) last_anchor_update[i] = 0; //not from this one
    if (last_anchor_update[i] > 0) detected++;
  }

    if (detected == 3) { 
      
        trilat2D_3A();

        float filtered_x = kalmanUpdate(kalman_x, p_x, current_tag_position[0]*100);
        float filtered_y = kalmanUpdate(kalman_y, p_y, current_tag_position[1]*100);

        // Áp dụng bộ lọc trung bình động
        float smooth_x = movingAverage(ma_x, MA_WINDOW, filtered_x);
        float smooth_y = movingAverage(ma_y, MA_WINDOW, filtered_y);

        Serial.print("P= ");
        Serial.print(current_tag_position[0]*100);
        Serial.write(',');
        Serial.print(current_tag_position[1]*100);
        Serial.write(',');
        Serial.println(current_distance_rmse);
    }
}  

void newDevice(DW1000Device *device)
{
  Serial.print("Device added: ");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}

int trilat2D_3A(void) {
  static bool first = true;  //first time through, some preliminary work
  float b[N_ANCHORS], d[N_ANCHORS]; //temp vector, distances from anchors

  static float Ainv[2][2], k[N_ANCHORS]; //these are calculated only once

  int i;
  // copy distances to local storage
  for (i = 0; i < N_ANCHORS; i++) d[i] = last_anchor_distance[i];

#ifdef DEBUG_TRILAT
  char line[60];
  snprintf(line, sizeof line, "d: %6.2f %6.2f %6.2f", d[0], d[1], d[2]);
  Serial.println(line);
#endif

  if (first) {  //intermediate fixed vectors
    first = false;

    float x[N_ANCHORS], y[N_ANCHORS]; //intermediate vectors
    float A[2][2];  //the A matrix for system of equations to solve

    for (i = 0; i < N_ANCHORS; i++) {
      x[i] = anchor_matrix[i][0];
      y[i] = anchor_matrix[i][1];
      k[i] = x[i] * x[i] + y[i] * y[i];
    }

    // set up least squares equation

    for (i = 1; i < N_ANCHORS; i++) {
      A[i - 1][0] = x[i] - x[0];
      A[i - 1][1] = y[i] - y[0];
#ifdef DEBUG_TRILAT
      snprintf(line, sizeof line, "A  %5.2f %5.2f \n", A[i - 1][0], A[i - 1][1]);
      Serial.println(line);
#endif
    }
    //invert A
    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabs(det) < 1.0E-4) {
      Serial.println("***Singular matrix, check anchor coordinates***");
      while (1) delay(1); //hang
    }

#ifdef DEBUG_TRILAT
    snprintf(line, sizeof line, "det A %8.3e\n", det);
    Serial.println(line);
#endif

    det = 1.0 / det;
    //scale adjoint
    Ainv[0][0] =  det * A[1][1];
    Ainv[0][1] = -det * A[0][1];
    Ainv[1][0] = -det * A[1][0];
    Ainv[1][1] =  det * A[0][0];
  } //end if (first);

  for (i = 1; i < N_ANCHORS; i++) {
    b[i - 1] = d[0] * d[0] - d[i] * d[i] + k[i] - k[0];
  }

  current_tag_position[0] = 0.5 * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
  current_tag_position[1] = 0.5 * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

  float rmse = 0.0, dc0 = 0.0, dc1 = 0.0;
  for (i = 0; i < N_ANCHORS; i++) {
    dc0 = current_tag_position[0] - anchor_matrix[i][0];
    dc1 = current_tag_position[1] - anchor_matrix[i][1];
    dc0 = d[i] - sqrt(dc0 * dc0 + dc1 * dc1);
    rmse += dc0 * dc0;
  }
  current_distance_rmse = sqrt(rmse / ((float)N_ANCHORS));

  return 1;
}  //end trilat2D_3A
