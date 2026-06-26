#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// -------------------------------------------------------------------------
// DEVICE CONFIGURATION
// -------------------------------------------------------------------------
// Using the working SH1106 display driver constructor over I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const int MPU_ADDR = 0x68; // Standard I2C address for your MPU-9250/6500 module

// -------------------------------------------------------------------------
// PHYSICS & SIMULATION CONSTANTS 
// -------------------------------------------------------------------------
const float BOUNCINESS = 0.4;    //  Absorbs more impact on hit
const float FRICTION   = 0.92;   //  Dampening/air resistance
const float SENSITIVITY = 0.4;   //  Tilt scaling factor

// Simulation tracking variables
float ballX = 64.0, ballY = 32.0;
float velX = 0.0, velY = 0.0;
int16_t rawX, rawY, rawZ;

void setup() {
  Serial.begin(115200);
  
  // Initialize the I2C bus and the display configuration
  Wire.begin();
  u8g2.begin();

  // Wake up the MPU-6500 by clearing the sleep bit in power management register 0x6B
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  
  Serial.println("System Initialized Successfully.");
}

void loop() {
  // 1. READ RAW SENSOR DATA DIRECTLY FROM THE REGISTER MAP
  Wire.beginTransmission((uint8_t)MPU_ADDR);
  Wire.write(0x3B); // Starting register for Accelerometer data (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  
  // Request 6 sequential bytes (High/Low pairs for X, Y, and Z axes)
  Wire.requestFrom((uint8_t)MPU_ADDR, (size_t)6, true);

  // Reconstruct 16-bit signed integer values from individual high/low bytes
  rawX = (Wire.read() << 8) | Wire.read();
  rawY = (Wire.read() << 8) | Wire.read();
  rawZ = (Wire.read() << 8) | Wire.read();

  // Convert raw values to standard gravity scales (+/-2g full-scale range)
  float accelX = rawX / 16384.0;
  float accelY = rawY / 16384.0;

  // 2. PHYSICS ENGINE PROCESSING (With Inverted Axis Controls)
  velX -= accelY * SENSITIVITY; 
  velY -= accelX * SENSITIVITY;
  
  // Apply velocity damping over time (simulates environment friction)
  velX *= FRICTION;
  velY *= FRICTION;
  
  // Integrate position coordinates
  ballX += velX;
  ballY += velY;

  // 3. COLLISION DETECTION & BOUNDARY CONSTRAINT LOGIC
  if (ballX < 4)   { ballX = 4;   velX = -velX * BOUNCINESS; }
  if (ballX > 124) { ballX = 124; velX = -velX * BOUNCINESS; }
  if (ballY < 4)   { ballY = 4;   velY = -velY * BOUNCINESS; }
  if (ballY > 60)  { ballY = 60;  velY = -velY * BOUNCINESS; }

  // 4. GRAPHICS RENDERING SYSTEM
  u8g2.clearBuffer();
  
  // Render the enclosing perimeter box frame
  u8g2.drawFrame(0, 0, 128, 64);
  
  // Render the physical sphere object at current coordinates
  u8g2.drawDisc((int)ballX, (int)ballY, 3);
  
  // Push the virtual frame buffer out to the physical display controller
  u8g2.sendBuffer();

  // Control loop timing step for predictable physical execution
  delay(15); 
}