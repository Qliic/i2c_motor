/*********
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp32-i2c-master-slave-arduino/
  ESP32 I2C Slave example: https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/examples/WireSlave/WireSlave.ino
*********/

#include "Wire.h"

#define I2C_DEV_ADDR 0x10

uint32_t i = 0;
uint32_t i2c_register = 0;
uint32_t data = 0;
uint32_t registres[20];

void onRequest() {
  //Wire.print(i++);
  //Wire.print(" requete");
  Wire.write(registres[i2c_register] );
  //Wire.endTransmission();
  Serial.print("onRequest register: ");
  Serial.print(i2c_register);
  Serial.print(" request: ");
  Serial.print(i++);
  Serial.println();
}

void onReceive(int len) {
  Serial.print("onReceived ");
  Serial.println(len);
  if (len == 1) {
    i2c_register = Wire.read(); // On lit l'octet reçu
    Serial.print("Register: ");
    Serial.print(i2c_register); 
    Serial.print(" data: ");
    Serial.print(registres[i2c_register]); 
    //Wire.print(registres[i2c_register]);
  } else {
    if (len == 2) {
      i2c_register = Wire.read(); // On lit l'octet reçu
      Serial.print("registre: ");
      Serial.print(i2c_register); 
      data = Wire.read(); // On lit la donnee reçu
      Serial.print(" data: ");
      Serial.print(data);
      registres[i2c_register] = data;
    } else {

    while (Wire.available()) {
      //Serial.write(Wire.read());
      byte c = Wire.read(); // On lit l'octet reçu
      
      // 1. Affiche le caractère (ex: 'A')
      Serial.print("Char: ");
      Serial.print((char)c); 
      
      // 2. Affiche la valeur numérique en Décimal (ex: 65)
      Serial.print(" | Dec: ");
      Serial.print(c); 
      
      // 3. Affiche la valeur numérique en Hexadécimal (ex: 0x41)
      Serial.print(" | Hex: 0x");
      Serial.println(c, HEX);
    }
    }
  }

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  //Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);

/*#if CONFIG_IDF_TARGET_ESP32
  char message[64];
  snprintf(message, 64, "%lu Packets.", i++);
  Wire.slaveWrite((uint8_t *)message, strlen(message));
  Serial.print('Printing config %lu', i);
#endif*/
}

void loop() {
  Serial.print("I2C port ");
  Serial.println(I2C_DEV_ADDR);
  delay(1000);
  
  for (int i = 0; i < 20; i++) {
    Serial.print(registres[i]); // Affiche la valeur
    
    // Ajoute une virgule et un espace entre les valeurs, 
    // sauf après la dernière valeur pour que ce soit propre.
    if (i < 19) {
      Serial.print(", ");
    }
  }
  Serial.println(); // Saut de ligne final
}
