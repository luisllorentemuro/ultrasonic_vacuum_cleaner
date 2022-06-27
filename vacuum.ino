#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

#define PIN_RED 27 // GIOP27
#define PIN_BLUE 12 // GIOP12
#define PIN_GREEN 14 // GIOP14
#define POTENCIOMETRO 39 // GPIO39;  
#define VENTILADOR 35 // GIOP35
#define ULTRASONIDOS1 16 // GIOP16
#define ULTRASONIDOS2 15      // GIOP15

int contador;
int potValue = 0; // value read from the pot
int outputValuePWM = 0; // value output to the PWM (analog out)


void setup() {

  Serial.begin(115200);
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  //Serial.begin(9600);
  mlx.begin();

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_BLUE, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);

  pinMode(VENTILADOR, OUTPUT);
  digitalWrite(VENTILADOR, LOW);

}

void loop() {

  delay(0.001);
  digitalWrite(ULTRASONIDOS1, LOW);
  digitalWrite(ULTRASONIDOS2, HIGH);
  delay(0.001);
  digitalWrite(ULTRASONIDOS2, LOW);
  digitalWrite(ULTRASONIDOS1, HIGH);

  // read the analog in value:
  potValue = analogRead(POTENCIOMETRO);
  // map it to the range of the analog out:
  outputValuePWM = map(potValue, 0, 4095, 0, 255); 
  //Arduino has an analogRead range from 0 to 1023, and an analogWrite range only from 0 to 255, therefore the data from the potentiometer needs to be converted to fit into the smaller range before using it to dim the LED.
  
  Serial.print("potenciometro = ");
  Serial.print(outputValuePWM);
  Serial.print("\n");

  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = ");
  Serial.print(mlx.readObjectTempC());
  Serial.println("*C");
  Serial.print("Ambient = ");
  Serial.print(mlx.readAmbientTempF());
  Serial.print("*F\tObject = ");
  Serial.print(mlx.readObjectTempF());
  Serial.println("*F");
   
  Serial.println();
  delay(1000);
    
  if (mlx.readObjectTempC() < 40){ 
      // color verde
      digitalWrite(PIN_RED,   0);
      digitalWrite(PIN_GREEN, 255);
      digitalWrite(PIN_BLUE,  0);

      digitalWrite(VENTILADOR, outputValuePWM); 

      delay(2000);
      SerialBT.println("Todo correcto");
          
  }
  else{
      // color rojo
      digitalWrite(PIN_RED,   255);
      digitalWrite(PIN_GREEN, 0);
      digitalWrite(PIN_BLUE,  0);

    
      delay(2000);
      SerialBT.println("Temperatura demasiado alta. Poniendo automáticamente el ventilador al máximo...");
      
      
      while (mlx.readAmbientTempC() > 25){
           digitalWrite(VENTILADOR, HIGH); // Hasta que no bajemos de 30º, ponemos el ventilador a las máximas revoluciones para conseguir disipar el calor
      }  
      delay(2);
  }

}
