/* Estacion O1A */
/*************************************** Librerías de los sensores *************************************************************************************************************/

/*libreria UV*/
#include <dummy.h> // Librería
#include <Adafruit_Sensor.h> // Librería

/* Librería SGP30 */ 
#include <Wire.h> // Librería
#include "Adafruit_SGP30.h" // Librería
Adafruit_SGP30 sgp;


/* Librería BME280 */ 
// #include <Wire.h>
#include <SPI.h> // Librería
#include <Adafruit_Sensor.h> // Librería
#include <Adafruit_BME280.h> // Librería
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
unsigned long delayTime;



/* Sensor GP2Y10 */ 
// Definición de pines.
const int sharpLEDPin = 25;   // Pin 25 connectado al LED-GND(2) del sensor.
const int sharpVoPin = 33;   // Pin 33 connectado al Vo(5) del sensor.

// Para promediar las últimas N lecturas de voltaje bruto.
static unsigned long VoRawTotal = 0;
static int VoRawCount = 0;

// Configure el voltaje de salida típico en voltios cuando no haya polvo.
static float Voc = 0.6;

// Use la sensibilidad típica en unidades de V por 100ug/m3.
const float K = 0.5;

// Funciones auxiliares para imprimir un valor de datos en el monitor serie.

void printValue(String text, unsigned int value, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  if (!isLast) {
    Serial.print(", ");
  }
}

void printFValue(String text, float value, String units, bool isLast = false) {
  Serial.print(text);
  Serial.print("=");
  Serial.print(value);
  Serial.print(units);
  if (!isLast) {
    Serial.print(", ");
  }
}

/***************************************************************************** Config. comunicación ******************************************************************************/

#include "config.h" // Libreria

/***************************************************************** Envió de variables de los sensores a Adafruit *****************************************************************/

/* Variables sensor SGP30 */ 
AdafruitIO_Feed *CO2_O1A = io.feed("e-o1a.co2-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
AdafruitIO_Feed *VOC_O1A = io.feed("e-o1a.voc-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN

/* Variables sensor BME280 */ 
AdafruitIO_Feed *Tem_O1A = io.feed("e-o1a.tem-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
AdafruitIO_Feed *Pre_O1A = io.feed("e-o1a.pre-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
AdafruitIO_Feed *Hum_O1A = io.feed("e-o1a.hum-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN

/* Variables sensor GP2Y10 */ 
AdafruitIO_Feed *DEN_O1A = io.feed("e-o1a.den-o1a"); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN

/* Variables sensor GYML8511 */ 
AdafruitIO_Feed *UV_O1A = io.feed("e-o1a.uv-o1a");



// Variables del sensor UV GY-ML8511
int UVOUT = 35;  // Conectar al pin analógico A35 de la 

/**Libreria LoRa**/


/**Variables globales para punteros**/
short unsigned int CO2_ex = 0; short unsigned int VOC_ex = 0;
float Temperatura_ex=0; float Presion_ex = 0; float Humedad_ex=0;
float dustDensity_ex=0;

/* Inicia el void setup */
void setup() {
  
  Serial.begin(9600);
  while (!Serial) { delay(10); } // Espera a que se abra la consola serie
  
  /* Funciones setup de los sensores */
  SGP30_CO2_setup ();   // Función setup del sensor SGP30
  BME280_setup();       // Función setup del sensor BME280
  GP2Y10_setup();       // Función setup del sensor GP2Y10
  GYML8511_setup(); // Función setup del sensor GYML8511

  /* Comunicación con Adafruit por medio de WiFi */
  Serial.print("Conectando a Adafruit IO");
  io.connect();
  while(io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());
  
}

/* Inicia el void loop */
int counter = 0; // Contador loop SGP30_CO2


void loop() {

  io.run();

  /*Funciones loop de los sensores*/
  SGP30_CO2_loop (&CO2_ex, &VOC_ex); // Función loop del sensor SGP30
  BME280_loop(&Temperatura_ex, &Presion_ex, &Humedad_ex);     // Función loop del sensor BME280
  GP2Y10_loop(&dustDensity_ex);      // Función loop del sensor GP2Y10
  GYML8511_loop();                   // Función loop del sensor GP2Y10

}

/********************************************************************* Funciones setup y loop de los sensores ********************************************************************/

void SGP30_CO2_setup ()
{
  Serial.println("SGP30 test");

  if (! sgp.begin(&Wire,0x58)){
    Serial.println("Sensor no encontrado :(");
    while (1);
  }
  Serial.print("Encontrado serie SGP30 #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);

  // Si usted tiene una línea de base de medición desde antes de que usted puede asignar al inicio, a 'auto-calibrar'
  // sgp.setIAQBaseline(0x8E68, 0x8F41); // ¡Variará para cada sensor!
}

void SGP30_CO2_loop (short unsigned int *CO2_ex, short unsigned int *VOC_ex)
{
  // Si tiene un sensor de temperatura / humedad, puede establecer la humedad absoluta para habilitar la compensación de humedad para las señales de calidad del aire
  // float temperature = 22.1; // [°C]
  // float humidity = 45.2; // [%RH]
  // sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));

  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  
  Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t"); // compuestos organicos volatiles totales
  Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm"); // C02
 
  delay(2000);

  counter++;
  if (counter == 30) {
    counter = 0;

    uint16_t TVOC_base, eCO2_base;
    if (! sgp.getIAQBaseline(&eCO2_base, &TVOC_base)) {
      Serial.println("Failed to get baseline readings");
      return;
    }
    Serial.print("****Baseline values: eCO2: 0x"); Serial.print(eCO2_base, HEX);
    Serial.print(" & TVOC: 0x"); Serial.println(TVOC_base, HEX);
  }
  delay(2000);      // 2000  //  para no saturar la toma de datos se deja en 3-5 minutos
  
  CO2_ex = &sgp.eCO2;
  VOC_ex = &sgp.TVOC;

  short unsigned int CO2 = *CO2_ex;
  short unsigned int VOC = *VOC_ex;
  
  CO2_O1A->save(CO2); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
  VOC_O1A->save(VOC); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
}



void BME280_setup()
{ 
  Serial.println(F("BME280 test"));

    unsigned status;
    
    // Configuración por defecto
    status = bme.begin(0x76);  
    // También puedes pasar un objeto de la librería Wire como &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("No se ha podido encontrar un sensor BME280 válido, compruebe el cableado, la dirección y el ID del sensor.");
        Serial.print("SensorID era: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("ID de 0xFF probablemente significa una mala dirección, un BMP 180 o BMP 085\n");
        Serial.print("ID de 0x56-0x58 representa un BMP 280,\n");
        Serial.print("ID de 0x60 representa un BME 280.\n");
        Serial.print("ID de 0x61 representa un BME 680.\n");
        while (1) delay(10);
    }
    
    Serial.println("-- Prueba por defecto --");
    delayTime = 300000; // 2000 /7 se plasma para  tres  minutos 

    Serial.println();
}

void BME280_loop(float *Temperatura_ex, float *Presion_ex, float *Humedad_ex )
{
  Serial.print("Temperatura = ");
  Serial.print(bme.readTemperature());
  Serial.println(" °C");

  Serial.print("Presión = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humedad = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();

  delay(delayTime);
  float a = bme.readPressure() / 100.0F;
  float b = bme.readTemperature();
  float c = bme.readHumidity();
  
  Temperatura_ex = &b;
  Presion_ex = &a;
  Humedad_ex = &c;
  
  float Temperatura = *Temperatura_ex;
  float Presion = *Presion_ex;
  float Humedad = *Humedad_ex;  
  
  Tem_O1A->save(Temperatura); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
  Pre_O1A->save(Presion);     // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
  Hum_O1A->save(Humedad);     // CAMBIAR EL GRUPO PARA CADA ESTACIÓN
}

void GP2Y10_setup()
{
  // Establecer el pin LED como salida.
  pinMode(sharpLEDPin, OUTPUT);

  // Espere dos segundos para el inicio.
  delay(2000);
  Serial.println("");
  Serial.println("GP2Y1014AU0F programa");
  Serial.println("=================");
}

void GP2Y10_loop(float *dustDensity_ex)
{
  int N = 100;
    VoRawCount = 0;
    VoRawTotal = 0;
  while ( VoRawCount < N){ // mientras que no complete las muestras
    // Encienda el LED del sensor de polvo configurando el pin digital como LOW.
    digitalWrite(sharpLEDPin, LOW);

    // Espere 0,28 ms antes de tomar una lectura del voltaje de salida según las especificaciones.
    delayMicroseconds(180000); // 280 //  se pasa a tres minutos 

    // Registre el voltaje de salida. Esta operación tarda alrededor de 100 microsegundos.
    int VoRaw = analogRead(sharpVoPin);
    
    // Apague el LED del sensor de polvo configurando el pin digital HIGH.  
    digitalWrite(sharpLEDPin, HIGH);

    // Espere el resto del ciclo de 10 ms => 10000 - 280 - 100 = 9620 microsegundos.
    delayMicroseconds(9620);
    // Imprime el valor de voltaje sin procesar (número de 0 a 1023), mediante la función Print Value
    //#ifdef PRINT_RAW_DATA
    //printValue("VoRaw", VoRaw, true);
    //Serial.println("");
    //#endif
    
    VoRawTotal += VoRaw;
    VoRawCount++;

  }

  float Vo = 1.0 * VoRawTotal / N;  // Calcule el voltaje de salida en voltios.
  Vo = Vo / 1024.0 * 5.0;
  
  printFValue("Vo", Vo*1000.0, "mV");

  // Convertir a Densidad de Polvo en unidades de ug/m3.
  float dV = Vo - Voc;
  if ( dV < 0 ) {
    dV = 0;
    Voc = Vo;
  }
  // imprime la densidad de polvo mediante la función printFvalue
  float b = dV / K * 100.0;
  dustDensity_ex = &b;
  float dustDensity = *dustDensity_ex;
  printFValue("DustDensity", dustDensity, "ug/m3", true);
  Serial.println("");
  
  delay(2000);// 2000 //  se pone a tres minutos 
  
  
  DEN_O1A->save(dustDensity); // CAMBIAR EL GRUPO PARA CADA ESTACIÓN

}

//*********************** GYML8511 ******************************************//
void GYML8511_setup(){
}

void GYML8511_loop() {
  LeerSensorUV();
  delay(2000); // 5000//   Espera 5 segundos antes de realizar la siguiente lectura
}

void LeerSensorUV() {
  int uvLevel = averageAnalogRead(UVOUT);

  // Conversión del nivel analógico a intensidad UV en mW/cm^2
  float uvIntensity = mapfloat(uvLevel, 1030, 4095, 0.0, 15.0);

  Serial.print("Lectura - Nivel UV: ");
  Serial.print(uvLevel);
  Serial.print(" - Intensidad UV (mW/cm^2): ");
  Serial.println(uvIntensity);

  // Envío del valor a Adafruit IO solo si no es NaN
  if (!isnan(uvIntensity)) {
    UV_O1A->save(uvIntensity);
  }
}

int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++) {
    runningValue += analogRead(pinToRead);
  }

  runningValue /= numberOfReadings;
  return runningValue;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
