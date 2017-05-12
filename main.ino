/* ============================================================================
  Autor:			  Jhon Jairo Realpe
  Proyecto/Archivo:               spinCoater/main.ino
  Microcontrolador:               atmega328p
  Frecuencia:                     16MHz
  Licencia:                       Creative Commons
  Fecha última Compilación:       Ago. 20/16
  ===============================================================================
  Descripción:                    Este archivo muestra la Implementación  e
                                  integración de los módulos de funcionamiento
                                  de un spincoater

  Historia:                       Implementación módulos de medición de velocidad
                                  Implementación módulos control PID Manual
                                  Implementación módulos control PID Automático
                                  Implementación módulo EEPROM
                                  Implementación módulos Ingreso datos
                                  Implementación módulos Muestra datos
                                  Integración de todos los módulos Ago. 20/16
  =============================================================================*/
// Declaración de librerías
#include <SPI.h>	// Liberia SPI para controlar LCD
#include <LiquidCrystal.h>	// Librería usada para controlar LCD
#include <Keypad.h>	// Librería usada para manipular teclados matriciales
#include <EEPROM.h>	// Librería usada para almacenar datos en la EEPROM

#define encoder0PinA  2	// Definición del pin2 y 3 para medición de velocidad de motorDC
#define encoder0PinB  3 // con el encoder

#define DIRECCIONINT1 0 // Definición de posiciones de memoria en bytes para almacenamiento
#define DIRECCIONINT2 2 // de información en la EEPROM
#define DIRECCIONINT3 4
#define DIRECCIONINT4 6
#define DIRECCIONINT5 8
// Uso de la función LCD modificada, de librería LiquidCrystal (SPI)
// para controlar por pin10
LiquidCrystal lcd(10);

int PinPWM = 9;	// Definición del pin9 para controlar motorDC con señal PWM

const int ROWS = 4;	// Número de Filas Teclado
const int COLS = 3; // Número de Columnas Teclado
char keys[ROWS][COLS] = { // keymap define el carácter retornado cuando se presione una tecla
  { '1', '2', '3' } ,
  { '4', '5', '6' } ,
  { '7', '8', '9' } ,
  { '*', '0', '#' }
};
byte colPins[COLS] = { 15, 16, 17 };  // Asignación de pines del microcontrolador para definir
// las columnas 0 a 2 y filas 0 a 3 del teclado matricial
byte rowPins[ROWS] = { 1, 0, 18, 19 };
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );   // Definición de objeto Keypad con los atributos
// Citados anteriormente
double PV;	// Definición de las variable para la medición de velocidad
float sumcount;
volatile long count;	// Definición de la variable para el conteo de pulsos del encoder

// Definición variables medición tiempo de muestreo control PID
unsigned long nowc;
volatile unsigned long lastTimec;

// Definición de variables del control PID
double error, dInput;
volatile double lastOutput;
volatile double dTc;
double ITerm, lastInput;
double Output;
float dTs;
double kp, ki, kd;

// Definición de variables para almacenar información en EEPROM
int vi, vf, ri, tf, rf;						
// Definición de variable de conteo de instrucciones para mostrar información por LCD
int countinfo;										

/* ============================================================================ */
// Inicio Función setup
void setup() {
  pinMode(encoder0PinA, INPUT);				// Define el PinA y PinB como entradas
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinA, HIGH); 	// Se Activa resistencia Pull up en PinA y PinB
  digitalWrite(encoder0PinB, HIGH);

  attachInterrupt(0, doEncoder, CHANGE);  // Se define interrupción externa sobre el PinA y PinB cuando haya un cambio
  attachInterrupt(1, doEncoder, CHANGE);  // en el flanco de subida y bajada del tren de pulsos

  pinMode(PinPWM, OUTPUT);				// Se Define el PinPWM como salida
  setupPWM16();                   // Se llama a la función setupPWM16 para configurar el timer1
  // en modo PWM
  analogWrite16(PinPWM, 0);				// Se inicializa PinPWM a cero

  lastTimec = 0;									// Se inicializan las variables de tiempo muestreo
  ITerm = 0; lastInput = 0;				// del control PID
  kp = 0.03;  ki = 0.26;  kd = 0.00025;  dTs = 0.040;		// Se inicializan las constantes del control PID
  dTc = 0;
  lastOutput = 0;
  Output = 0;

  countinfo = 0;

  lcd.begin(16, 2);								// Se define LCD de 16 columnas x 2 filas
  lcd.clear();										// Limpia pantalla de LCD
  lcd.setCursor(2, 0); lcd.print("**Welcome**");	// Escribe mensaje de bienvenida en LCD
  delay(2000);
}
// Fin Función setup
/* ============================================================================ */

/* ============================================================================ */
// Inicio Función Principal
void loop() {
  int inicio;
  lcd.clear();										// Limpia pantalla LCD
  lcd.setCursor(0, 0); lcd.print("1:Manual");		// Menú de selección de control Manual o automático
  lcd.setCursor(0, 1); lcd.print("2:Automatic:");
  inicio = keypad2int(1, 12, 1);	// Lee dígito del teclado matricial
  delay(1000);
  switch (inicio) {
    case 1: {
        manual();									// se Llama a la función manual
      } break;
    case 2: {
        automatico();							// se Llama a la función automático
      } break;
    default: {}
  }
}
// Fin Función Principal
/* ============================================================================ */

/* ============================================================================ */
// Inicio Módulos de control de velocidad Manual y automático
void manual() {
  double adcProm = 0, ref = 0;		// Se declaran variables para medir señal del puerto ADC
  char key;
  while (1) {
    adcProm = analogRead(A0);			// Se lee la señal de un potenciómetro con el puerto ADC
    if (adcProm >= 0) {
      ref = map(adcProm, 0, 1023, 0, 6000);		// Se escala la señal medida con el ADC
    }																				  // de 0 a 1023 a 0 a 6000 rpm
    CalcOutCtrl(ref);							// Se llama a la función del controlador PID, pasando
																	// como parámetro la variable leída y escalada con el canal ADC
    mostrarinfo1(ref);						// Se muestra la información
    key = keypad.getKey();				// Se verifica si se ha presionado el teclado matricial
    if (key == '0') {							// En el caso de presionar cero. El proceso se termina
      analogWrite16(PinPWM, 0);		// Se envía 0 al pin que genera la señal de control PWM
      lcd.clear();
      lcd.setCursor(0, 0); lcd.print("Finished");
      lcd.setCursor(0, 1); lcd.print("Process...");
      delay(2000);
      break;
    }
  }
}

void automatico() {
  int inicio;
automatic:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("1:New");	// Menú de selección tres opciones nuevo, repetir y salir
  lcd.setCursor(0, 1); lcd.print("2:Repeat:");
  lcd.setCursor(6, 0); lcd.print("3:Out");
  inicio = keypad2int(1, 10, 1);	// Lee dato del teclado y lo convierte a numero entero
  delay(1000);
  switch (inicio) {
    case 1: {
        rampa2();									// Llama a la función rampa2
      } break;
    case 2: {
        RepRamp2();								// LLama a la función repetir rampa2
      } break;
    case 3: {
        break;
      }
    default: {
        goto automatic;
      }
  }
}
// Fin Módulos de control de velocidad Manual y automático
/* ============================================================================ */

/* ============================================================================ */
// Inicio de rampas del control automático de velocidad
void rampa2() {
  int inicio;

initNuevo:

contModvi:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Enter 3-digit");	// Menú que pide almacenar las variables
  lcd.setCursor(0, 1); lcd.print("vi=");						// de programación de la rampa de velocidad
  vi = keypad2int(3, 3, 1);													// Lee del teclado el valor de velocidad Inicial
  lcd.setCursor(6, 1); lcd.print("rpm");
  delay(1000);
  if (vi > 500) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Attention!");
    lcd.setCursor(0, 1); lcd.print("Vel<=500rpm");
    delay(1000);
    goto contModvi;
  }

contModvf:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Enter 4-digit");
  lcd.setCursor(0, 1); lcd.print("vf=");
  vf = keypad2int(4, 3, 1);				// Lee del teclado el valor de velocidad Final
  lcd.setCursor(7, 1); lcd.print("rpm");
  delay(1000);
  if (vf > 6000) {								// Se valida que el valor ingresado sea menor a 6000rpm
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Attention!");
    lcd.setCursor(0, 1); lcd.print("Vel<=6000rpm");
    delay(1000);
    goto contModvf;
  }

contModri:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Enter 4-digit");
  lcd.setCursor(0, 1); lcd.print("ri=");
  ri = keypad2int(4, 3, 1);				// Lee del teclado el valor de rata de velocidad de inicio de la rampa
  lcd.setCursor(7, 1); lcd.print("rpm/s");
  delay(1000);
  if (ri > 1000) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Attention!");
    lcd.setCursor(0, 1); lcd.print("ri<=1000rpm/s");	// Se valida que el valor ingresado sea menor a 1000rpm/s
    delay(1000);
    goto contModri;
  }

contModrf:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Enter 4-digit");	// Lee del teclado el valor de rata de velocidad final de la rampa
  lcd.setCursor(0, 1); lcd.print("rf=");
  rf = keypad2int(4, 3, 1);
  lcd.setCursor(7, 1); lcd.print("rpm/s");
  delay(1000);
  if (rf > 1000) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Attention!");
    lcd.setCursor(0, 1); lcd.print("rf<=1000rpm/s");	// Se valida que el valor ingresado sea menor a 1000rpm/s
    delay(1000);
    goto contModrf;
  }

contModtf:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Enter 3-digit");
  lcd.setCursor(0, 1); lcd.print("tf=");
  tf = keypad2int(3, 3, 1);				// Lee del teclado el valor del tiempo de estabilización de la rampa
  lcd.setCursor(6, 1); lcd.print("s");
  delay(1000);
  if (tf > 300) {									// Se valida que el valor ingresado sea menor a 300s
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Attention!");
    lcd.setCursor(0, 1); lcd.print("tf<=300s");
    delay(1000);
    goto contModtf;
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("vi=");	// Se muestra en la LCD los datos ingresados
  lcd.setCursor(3, 0); lcd.print(vi);
  lcd.setCursor(9, 0); lcd.print("ri=");
  lcd.setCursor(12, 0); lcd.print(ri);
  lcd.setCursor(0, 1); lcd.print("vf=");
  lcd.setCursor(3, 1); lcd.print(vf);
  lcd.setCursor(9, 1); lcd.print("rf=");
  lcd.setCursor(12, 1); lcd.print(rf);
  delay(3000);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("tf=");
  lcd.setCursor(3, 0); lcd.print(tf);
  delay(3000);

initNuevo2:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("1:Start");	// Menú con opciones de iniciar el proceso,
																							// modificar los valores o cancelar el proceso
  lcd.setCursor(8, 0); lcd.print("2:Modify");
  lcd.setCursor(0, 1); lcd.print("3:Cancel:");
  inicio = keypad2int(1, 9, 1);
  delay(1000);
  switch (inicio) {
    case 1: {
        IniciarRampa2();											// Se llama a la función InicarRampa2
      } break;
    case 2: {
        goto initNuevo;
      } break;
    case 3: {} break;
    default: {
        goto initNuevo2;
      }
  }

  writeIntEeprom(vi, DIRECCIONINT1);					// Se almacenan los valores ingresados en la memoria EEPROM
  writeIntEeprom(vf, DIRECCIONINT2);
  writeIntEeprom(ri, DIRECCIONINT3);
  writeIntEeprom(tf, DIRECCIONINT4);
  writeIntEeprom(rf, DIRECCIONINT5);
}

void IniciarRampa2() {
  lcd.clear();
  velrampUp(vi, vf, ri);					// Se llama a la función velrampUp
  lcd.clear();
  velcte(vf, tf);									// Se llama a la función velcte
  lcd.clear();
  velrampDown(vf, 0, rf);					// Se llama a la función velrampDown
  lcd.clear();
  analogWrite16(PinPWM, 0);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Finished");
  lcd.setCursor(0, 1); lcd.print("Process...");
  delay(2000);
}

void velrampUp(float sp, float vf, float rate) {	// Se Define la función VelramUP, que genera rampa de ascenso
  rate = rate / 10;								// Se divide el valor de la rata de cambio entre 10 para hacer cambios suaves
  CalcOutCtrl(0);									// Cada 100ms
  unsigned long prevms = 0;
  while (sp < vf) {
    if ((unsigned long)(millis() - prevms) >= 100) {	// Se hacen Incrementos en la velocidad cada 100ms
      prevms = millis();
      sp = sp + rate;
    }
    CalcOutCtrl(sp);							// Se llama a la función que implementa el control PID, enviando el parámetro setpoint
    mostrarinfo1(sp);							// Se muestra la información en LCD

  }
}

void velcte(float vf, int tf) {	// Se Define la función velcte, donde se estabiliza la rampa
  unsigned long prevms = 0;
  while (tf > 0) {
    CalcOutCtrl(vf);						// Se llama a la función que implementa el control PID
    mostrarinfo2(tf);						// Se muestra la información en LCD
    if ((unsigned long)(millis() - prevms) >= 1000) {	// se genera un temporizador hasta que el tiempo final sea cero
      prevms = millis();
      tf -= 1;
    }
  }
}

void velrampDown(float sp, float vf, float rate) {	// Se Define la función VelramUP, que genera rampa de ascenso
  rate = rate / 10;								// Se divide el valor de la rata de cambio entre 10 para hacer cambios suaves
  unsigned long prevms = 0;				// cada 100 ms
  while (sp > vf) {
    if ((unsigned long)(millis() - prevms) >= 100) {	// Se hacen decrementos en la velocidad cada 100ms
      prevms = millis();
      sp = sp - rate;
    }
    CalcOutCtrl(sp);							// Se llama a la función que implementa el control PID, enviando el parámetro setpoint
    mostrarinfo1(sp);							// Se muestra la información en LCD
  }
}

void RepRamp2() {									// Se implementa la función que permite repetir la ultima rampa
  int inicio;

  vi = readIntEeprom(DIRECCIONINT1);	// Se accede a los valores almacenados en la memoria EEPROM
  vf = readIntEeprom(DIRECCIONINT2);
  ri = readIntEeprom(DIRECCIONINT3);
  tf = readIntEeprom(DIRECCIONINT4);
  rf = readIntEeprom(DIRECCIONINT5);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("vi=");	// Se muestra en la LCD los datos ingresados
  lcd.setCursor(3, 0); lcd.print(vi);
  lcd.setCursor(9, 0); lcd.print("ri=");
  lcd.setCursor(12, 0); lcd.print(ri);
  lcd.setCursor(0, 1); lcd.print("vf=");
  lcd.setCursor(3, 1); lcd.print(vf);
  lcd.setCursor(9, 1); lcd.print("rf=");
  lcd.setCursor(12, 1); lcd.print(rf);
  delay(3000);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("tf=");
  lcd.setCursor(3, 0); lcd.print(tf);
  delay(3000);

initNuevo:
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("1:Start");	// Menú con opciones de iniciar o cancelar el proceso
  lcd.setCursor(0, 1); lcd.print("2:Cancel:");
  inicio = keypad2int(1, 9, 1);
  delay(1000);
  switch (inicio) {
    case 1: {
        IniciarRampa2();
      } break;
    case 2: {} break;
    default: {
        goto initNuevo;
      }
  }
}
// Fin de rampas del control automático de velocidad
/* ============================================================================ */

/* ============================================================================ */
// Inicio de módulos de control PID, medición de Velocidad y configuración de señal PWM
void doEncoder() {								// Se implementa la rutina de interrupción que cuenta los pulsos del encoder
  static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
  static uint8_t enc_val = 0;
  enc_val = enc_val << 2;
  enc_val = enc_val | ((PIND & 0b1100) >> 2);
  count = count + lookup_table[enc_val & 0b1111];
}

void CalcOutCtrl(float sp) {			// Se implementa la función de control PID
	unsigned int N = 64;					// Se define el numero de divisiones del encoder
  nowc = micros();
  dTc = (double)(nowc - lastTimec) / 1000000;	// Se hace la medición del tiempo para
  if (dTc >= dTs) {														// determinar el periodo de muestreo y compararlo con dTs = 0.040s
    static long lastcount = 0;
    detachInterrupt(0);						// Se desactivan las interrupciones
    detachInterrupt(1);						// mientras se hacen los cálculos
    if (count == 0) {
      PV = 0;
    }
    else {
      sumcount = count - lastcount;	// Se hace el conteo de pulsos
      PV = ((-1) * sumcount * (60 * (1 / dTc)) ) / N;	// Se calcula la velocidad
      lastcount = count;
    }
    error = sp - PV;							// Se calcula el error entre el setpoint y el valor medido de velocidad
    ITerm += dTc * ki * error;		// Se determina la componente Integral del Control PID
    if (ITerm > 356) ITerm = 356;	// Se valida el efecto WindUP
    else if (ITerm < 0) ITerm = 0;
    dInput = PV - lastInput;			// Se determina la componente Derivativa del Control PID
    Output = kp * error + ITerm - kd * dInput / dTc;	// Se calcula la señal de control, con las componentes PID
    if (Output > 356) {						// Se limita la señal de control en función de la máxima y mínima salida del la señal PWM
      Output = 356;
    }
    else if (Output < 0) {
      Output = 0;
    }
    analogWrite16(PinPWM, (int)Output);	// Se pasa la señal de control a la función de control de la señal PWM
    lastInput = PV;
    attachInterrupt(0, doEncoder, CHANGE); // Se habilitan las interrupciones externas
    attachInterrupt(1, doEncoder, CHANGE);
    lastTimec = nowc;
  }
}

void analogWrite16(int pin, int val) {	// Se implementa la función que controla el ciclo útil de la señal PWM
  OCR1A = val;													// Para controlar la etapa de potencia
}

// Se genera una señal PWM de 2800Hz
// N = 8
// freqpwm = 2800
// TOP = (16*1e6 / (N*freqpwm))-1
// (TOP/2)=356
void setupPWM16() {
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  // Se configura el timer1 en modo FastPWM (mode 14), non-inverting
           | _BV(WGM11);              // TOP=ICR1
  TCCR1B = _BV(WGM13) | _BV(WGM12)		// Se configura el timer1 con preescaler de 8 y conteo hasta el valor
           | _BV(CS11);               // TOP del registro ICR1
  ICR1 = 356;
}
// Fin de módulos de control PID, medición de Velocidad y configuración de señal PWM
/* ============================================================================ */

/* ============================================================================ */
// Inicio de módulos para ingresar y mostrar información
int keypad2int(int n, int c, int f) {	// Se implementa la función de lectura de datos
  char Cadkey[n], key;								// y conversión a entero a partir de un teclado matricial
  int char2int;
  for (int i = 0; i < n; i++) {
    key = keypad.waitForKey();
    Cadkey[i] = key;
    lcd.setCursor(c, f);
    lcd.print(key);
    c++;
  }
  char2int = atoi(Cadkey);				// Se convierte los datos ingresados a un numero entero
  return (char2int);
}

void mostrarinfo1(double ref) {		// Se implementa primer módulo para mostrar información
  if (countinfo > 2500) {					// Se genera un contador para mostrar la información cada 2500 instrucciones
    // y evitar retrasos o cambios en el periodo de muestreo
    lcd.setCursor(0, 0); lcd.print("SetPoint=");
    lcd.setCursor(9, 0); lcd.print(ref);
    lcd.setCursor(13, 0); lcd.print("rpm");
    lcd.setCursor(0, 1); lcd.print("Speed=");
    lcd.setCursor(6, 1); lcd.print(PV);
    lcd.setCursor(10, 1); lcd.print("rpm  ");
    countinfo = 0;
  } else {
    countinfo++;
  }
}

void mostrarinfo2(int tc) {				// Se implementa segundo módulo para mostrar información
  if (countinfo > 2500) {
    lcd.setCursor(0, 0); lcd.print("Speed=");
    lcd.setCursor(6, 0); lcd.print(PV);
    lcd.setCursor(10, 0); lcd.print("rpm  ");
    lcd.setCursor(0, 1); lcd.print("Time=");
    lcd.setCursor(5, 1); lcd.print(tc);
    lcd.setCursor(7, 1); lcd.print("s");
    countinfo = 0;
  } else {
    countinfo++;
  }
}
// Fin de módulos para ingresar y mostrar información
/* ============================================================================ */

/* ============================================================================ */
// Inicio de módulos de lectura y escritura de información en la EEPROM
void writeIntEeprom(int valori, int direccion) {
  byte lowByte = ((valori >> 0) & 0xFF);
  byte highByte = ((valori >> 8) & 0xFF);
  EEPROM.write(direccion, lowByte);
  EEPROM.write(direccion + 1, highByte);
}

int readIntEeprom(int direccion) {
  byte lowByte = EEPROM.read(direccion);
  byte highByte = EEPROM.read(direccion + 1);
  return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}
// Fin de módulos de lectura y escritura de información en la EEPROM
/* ============================================================================ */
