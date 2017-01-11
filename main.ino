/* ============================================================================
Proyecto/Archivo:                	spinCoater/main.ino
Microcontrolador:                	atmega328p
Frequencia:                      	16MHz
Copyrights:                      	Jhon Jairo Realpe
Fecha última Compilación:        	Ago. 20/16
===============================================================================
Descripción:                     	Este archivo muestra la implementación  e 
									                integración de los módulos de funcionamiento 
									                de un spincoater 
									
Historia:  							          Implementación módulos de medicíón de velocidad 									
									                Implementación módulos control PID Manual
									                Implementación módulos control PID Automatico
									                Implementación módulo EEPROM
									                Implementación módulos Ingreso datos
									                Implementación módulos Muestra datos
									                Integración de todos los módulos Ago. 20/16
=============================================================================*/

// Declaración de librerias
#include <SPI.h>                                // Libreria SPI, para controlar LCD
#include <LiquidCrystal.h>                      // Libreria usada para controlar LCD
#include <Keypad.h>                             // Libreria usada para manipular teclados matriciales
#include <EEPROM.h>                             // Libreria usada para almacenar datos en la EEPROM

#define encodPinB 3                             // Definición del pin3 para medición de velocidad de motorDC
                                                // por encoder 
#define N 32                                    // Número de pulsos del encoder

#define DIRECCIONINT1 0                         // Definicón de posiiciones de memoria en bytes para almacenamiento
#define DIRECCIONINT2 2                         // de información en la EEPROM
#define DIRECCIONINT3 4
#define DIRECCIONINT4 6
#define DIRECCIONINT5 8

int PinPWM = 9;                                 // Definición del pin9 para controlar motorDC con señal PWM

LiquidCrystal lcd(10);                          // Uso de la función LCD modificida, de libreria LiquidCrystal (SPI)
                                                // para controlar por pin10

const int ROWS = 4;                             // Número de Filas Teclado
const int COLS = 3;                             // Número de Columnas Teclado
char keys[ROWS][COLS] = {                       // Se define el caracter retornado cuando se presione una tecla
    { '1', '2', '3' } ,
    { '4', '5', '6' } ,
    { '7', '8', '9' } ,
    { '*', '0', '#' }
};
byte colPins[COLS] = { 15, 16, 17 };            // Se definen Columnas 0 a 2 del teclado matricial
byte rowPins[ROWS] = { 1, 0, 18, 19 };          // Se definen filas 0 a 3 del teclado matricial
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );   // Definición de objeto Keypad con los atributos
                                                                            // Citados anteriormente
                                                                            
volatile unsigned long lastTime;                // Definición variables medición tiempo de muestreo pulsos encoder
volatile unsigned long duration;                // Definición variables medición tiempo pulsos encoder 
volatile unsigned long CapTime, LastCapTime;    // Definición variables medición tiempo pulsos encoder
volatile double dTv;                            // Definición variables medición tiempo de muestreo pulsos encoder
double Vrpm, Vprom;                             // Definición variables medición velocidad encoder

double dTs;                                     // Definición variable periodo de muestreo para control PID y medición 
                                                // de velocidad del motor
                                                
volatile unsigned long lastTimec;               // Definición variables medición tiempo de muestreo control PID
volatile double dTc;                            // Definición variable medición tiempo de muestreo control PID
volatile double lastOutput;                     // Definicón de variables del control PID
double Output;
double kp, ki, kd;                              // Definicón de las constantes kp, ki y kd del control PID
double ITerm, lastInput;

int vi, vf, ri, tf, rf;                         // Definición de variables para almacenar información en EEPROM

/* ============================================================================ */
// Inicio Función setup
void setup(){                                   
    pinMode(encodPinB, INPUT);                  // Definine el PinB como entrada
    digitalWrite(encodPinB, HIGH);              // Activa el PinB como entrada Pull uP
    attachInterrupt(1, rpmfun, CHANGE);         // Define interrupción externa sobre el PinB cuando haya un cambio
                                                // en el flanco de subida y subida y bajada del tren de  pulsos
    dTs = 0.000040;                             // Define el periodo de muestreo a 40us
    
    CapTime = 0;                                // Se inicializan las variables de tiempo muestreo                                                 
    lastTime = 0;                               // del encoder para la medición de velocidad del motor DC 
    LastCapTime = 0;
    dTv = 0;
    
    lastTimec = 0;                              // Se inicializan las variables de tiempo muestreo
    dTc = 0;                                    // del control PID
    kp = 0.0170, ki = 0.17, kd = 0.00;          // Se inicializan las variables del control PID
    ITerm = 0; lastInput = 0;  
    lastOutput = 0;
    Output = 0;
    
    pinMode(PinPWM, OUTPUT);                    // Se Define el PinPWM como salida
    setupPWM16();                               // Se llama a la función setupPWM16 para configurar el timer1 
                                                // en modo PWM  
    analogWrite16(PinPWM, 0);                   // Se inicializa PinPWM a cero
    
    lcd.begin(16, 2);                           // Se define LCD de 16 columnas x 2 filas
    lcd.clear();                                // Limpia pantalla de LCD
    lcd.setCursor(2, 0); lcd.print("**Welcome**");    // Escribe mensaje de bienvendia en LCD
    delay(2000);
}
// Fin Función setup
/* ============================================================================ */

/* ============================================================================ */
// Inicio Función Principal
void loop(){                                    
    int inicio;
    lcd.clear();                                // Limpia pantalla LCD
    lcd.setCursor(0, 0); lcd.print("1:Manual"); // Posiciona Cursor e imprime texto en LCD
    lcd.setCursor(0, 1); lcd.print("2:Automatic:");
    inicio = keypad2int(1, 12, 1);              // Lee digito del teclado matricial
    delay(1000);                                // Se crea un retardo 
    switch (inicio){                            
        case 1: {manual();} break;              // se Llama a la funcion manual
        case 2: {automatico();} break;          // se Llama a la funcion automatico
        default: {}
    }
}
// Fin Función Principal
/* ============================================================================ */

/* ============================================================================ */
// Inicio Modulos de Medición de Velocidad del motor DC

// Mide el tiempo entre flancos de subida y bajada 
// del tren de pulsos generado por el encoder
void rpmfun(){
    LastCapTime = CapTime;
    CapTime = micros();                         // Mide el tiempo en microsegundos
}

void medirvel(){
	  unsigned long now;      
	  now = micros();
    dTv = (double)(now - lastTime) / 1000000;  // Mide el intervalo de tiempo
    if(dTv >= dTs) {                           // se valida el intervalo de tiempo medido con
                                               // el perido de muestreo.
        lastTime = now;
        double Freq;
        detachInterrupt(1);                    // Se deshabilita las interrupciones externas
        duration = CapTime - LastCapTime;      // Se determina el periodo entre el 
                                               // el flanco de subida y bajada del tren pulsos del encoder 
        LastCapTime = CapTime;
        if(duration == 0) { Freq = 0; }        
        else { Freq = 1000000 / (double)duration; } // Se transforma a frecuencia
        Vrpm = Freq * 60 / N;                   // Se hace la conversión a rpm
        attachInterrupt(1, rpmfun, CHANGE);     // Se habilita la interrupción externa
    }
}

// Se hace un filtro de promedio de la velocidad 
// medida con el encoder
void velprom(){
  double Vsum = 0;
  int count = 1;
  for (int i = 0; i < 1000; i++) {
    medirvel();
    if (Vrpm > 0) {
      Vsum += Vrpm;
      count++;
    }
  }
  Vprom = Vsum / count;
}
// Fin módulos de medición de velocidad de motor DC
/* ============================================================================ */

void manual(){
    double adcProm = 0, ref = 0;
    char key;
    while(1){
        adcProm = adcprom();
        if (adcProm >= 0){
            ref = map(adcProm, 0, 1024, 0, 6132);
        }
        velprom();
        CalcOutCtrl(ref);
        analogWrite16(PinPWM, (int)Output);
        mostrarinfo1(ref);
        key = keypad.getKey();
        if(key == '0'){
            analogWrite16(PinPWM, 0);
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("Finished");
            lcd.setCursor(0, 1); lcd.print("Process...");
            delay(2000);
            break;
        }
    }
}

void automatico(){  
    int inicio;
    automatic:
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("1:New");
    lcd.setCursor(0, 1); lcd.print("2:Repeat:");
    lcd.setCursor(6, 0); lcd.print("3:Out");
    inicio = keypad2int(1, 10, 1);
    delay(1000);
    switch(inicio){
        case 1: {rampa2();} break;
        case 2: {RepRamp2();} break;
        case 3: {break;}
        default: {goto automatic;}
    }
}

/********************************Nuevo**********************************/

void rampa2(){
    int inicio;
    initNuevo:
    contModvi:
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Enter 3-digit");
    lcd.setCursor(0, 1); lcd.print("vi=");
    vi = keypad2int(3, 3, 1);
    lcd.setCursor(6, 1); lcd.print("rpm");
    delay(1000);
    if(vi > 500){
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
    vf = keypad2int(4, 3, 1);
    lcd.setCursor(7, 1); lcd.print("rpm");
    delay(1000);
    if(vf > 6000){
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
    ri = keypad2int(4, 3, 1);
    lcd.setCursor(7, 1); lcd.print("rpm/s");
    delay(1000);
    if(ri > 1000){
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Attention!");
        lcd.setCursor(0, 1); lcd.print("ri<=1000rpm/s");
        delay(1000);
        goto contModri;
    }
    contModrf:
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Enter 4-digit");
    lcd.setCursor(0, 1); lcd.print("rf=");
    rf = keypad2int(4, 3, 1);
    lcd.setCursor(7, 1); lcd.print("rpm/s");
    delay(1000);
    if(rf > 1000){
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Attention!");
        lcd.setCursor(0, 1); lcd.print("rf<=1000rpm/s");
        delay(1000);
        goto contModrf;
    }
    contModtf:
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("Enter 3-digit");
    lcd.setCursor(0, 1); lcd.print("tf=");
    tf = keypad2int(3, 3, 1);
    lcd.setCursor(6, 1); lcd.print("s");
    delay(1000);
    if(tf > 300){
        lcd.clear();
        lcd.setCursor(0, 0); lcd.print("Attention!");
        lcd.setCursor(0, 1); lcd.print("tf<=300s");
        delay(1000);
        goto contModtf;
    }
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("vi=");
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
    lcd.setCursor(0, 0); lcd.print("1:Start");
    lcd.setCursor(8, 0); lcd.print("2:Modify");
    lcd.setCursor(0, 1); lcd.print("3:Cancel:");
    inicio = keypad2int(1, 9, 1);
    delay(1000);
    switch(inicio){
        case 1:{IniciarRampa2();} break;
        case 2: {goto initNuevo;} break;
        case 3: {} break;
        default:{goto initNuevo2;}
    } 

    writeIntEeprom(vi, DIRECCIONINT1);
    writeIntEeprom(vf, DIRECCIONINT2);
    writeIntEeprom(ri, DIRECCIONINT3);
    writeIntEeprom(tf, DIRECCIONINT4);
    writeIntEeprom(rf, DIRECCIONINT5);
}

void IniciarRampa2() {
  lcd.clear();
  velrampUp(vi, vf, ri);
  lcd.clear();
  velcte(vf, tf);
  lcd.clear();
  velrampDown(vf, 0, rf);
  lcd.clear();
  analogWrite16(PinPWM, 0);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Finished");
  lcd.setCursor(0, 1); lcd.print("Process...");
  delay(2000);
}

/********************************Repetir*********************************/

void RepRamp2() {
  int inicio;

  vi = readIntEeprom(DIRECCIONINT1);
  vf = readIntEeprom(DIRECCIONINT2);
  ri = readIntEeprom(DIRECCIONINT3);
  tf = readIntEeprom(DIRECCIONINT4);
  rf = readIntEeprom(DIRECCIONINT5);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("vi=");
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
  lcd.setCursor(0, 0); lcd.print("1:Start");
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

/******************Teclado LCD Encoder Motor PWM************************/

void velrampUp(float sp, float vf, float rate) {
  rate = rate / 10;
  velprom();
  CalcOutCtrl(0);
  while (sp < vf) {
    unsigned long prevms = 0;
    if ((unsigned long)(millis() - prevms) >= 100) {
      prevms = millis();
      sp = sp + rate;
    }
    velprom();
    CalcOutCtrl(sp);
    analogWrite16(PinPWM, (int)Output);
    mostrarinfo1(sp);
  }
}

void velcte(float vf, int tf) {
  unsigned long prevms = 0;
  while (tf > 0) {
    velprom();
    CalcOutCtrl(vf);
    analogWrite16(PinPWM, (int)Output);
    mostrarinfo2(tf, Vprom);
    if ((unsigned long)(millis() - prevms) >= 1000) {
      prevms = millis();
      tf -= 1;
    }
  }
}

void velrampDown(float sp, float vf, float rate) {
  rate = rate / 10;
  while (sp > vf) {
    unsigned long prevms = 0;
    if ((unsigned long)(millis() - prevms) >= 100) {
      prevms = millis();
      sp = sp - rate;
    }
    velprom();
    CalcOutCtrl(sp);
    analogWrite16(PinPWM, (int)Output);
    mostrarinfo1(sp);
  }
}

int keypad2int(int n, int c, int f) {
  char Cadkey[n], key;
  int char2int;
  for (int i = 0; i < n; i++) {
    key = keypad.waitForKey();
    Cadkey[i] = key;
    lcd.setCursor(c, f);
    lcd.print(key);
    c++;
  }
  char2int = atoi(Cadkey);
  return (char2int);
}

float adcprom() {
  double adcProm = 0, adcsum = 0, adc0;
  int count = 1;
  for (int i = 0; i < 50; i++) {
    adc0 = analogRead(A0);
    if (adc0 > 0) {
      adcsum += adc0;
      count++;
    }
  }
  adcProm = adcsum / count;
  return adcProm;
}


void setupPWM16() {
  TCCR1A = _BV(COM1A1) | _BV(COM1B1)  /* non-inverting PWM */
           | _BV(WGM11);                   /* mode 14: fast PWM, TOP=ICR1 */
  TCCR1B = _BV(WGM13) | _BV(WGM12)
           | _BV(CS11);                   /* no prescaling */
  ICR1 = 356;                       /* TOP counter value */
}


void analogWrite16(int pin, int val) {
  OCR1A = val;
}

void CalcOutCtrl(float sp) {
  unsigned long nowc;
  double error, dInput;
  nowc = micros();
  dTc = (double)(nowc - lastTimec) / 1000000;
  if (dTc >= dTs) {
    error = sp - Vprom;
    ITerm += dTc * ki * error;
    if (ITerm > 356) ITerm = 356;
    else if (ITerm < 0) ITerm = 0;
    dInput = Vprom - lastInput;
    Output = kp * error + ITerm - kd * dInput / dTc;
    if (Output > 356) {
      Output = 356;
    }
    else if (Output < 0) {
      Output = 0;
    }
    lastInput = Vprom;
    lastTimec = nowc;
  }
}

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

void mostrarinfo1(double ref) {
  lcd.setCursor(0, 0); lcd.print("SetPoint=");
  lcd.setCursor(9, 0); lcd.print(ref);
  lcd.setCursor(13, 0); lcd.print("rpm");
  lcd.setCursor(0, 1); lcd.print("Speed=");
  lcd.setCursor(6, 1); lcd.print(Vprom);
  lcd.setCursor(10, 1); lcd.print("rpm  ");
}

void mostrarinfo2(int tc, double Vprom) {
  lcd.setCursor(0, 0); lcd.print("Speed=");
  lcd.setCursor(6, 0); lcd.print(Vprom);
  lcd.setCursor(10, 0); lcd.print("rpm  ");
  lcd.setCursor(0, 1); lcd.print("Time=");
  lcd.setCursor(5, 1); lcd.print(tc);
  lcd.setCursor(8, 1); lcd.print("s");
}
