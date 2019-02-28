//ESTE PROGRAMA ESTA DISEÑADO PARA LA PLACA DE PRUEBA DE mayo 2017 CON EL SENSOR MB1000 DE MAXBOTICS

#include <avr/sleep.h>

//TONOS PARA LA GENERACIÓN DE SONIDOS
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

//declaracion de tonos para modo proporcional

int tonos[]={31,33,35,37,39,41,44,46,49,52,55,58,62,65,69,73,78,82,87,93,98,104,110,117,123,131,139,147,156,165,175,185,196,208,220,233,247,262,277,294,311,330,349,370,392,
415,440,466,494,523,554,587,622,659,698,740,784,831,880,932,988,1047,1109,1175,1245,1319,1397,1480,1568,1661,1760,1865,1976,2093,2217,2349,2489,2637,2794,2960,3136,3322,3520,
3729,3951,4186,4435,4699,4978};

//declaración de variables de entrada y salida
//bool sinuso19=19;
//bool sinuso18=18;
int sensor=A3;   //lectura de sensor forma análoga
int detector_mot_ext=A2; //detector de conexion de motor
int bateria=A1;  //lectura de variable de batería
int encendido = 14; //on off encendido del circuito general
//bool sinuso13=13;
//bool sinuso12=12;
//bool sinuso11=11;
int boton2=10;      //pulsador izquierdo
int buzzer=9; //pin buzzer frecuencias
//bool sinuso8=8;
//bool sinuso7=7;
//bool sinuso6=6;
int mot = 5;    //pin de motor vibrador pwm 0-255
int led = 4;    //
int mot_ext=3;  //motor externo activacion 
int boton1=2;   //pulsador derecho  INTERRUPCION 0
//bool sinuso1=1;  tx
//bool sinuso0=0;  rx


///////////////////////////////////////////////declaración de variables para uso general
bool lectura1=0;              //variables para lectura de botones
bool lectura2=0;
bool estado_sistema=0;
bool deteccion_accion2=0;     //variable de enclavamiento para accion2 del boton1 
bool deteccion_accion3=0;     //variable de enclavamiento para accion2 del boton0
int t_rebote=0;               //tiempo de retardo para evitar rebote de ambos botones
int t_accion2=0;              //tiempo de pulsacion para cambio de accion1 a accion2
unsigned long ultima_act=0;
unsigned long ultima_act2=0;
unsigned long t_bajada=0;
unsigned long t_bajada2=0;
int motor_pop_alto=0;     //factor de pulso en alto para el motor proporcional
int s_motor_pop_bajo=0;   //sumador de pulso bajo para el motor en modo proporcional
int p_motor_pop_bajo=0;   //factor de producto de pulso en bajo para el motor en modo proporcional
bool b_actual=0;
bool b_anterior=0;
bool b2_actual=0;
bool b2_anterior=0;
int distancia_interior=0;
int distancia_exterior=0;
int pwm_motor=0;
int pwm_motor_p=0;
int distancia=0;
int tono=0;
int nota_duracion=50;
int stop_duracion=50;
int contador=0;
int relacion=0;
int multiplicador_sonido_pop=0;
int multiplicador_sonido_pop_duo=0;
int s_motor_pop_bajo_duo=0;   //sumador de del tiempo en bajo del modo porporcional 200
int p_motor_pop_bajo_duo=0;    //multiplicador del tiempo en bajo del modo porporcional  10
int motor_pop_alto_duo=0;
int distancia_conf=0; //variable para guardar la distancia interior o exterior
int bateriaValue=0;
/////                    //VARIABLES PARA MENU

int modo=0;  //seleccionar modo de funcionamieto 0 modo aviso 1 modo escaner 2 modo ecolocalizacion 
int modo1=0; //seleccionar modo de funcionamieto 0 vibracion 1 sonido 2 dual 
            //configuracion y codigo inicial
bool modo2=0; //seleccionar modo de funcionamieto 0 exterior 1 interior
int sensorValue=0; // variable para almacenar el nivel de bateria

void setup() {
 //declaracion de variables
pinMode(19,OUTPUT);
pinMode(18,OUTPUT);
pinMode(sensor,INPUT);
pinMode(detector_mot_ext,OUTPUT);
pinMode(bateria,INPUT);     //estado de la bateria
pinMode(encendido,OUTPUT);
pinMode(13,OUTPUT);
pinMode(12,OUTPUT);
pinMode(11,OUTPUT);
pinMode(boton2,INPUT); //directo a tierra
pinMode(buzzer,OUTPUT);
pinMode(8,OUTPUT);
pinMode(7,OUTPUT);
pinMode(6,OUTPUT);
pinMode(mot,OUTPUT);
pinMode(led,OUTPUT); 
pinMode(mot_ext,OUTPUT);
pinMode(boton1,INPUT);      //tiene resistencia en la placa
pinMode(1,OUTPUT);
pinMode(0,OUTPUT);

 
//comunicacion Serial
Serial.begin(9600);

//Estado inicial de variables
digitalWrite(19,LOW);
digitalWrite(18,LOW);
//17 SENSOR ENTRADA  //A3
//16 SENSOR MOTOR    //A2
//15 bateria ENTRADA //A1
digitalWrite(encendido,LOW);
digitalWrite(13,LOW);
digitalWrite(12,LOW);
digitalWrite(11,LOW);
//10 boton ENTRADA
digitalWrite(buzzer,LOW);
digitalWrite(8,LOW);
digitalWrite(7,LOW);
digitalWrite(6,LOW);
digitalWrite(mot,LOW);
digitalWrite(led,LOW);
digitalWrite(mot_ext,LOW);
//2 boton ENTRADA
digitalWrite(1,LOW);
digitalWrite(0,LOW);

b_actual=1;
b_anterior=1;
//valores de configuracion de HandEyes
t_accion2=1300;      //en milisegundos tiempo que se requiere para la accion 2 del pulsador

distancia_interior=100;   //distancia de seteo para el modo interior
distancia_exterior=170;  //distancia de seteo para el modo exterior
distancia_conf=90;   //distancia de inicio

pwm_motor=100;       //potencia del motor de 0 a 255
pwm_motor_p=230;    //potencia del motor para modo proporcional
tono=31;            //modificable de 30 a 5000
nota_duracion=50;   //duracion en milisegundos
stop_duracion=50;   //duracion en milisegundos
t_rebote=50;        //duracion en milisegundos es para las pulsaciones
motor_pop_alto=150;     //factor de pulso en alto para el motor proporcional 250
s_motor_pop_bajo=70;   //sumador de del tiempo en bajo del modo porporcional 200
p_motor_pop_bajo=4;    //multiplicador del tiempo en bajo del modo porporcional  10
motor_pop_alto_duo=250;
s_motor_pop_bajo_duo=130;   //sumador de del tiempo en bajo del modo porporcional 200
p_motor_pop_bajo_duo=8;    //multiplicador del tiempo en bajo del modo porporcional  10

modo2=1;            //inicio en 1 interior 0 exterior  
modo1=1;            //inicio de modo1 sonido modo 2 sonido y vibracion modo 0 vibracion
modo=1;             //inicio de modo1 porporcional modo0 aviso
multiplicador_sonido_pop=3;  //relacion de distancia para la pausa del sonido en proporcional  8
multiplicador_sonido_pop_duo=6; //relacion de distancia para la pausa del sonido en proporcional cuando esta en vibracion y sonido 
}

////////////////////
///MAIN PRINCIPAL///
////////////////////

void loop() 
{
  distancia=analogRead(sensor);  
  Serial.write(analogRead(detector_mot_ext));
  boton_principal();
  if(estado_sistema==1)
{ 
  boton_secundario();
  if(lectura1==LOW && lectura2==LOW)
  {lectura_bateria();
  }
  if(modo==0)
  {  digitalWrite(mot_ext,HIGH);
    modo_aviso();
  }
  else
  {  digitalWrite(mot_ext,LOW);
    modo_proporcional();
  }
if(sensorValue<543)
{
 sensorValue = analogRead(bateria);
 if(sensorValue<528) //3.4v
{
  bateria_baja();
}
}
}

}

////////////////////
///NIVEL DE BATERIA///
////////////////////

void lectura_bateria()
{ 
  apagar_motor();
  sensorValue = analogRead(bateria);
  if(sensorValue<543)
  {bateriaValue=1;}
  else if(sensorValue<574)
  {bateriaValue=2;}
  else if(sensorValue<605)
  {bateriaValue=3;}
  else if(sensorValue<636)
  {bateriaValue=4;}
  else if(sensorValue<651) //4.2v
  {bateriaValue=5;}
  else {
  bateriaValue=6;
  //exceso de carga mayor a 4.2//
  }
  for(int y=0;y<bateriaValue;y++)
  {
      buzz(buzzer,NOTE_B0,70);
      delay(50);
      buzz(buzzer,NOTE_B1,70);
        delay(500);
  }

  }

///////////////////////////////
///SECUENCIA DE BATERIA BAJA///
///////////////////////////////


void bateria_baja()
{
 delay(1000);
  melodia_bateria_baja();
encendido_falso();

  }

//////////////////////
///FUNCIONES VARIAS///
//////////////////////

void lectura_botones()
{
if(estado_sistema==HIGH)
{  boton_secundario();
  if(lectura1==LOW && lectura2==LOW)
  {lectura_bateria();
  }
}
   }

void despertar()        // despues que se despierta realiza esta funcion
{
deteccion_accion2=0;
//el codigo se ejecuta antes de regresar al loop de ejecución
//funciones que utilicen timers como Serial.printl no funcionan aqui.
//puede estar vacia o cambiar alguna variable si se requiere
}

void dormir()         // ponemos a dormir al arduino
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // tipo de sleep
 
    sleep_enable();          // habilita el bit del sleep en el registro mcucr
                             // para que el sleep sea posible. just a safety pin
 
    attachInterrupt(0,despertar,FALLING); // usa la interrupcion 0 y ejecuta la funcion despertar 
                                       // cuadno el pin2 detecta un estado en bajo
 
    sleep_mode();            // AQUI SE PONE A DORMIR!!
                             // EL PROGRAMA CONTINUA DESDE AQUI CUANDO SE DESPIERTA
 
    sleep_disable();         //lo primero que hacemos es deshabilitar el modo sleep
    detachInterrupt(0);      // deshabilitamos la interrupcion 
                             // el codigo despertar no se ejecutara
                             // durante el timepo normal del programa
}

////////////////////////////////////////
//RUTINAS Y MELODIAS PARA LAS ACCIONES//
////////////////////////////////////////

void rutina_inicio()
{
 //Serial.println("encendido");
   estado_sistema=1;   //estado de sistema encendido
   digitalWrite(encendido,HIGH);
   digitalWrite(led,HIGH);
   analogWrite(mot,pwm_motor);
   delay(500);
   digitalWrite(mot,LOW);
   melodia_encendido();
   digitalWrite(led,LOW);   
   //inicializar acelerómetro      
   delay(1000);
}

void rutina_apagado()
{ 
  estado_sistema=0;   //estado del sistema apagado
  apagar_motor();
  melodia_apagado();
  digitalWrite(encendido,LOW);
  delay(1000);
  dormir();
}

void encendido_falso()
{
  estado_sistema=0; 
  apagar_motor();
  digitalWrite(encendido,LOW);  
  delay(1000);
  dormir();
}

void rutina_exterior()
{ 
  apagar_motor();
  distancia_conf=distancia_exterior; 
  melodia_exterior();
  delay(400);

}

void rutina_interior()
{ 
  apagar_motor();
  distancia_conf=distancia_interior;
  melodia_interior();
  delay(400);
}

void melodia_apagado()
{
  buzz(buzzer,NOTE_C6 ,100);
  buzz(buzzer,0,100);
  buzz(buzzer,NOTE_AS5,100);
  buzz(buzzer,0,100);
  buzz(buzzer,NOTE_G4,100);
}

void melodia_encendido()
{
  buzz(buzzer,NOTE_C4,100);
  buzz(buzzer,0,100);
  buzz(buzzer,NOTE_A5,100);
  buzz(buzzer,0,100);
  buzz(buzzer,NOTE_F6,100);
}

void melodia_bateria_baja()
{

  buzz(buzzer,NOTE_C7,60);
  delay(80);
  buzz(buzzer,NOTE_CS7,60);
  delay(80);
  buzz(buzzer,NOTE_D7,90);
  delay(110);
  buzz(buzzer,0,160);
  delay(200);
  buzz(buzzer,NOTE_C7,90);
  delay(120);
  buzz(buzzer,NOTE_CS7,90);
  delay(120);
  buzz(buzzer,NOTE_D7,120);
  delay(140);
  buzz(buzzer,0,220);
  delay(270);
  buzz(buzzer,NOTE_C7,110);
  delay(140);
  buzz(buzzer,NOTE_CS7,110);
  delay(140);
  buzz(buzzer,NOTE_D7,140);
  delay(160);
  buzz(buzzer,0,240);
  delay(290);
  buzz(buzzer,NOTE_C7,130);
  delay(180);
  buzz(buzzer,NOTE_CS7,160);
  delay(180);
  buzz(buzzer,NOTE_D7,500);
  
  }

void melodia_interior()
{
  int tempo=0;
  int entrepausas=0;
  entrepausas=1.3;
  tempo=31;

  buzz(buzzer,NOTE_C6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_D6,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,5*tempo);
  delay(5*tempo*entrepausas);

buzz(buzzer,NOTE_C6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_D6,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,5*tempo);
  delay(5*tempo*entrepausas);

  
buzz(buzzer,NOTE_C6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS6,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_D6,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
 buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
}

void melodia_exterior()
{
  int tempo=0;
  int entrepausas=0;
  entrepausas=1.3;
  tempo=31;
  
  buzz(buzzer,NOTE_D4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_C4,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
  buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
  
  buzz(buzzer,NOTE_D4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_C4,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
 buzz(buzzer,0,8*tempo); 
  delay(8*tempo*entrepausas);
  
   buzz(buzzer,NOTE_D4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_CS4,tempo);
  delay(tempo*entrepausas);
  buzz(buzzer,NOTE_C4,2*tempo);
  delay(2*tempo*entrepausas);
  buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
 buzz(buzzer,0,8*tempo);
  delay(8*tempo*entrepausas);
}

//////////////////////
////FIN DE SECCION////
//////////////////////

void tono_distancia()
{
int datos=distancia;
    datos=180-distancia;
    datos=datos/2;
    if(datos<1)
    relacion=0;
    else
    relacion=tonos[datos]; 
}

///////////////////////////////////////////
//SECCION DE DIVSIÓN DE ACCIONES POR MODO//
///////////////////////////////////////////
  
void modo_proporcional()
{
  if (distancia<distancia_conf && distancia != 0 )
  {
    if(modo1==0)
    {
      pop_motor();
    }
    else if(modo1==1)
    { 
      pop_buzzer();
      apagar_motor();
    }
    else if(modo1==2)
    {
      pop_motor();
      pop_buzzer();
    }
  }
  else
  {
    apagar_motor();
  }   

}

void modo_aviso()
{
   
  if (distancia<distancia_conf )
  { 
    if(modo1==0)
    { 
     act_motor();
    }
    else if(modo1==1)
    {
     act_buzzer();
     apagar_motor();
    }
    else if(modo1==2)
    {
     act_buzzer();
     act_motor();
    }
  }
  else
  {
    apagar_motor();
  }
}

//////////////////////
////FIN DE SECCION////
//////////////////////

/////////////////////////////////////////////////////////
//SECCION DE PROGRAMACION DE ACCION PARA CADA BOTON//////
/////////////////////////////////////////////////////////

void accion1_boton0(){
       modo1++;
        if(modo1>2)
        modo1=0;
        
}

void accion2_boton0(){ 
  if (modo2 == HIGH) {
    rutina_exterior();
    modo2=LOW;
  } else 
  {
    rutina_interior();
    modo2=HIGH;
   
  }
//b2_anterior=HIGH;
//b2_actual=HIGH;     
}

void accion1_boton1(){
modo++;
if(modo>1)
modo=0;
}

void accion2_boton1()
{
if(estado_sistema==1)
{
  rutina_apagado();
}
else
{
  rutina_inicio();
  deteccion_accion2=0;
}
b_anterior=HIGH;
b_actual=HIGH;
} 

//////////////////////
////FIN DE SECCION////
//////////////////////

////////////////////////////////
//PROGRAMACION DE BOTONES //////
///////////////////////////////

void boton_principal()
{ 
  lectura1 = digitalRead(boton1);
  if (lectura1 != b_anterior) 
  {
    ultima_act = millis();
  }
  b_anterior=lectura1;
  if ((millis() - ultima_act) > t_rebote) 
  {
    if (lectura1 != b_actual)
   {  b_actual = lectura1;
      if (b_actual == 0) 
      { 
        t_bajada=ultima_act;     
      }
      else
      { 
        if((ultima_act-t_bajada)<t_accion2)
      {  if(estado_sistema==1)
        {        
         deteccion_accion2=0;
         accion1_boton1();
        }
        else
        {encendido_falso();
        }
        sensorValue = analogRead(bateria);
      } 
      }
    }
    else
    { if (b_actual == LOW) 
      {        
         if((millis()-t_bajada)>t_accion2)
        { if(deteccion_accion2==0)
          {
            deteccion_accion2=1;
            accion2_boton1();
          }
        } 
      }
    }
  }
}


void boton_secundario()
{
  lectura2 = digitalRead(boton2);
  if (lectura2 != b2_anterior) 
  {
    ultima_act2 = millis();
  }
  b2_anterior=lectura2;
  if ((millis() - ultima_act2) > t_rebote) 
  {
    if (lectura2 != b2_actual)
    { deteccion_accion3=0;
      b2_actual = lectura2;
      if (b2_actual == LOW) 
      {
      t_bajada2=ultima_act2;  
      }
      else
      {
      if((ultima_act2-t_bajada2)<t_accion2)
       {    
        deteccion_accion3=0;
        accion1_boton0();
        sensorValue = analogRead(bateria);
       }
       }
    } 
    else
    { if (b2_actual == LOW) 
      {        
        if((millis()-t_bajada2)>t_accion2)
        {if(deteccion_accion3==0)
         {
           deteccion_accion3=1;
           accion2_boton0();
           sensorValue = analogRead(bateria);
         }
        } 
      }
    }      
  }
}

//////////////////////
////FIN DE SECCION////
//////////////////////

///////////////////////////////////////
//ACIONES ESPECÍFICAS DE ACTUADORES///
///////////////////////////////////////

void pop_motor()
{
  if(modo1==2)
{
 if(motor_pop_alto_duo>contador)
  {
    contador++;
    analogWrite(mot,pwm_motor_p);
  }
  else if((distancia*p_motor_pop_bajo_duo+s_motor_pop_bajo_duo)>contador)
  {
    contador++;
    analogWrite(mot,0);
  }
  else
    contador=0;
}
else
{
   if(motor_pop_alto>contador)
  {
    contador++;
    analogWrite(mot,pwm_motor_p);
  }
  else if((distancia*p_motor_pop_bajo+s_motor_pop_bajo)>contador)
  {
    contador++;
    analogWrite(mot,0);
  }
  else
    contador=0;
  }
  
}
void pop_buzzer()
{   
if(modo1==2)
{
if(distancia*multiplicador_sonido_pop_duo>contador)
  {
  contador++;
  buzz(buzzer,0,40);
  }
  else
    {
  buzz(buzzer,tono,40);
  contador=0;
  }
}
else
{
   if(distancia*multiplicador_sonido_pop>contador)
  {
  contador++;
  buzz(buzzer,0,40);
  }
  else
    {
  buzz(buzzer,tono,40);
  contador=0;
  }
  }
  
} 
void act_motor()
{
//   analogWrite(mot,pwm_motor);  
digitalWrite(mot,HIGH);
}
void apagar_motor()
{
   digitalWrite(mot,LOW);
} 
void act_buzzer()
{   
  buzz(buzzer,tono,nota_duracion);
  buzz(buzzer,0,stop_duracion);
}
void buzz(int targetPin, long frequency, long length) 
{
  long delayValue = 1000000 / frequency / 2;  // calculate the delay value between transitions
                                              //// 1 second's worth of microseconds, divided by the frequency, then split in half since
                                              //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
                                              //// multiply frequency, which is really cycles per second, by the number of seconds to
                                              //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) {      // for the calculated length of time...
  digitalWrite(targetPin, HIGH);            // write the buzzer pin high to push out the diaphram
  delayMicroseconds(delayValue);            // wait for the calculated delay value
  digitalWrite(targetPin, LOW);             // write the buzzer pin low to pull back the diaphram
  delayMicroseconds(delayValue);            // wait again or the calculated delay value
  }
}


