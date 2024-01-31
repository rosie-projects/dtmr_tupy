#include "Ultrasonic.h" //INCLUSÃO DA BIBLIOTECA NECESSÁRIA PARA FUNCIONAMENTO DO CÓDIGO
//Constants
#define channelA 5
#define channelB 4
#define channelC 3
//Parameters
const int echoPin = 7; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
const int trigPin = 6; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)

Ultrasonic ultrasonic(trigPin,echoPin); //INICIALIZANDO OS PINOS DO ARDUINO

int distancia; //VARIÁVEL DO TIPO INTEIRO
String result; //VARIÁVEL DO TIPO STRING
String sonar0; //VARIÁVEL DO TIPO STRING
String sonar1; //VARIÁVEL DO TIPO STRING
String sonar2; //VARIÁVEL DO TIPO STRING
String sonar3; //VARIÁVEL DO TIPO STRING
String sonar4; //VARIÁVEL DO TIPO STRING
String sonar5; //VARIÁVEL DO TIPO STRING
String sonar6; //VARIÁVEL DO TIPO STRING
String sonar7; //VARIÁVEL DO TIPO STRING

void setup(){
//Init Serial USB
pinMode(echoPin, INPUT); //DEFINE O PINO COMO ENTRADA (RECEBE)
pinMode(trigPin, OUTPUT); //DEFINE O PINO COMO SAIDA (ENVIA)
Serial.begin(9600);
Serial.println(F("Initialize System"));
//Init CD4051B
  pinMode(channelA, OUTPUT);
  pinMode(channelB, OUTPUT); 
  pinMode(channelC, OUTPUT); 
  digitalWrite(channelA, LOW);
  digitalWrite(channelB, LOW);
  digitalWrite(channelC, LOW);
}

void loop(){
MuxSONAR();
}

void selectChannel(int chnl){/* function selectChannel */ 
//// Select channel of the multiplexer 
  int A = bitRead(chnl,0); //Take first bit from binary value of i channel.
  int B = bitRead(chnl,1); //Take second bit from binary value of i channel.
  int C = bitRead(chnl,2); //Take third bit from value of i channel.
  digitalWrite(channelA, A);
  digitalWrite(channelB, B);
  digitalWrite(channelC, C);
  
 // Serial.print(F("channel "));Serial.print(chnl);Serial.print(F(" : "));
//  Serial.print(C);
//  Serial.print(F(","));
//  Serial.print(B);
//  Serial.print(F(","));
//  Serial.print(A);
//  Serial.println();
//  delay(2000); //INTERVALO DE 500 MILISSEGUNDOS


}

//MÉTODO RESPONSÁVEL POR CALCULAR A DISTÂNCIA
void hcsr04(){
    digitalWrite(trigPin, LOW); //SETA O PINO 6 COM UM PULSO BAIXO "LOW"
    delayMicroseconds(2); //INTERVALO DE 2 MICROSSEGUNDOS
    digitalWrite(trigPin, HIGH); //SETA O PINO 6 COM PULSO ALTO "HIGH"
    delayMicroseconds(10); //INTERVALO DE 10 MICROSSEGUNDOS
    digitalWrite(trigPin, LOW); //SETA O PINO 6 COM PULSO BAIXO "LOW" NOVAMENTE
    //FUNÇÃO RANGING, FAZ A CONVERSÃO DO TEMPO DE
    //RESPOSTA DO ECHO EM CENTIMETROS, E ARMAZENA
    //NA VARIAVEL "distancia"
    distancia = (ultrasonic.Ranging(CM)); //VARIÁVEL GLOBAL RECEBE O VALOR DA DISTÂNCIA MEDIDA
    result = String(distancia); //VARIÁVEL GLOBAL DO TIPO STRING RECEBE A DISTÂNCIA(CONVERTIDO DE INTEIRO PARA STRING)
    delay(50); //INTERVALO DE 500 MILISSEGUNDOS
 }

void MuxSONAR(){/* function MuxSONAR */ 
for(int i = 0; i < 8; i++){
    selectChannel(i);   
    hcsr04(); // FAZ A CHAMADA DO MÉTODO "hcsr04()"
    delay(500); //INTERVALO DE 500 MILISSEGUNDOS

    if (i == 0) {sonar0 = result;}
    if (i == 1) {sonar1 = result;}
    if (i == 2) {sonar2 = result;}
    if (i == 3) {sonar3 = result;}
    if (i == 4) {sonar4 = result;}
    if (i == 5) {sonar5 = result;}
    if (i == 6) {sonar6 = result;}
    if (i == 7) {sonar7 = result;}
   
    if (i == 7) {
        Serial.print("NOVA LEITURA\n"); //IMPRIME O TEXTO NO MONITOR SERIAL  
        
        Serial.print("Distância Sonar 0 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar0); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 1 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar1); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 2 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar2); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 3 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar3); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 4 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar4); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 5 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar5); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 6 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar6); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
        
        Serial.print("Distância Sonar 7 "); //IMPRIME O TEXTO NO MONITOR SERIAL
        Serial.print(sonar7); ////IMPRIME NO MONITOR SERIAL A DISTÂNCIA MEDIDA
        Serial.println("cm"); //IMPRIME O TEXTO NO MONITOR SERIAL
    }
    
    }
}
