// ===============================================================================
// --- Biblioteca Auxiliar ---
#include  "Nextion.h"   //biblioteca Nextion
#include <SoftwareSerial.h>
#include <Servo.h>
SoftwareSerial mySerial(0, 1); // RX, TX

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//Funções
void trigPuls(int trig);  //Função que gera o pulso de trigger
void EncheAgua(float);
void EncheSuco(float);
void BracoRobo(float);
void moveServo(Servo, int);
float Niveis(int, int, int);


//Arduino master

#define optico1 4               //Sensores ópticos, indicam a posição na esteira
#define optico2 2   //int0      //os sensores que precisam de interrupção, devem estar... 
#define optico3 3   //int1      //...nos pinos 2, 3 e 21 do arduino 
#define optico4 21  //int2
#define optico5 5                
#define transistor1 6           //Transistor do motor da esteira
#define transistor2 8           //Transistor da bomba d'água
#define transistor3 7           //Transistor da bomba de suco
#define pinservo1 9     //garra           //Servos que controlam o braço robótico
#define pinservo2 10    //gira
#define pinservo3 11    //cima/baixo/esquerda    
#define pinservo4 12    //frente/trás/direita
#define nivelAguaTrig 14         //Ultrassônico mede nível da água no pote 2 fios
#define nivelAguaEcho 15            
#define nivelSucoTrig 16         //Ultrassônico mede nível do suco no pote 2 fios
#define nivelSucoEcho 17
#define res_aguaTrig 18         //Ultrssônico mede nível de água no reservatório 2 fios
#define res_aguaEcho 19
#define res_sucoTrig 20         //Ultrssônico mede nível de suco no reservatório 2 fios
#define res_sucoEcho 22

//Variáveis
int bar_value = 0;    //valor da barra de progresso
uint32_t ds_var;
uint32_t number1 = 0;
uint32_t number2 = 0;

float Nivel = 0;
float pulse;                //Variável que armazena o tempo de duração do echo
float dist_cm;              //Variável que armazena o valor da distância em cm
const int numAnteriores = 20;
float nivelAnteriores[numAnteriores] = {0.0};
int contadorAcima = 0; // Contador para leituras consecutivas acima do nivel
float NivelRA; 
float bar_RA;
float NivelRS; 
float bar_RS;
float NivelPote;
float bar_Pote;
float vol1 = 0;
float vol2 = 0;

int potpin1 = A0;  // analog pin used to connect the potentiometer
int val1;    // variable to read the value from the analog pin

int potpin2 = A4;  // analog pin used to connect the potentiometer
int val2;    // variable to read the value from the analog pin

int potpin3 = A2;  // analog pin used to connect the potentiometer
int val3;    // variable to read the value from the analog pin

int potpin4 = A3;  // analog pin used to connect the potentiometer
int val4;    // variable to read the value from the analog pin

float Posicao;
int PosInicial1 = 133;
int PosInicial2 = 65;
int PosInicial3 = 60;
int PosInicial4 = 110;
int freq = 0;
char txt_nivel[10];
int Andamento = 0;
// ===============================================================================
// --- Declaração de Objetos ---
//                                    (page id, id componente, nome do componente)
NexProgressBar Pbar   = NexProgressBar(1, 2, "j0");
NexProgressBar Pote   = NexProgressBar(1, 6, "j5");
NexProgressBar Ragua  = NexProgressBar(2, 2, "j1");
NexProgressBar Rsuco  = NexProgressBar(2, 3, "j2");
NexProgressBar NivAgua= NexProgressBar(3, 2, "j3");
NexProgressBar NivSuco= NexProgressBar(3, 3, "j4");
NexDSButton bt0       = NexDSButton(0, 5, "bt0");
NexPage Start         = NexPage(0, 0, "page0");
NexPage Progresso     = NexPage(1, 0, "page1");
NexPage Reservatorios = NexPage(2, 0, "Reservatorios");
NexText txt_Pote      = NexText(1, 5, "t1");

NexTouch *nex_listen_list[] = 
{
  &bt0,
  NULL
};

void setup() {
  Serial.begin(9600);
  nexInit();  //inicializa o tft

  pinMode(optico1,          INPUT);
  pinMode(optico2,          INPUT);
  pinMode(optico3,          INPUT);
  pinMode(optico4,          INPUT);
  pinMode(optico5,          INPUT);
  pinMode(transistor1,     OUTPUT);
  digitalWrite(transistor1,   LOW);
  pinMode(transistor2,     OUTPUT);
  digitalWrite(transistor2,   LOW);
  pinMode(transistor3,     OUTPUT);
  digitalWrite(transistor3,   LOW);
  pinMode(nivelAguaTrig,   OUTPUT);
  digitalWrite(nivelAguaTrig, LOW);
  pinMode(nivelAguaEcho,    INPUT);
  pinMode(nivelSucoTrig,   OUTPUT);
  digitalWrite(nivelAguaTrig, LOW);
  pinMode(nivelSucoEcho,    INPUT);
  pinMode(res_aguaTrig,    OUTPUT);
  digitalWrite(res_aguaTrig,  LOW);
  pinMode(res_aguaEcho,     INPUT);
  pinMode(res_sucoTrig,    OUTPUT);
  digitalWrite(res_sucoTrig,  LOW);
  pinMode(res_sucoEcho,     INPUT);

  // Declarando os pinos para os servos
  servo1.attach(pinservo1);
  servo2.attach(pinservo2);
  servo3.attach(pinservo3);
  servo4.attach(pinservo4);

  // Configuração das posições iniciais dos servos
  servo1.write(PosInicial1);
  servo2.write(PosInicial2);
  servo3.write(PosInicial3);
  servo4.write(PosInicial4);
   
}

void loop() {
  nexLoop(nex_listen_list);
  
  NivAgua.getValue(&number1);     //Lê o valor da barra de progresso do nível de agua desejado
  vol1 = number1/14;              //Volume de agua varia de 0 a 7
  NivSuco.getValue(&number2);       //Lê o valor da barra de progresso do nível de suco desejado
  vol2 = vol1+(number2/25);         //Volume de suco varia do volume de agua + (0 a 4)
  
  NivelRA = Niveis(res_aguaTrig, res_aguaEcho, 12);  //Mede o nível de água do reservatório
  bar_RA = map(NivelRA, 0, 12, 0, 100);              //Manda o nível para a barra de progresso do Nextion

  NivelRS = Niveis(res_sucoTrig, res_sucoEcho, 12);  //Mede o nível de suco do reservatório
  bar_RS = map(NivelRS, 0, 12, 0, 100);              //Manda o nível para a barra de progresso do Nextion

  //Atualização dos valores do Nextion
  Pbar.setValue(Andamento);
  Ragua.setValue(bar_RA);
  Rsuco.setValue(bar_RS);

  //Verifica a condição do botão de INICIAR
  bt0.getValue(&ds_var);
    if(ds_var) 
    {
      if (!digitalRead(optico1)) {
        memset(txt_nivel, 0, sizeof(txt_nivel));
        itoa(0, txt_nivel, 10);
        txt_Pote.setText(txt_nivel);
        Pote.setValue(0);
        Andamento = 0;
        Posicao = 1;
      digitalWrite(transistor1, HIGH);
      }else if (!digitalRead(optico2)) {
        if(Posicao == 1){
          EncheAgua(vol1);
          Andamento = 25;
        }
      }else if (!digitalRead(optico3)) {
        if(Posicao == 2){
          EncheSuco(vol2);
          Andamento = 50;
        }
      }else if (!digitalRead(optico4)) {
        if(Posicao == 3){
        BracoRobo();
        Andamento = 75;
        }
      }else if (!digitalRead(optico5)) {
        Posicao = 5;
        Andamento = 100;
        digitalWrite(transistor1, LOW);
      }
    }
    else
    {
      digitalWrite(transistor1, LOW);
    }
}

void trigPulse(int trig)
{
  digitalWrite(trig, HIGH);  //Pulso de trigger em nível alto
  delayMicroseconds(10);     //duração de 10 micro segundos
  digitalWrite(trig, LOW);   //Pulso de trigger em nível baixo
}

float Niveis(int trig, int echo, int altura){
    trigPulse(trig); // Aciona o trigger do módulo ultrassônico
    pulse = pulseIn(echo, HIGH, 200000); // Mede o tempo em que o pino de echo fica em nível alto
    dist_cm = pulse / 58; // Valor da distância em centímetros
    if(dist_cm >altura){
      dist_cm = altura;
    }
    // Calcula o nível de água com base na distância medida
    Nivel = altura - dist_cm;
    return Nivel;
}

void EncheAgua(float volume){
  Posicao = 2;
  digitalWrite(transistor1, LOW);       //desliga o motor da esteira
  analogWrite(transistor2, 70);        //liga a bomba de agua
  contadorAcima = 0;
  
  while (contadorAcima < 30) {
    // Desloca os valores anteriores no array para a direita
    for (int i = numAnteriores - 1; i > 0; i--) {
      nivelAnteriores[i] = nivelAnteriores[i - 1];
    }
    
    // Calcula o nível de água com base na distância medida
    Nivel = Niveis(nivelAguaTrig, nivelAguaEcho, 15);

    // Armazena o valor atual do nível no início do array
    nivelAnteriores[0] = Nivel;

    // Calcula a média dos últimos 5 valores do nível
    float somaNiveis = 0;
    for (int i = 0; i < numAnteriores; i++) {
      somaNiveis += nivelAnteriores[i];
    }
    Nivel = somaNiveis / numAnteriores;
    // Verifica se o nível atual está acima do valor estabelecido
    if (Nivel > volume) {
      contadorAcima++; // Incrementa o contador
    } else {
      contadorAcima = 0; // Reinicia o contador se o nível estiver abaixo do desejado
    }
    memset(txt_nivel, 0, sizeof(txt_nivel));
    itoa(Nivel, txt_nivel, 10);
    txt_Pote.setText(txt_nivel);
    Pote.setValue(Nivel*9);
  }
  Nivel = 0;
  analogWrite(transistor2, 0);          //desliga a bomba de agua
  delay(1000);                          //tempo de espera para nao cair agua na esteira
  digitalWrite(transistor1, HIGH);      //liga o motor da esteira
}

void EncheSuco(float volume){
  Posicao = 3;
  digitalWrite(transistor1, LOW);       //desliga o motor da esteira
  analogWrite(transistor3, 70);        //liga a bomba de suco
  contadorAcima = 0;
  
  while (contadorAcima < 30) {
    // Desloca os valores anteriores no array para a direita
    for (int i = numAnteriores - 1; i > 0; i--) {
      nivelAnteriores[i] = nivelAnteriores[i - 1];
    }

    // Calcula o nível de água com base na distância medida
    Nivel = Niveis(nivelSucoTrig, nivelSucoEcho, 16);

    // Armazena o valor atual do nível no início do array
    nivelAnteriores[0] = Nivel;

    // Calcula a média dos últimos 20 valores do nível
    float somaNiveis = 0;
    for (int i = 0; i < numAnteriores; i++) {
      somaNiveis += nivelAnteriores[i];
    }
    Nivel = somaNiveis / numAnteriores;
    // Verifica se o nível atual está acima do desejado
    if (Nivel > volume) {
      contadorAcima++; // Incrementa o contador
    } else {
      contadorAcima = 0; // Reinicia o contador se o nível estiver abaixo do estabelecido
    }
    memset(txt_nivel, 0, sizeof(txt_nivel));
    itoa(Nivel, txt_nivel, 10);
    txt_Pote.setText(txt_nivel);
    Pote.setValue(Nivel*9);
  }
  Nivel = 0;
  analogWrite(transistor3, 0);          //desliga a bomba de suco
  delay(1000); 
  digitalWrite(transistor1, HIGH);       //liga o motor da esteira
}

void BracoRobo(){
  Posicao = 4;
  digitalWrite(transistor1, LOW);       //desliga o motor da esteira
  // Mover para a posição de pegar o objeto1
  moveServo(servo1, 90);
  moveServo(servo2, 29);
  moveServo(servo3, 96);
  moveServo(servo4, 142);
  moveServo(servo1, 143);
  delay(1000);

  // Mover para a posição de soltar o objeto1
  moveServo(servo3, 173);
  moveServo(servo4, 75);
  moveServo(servo2, 129);
  moveServo(servo4, 100);
  moveServo(servo1, 120);
  delay(1000);

  // Mover para a posição de pegar o objeto2
  moveServo(servo2,51);
  moveServo(servo1, 90);
  moveServo(servo3, 100);
  moveServo(servo4, 144);
  moveServo(servo1, 143);
  delay(1000);

  // Mover para a posição de soltar o objeto2
  moveServo(servo3, 173);
  moveServo(servo4, 75);
  moveServo(servo2, 129);
  moveServo(servo4, 100);
  moveServo(servo1, 120);
  delay(1000);
  
  // Volta a posição original
  moveServo(servo1, PosInicial1);
  moveServo(servo2, PosInicial2);
  moveServo(servo3, PosInicial3);
  moveServo(servo4, PosInicial4);
  delay(1000);
  digitalWrite(transistor1, HIGH);       //liga o motor da esteira
}

//Função criada para uma movimentação desacelerada do braço robótico
void moveServo(Servo servo, int PosDesejada) {
  
  int servoPos = servo.read();

  while (servoPos != PosDesejada) {
    if (servoPos < PosDesejada) {
      servoPos += 1;
    } else {
      servoPos -= 1;
    }
    servo.write(servoPos);
    delay(15);
  }
}
