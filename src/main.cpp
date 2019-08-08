
#include <U8g2lib.h>
#include "esp_system.h"

#include <Arduino.h>
#include <HardwareSerial.h>
#include <WString.h>

#include <rom/gpio.h>
#include <ESP32Encoder.h>

/*GPIO1*/
/*GPIO2*/ #define _GpioLigaSinalRotacao 2       //Chave ligar sinal de rotação ativa em nivel alto
/*GPIO3*/
/*GPIO4*/ #define _SaidaFase 4                  // Saida sinal fase para placa de conversão
/*GPIO5*/ #define _GPIO_pista2 5                 // define GPIO 5 com saida PWM para pista 2
/*GPIO12*/#define _GpioSensorAcelerador 12      // Potenciometro controle Acelerador
/*GPIO13*/#define _GpioSensorRPM 13             // Potenciometro controle rpm
/*GPIO14*/#define _SaidaRodaFonica 14           //Saida sinal Rotação para placa de conversão
/*GPIO15*/
/*GPIO16*/
/*GPIO17*/
/*GPIO18*/#define _GPIO_pista1 18                //saida PWM para pista 1
/*GPIO19*/#define _GpioSaidaSinalVelocidade 19  //Saida sinal velocidade
/*GPIO21*/#define _GpioEncCLK 21                 //Botão Giro encoder
/*GPIO22*/#define _GpioEncSW 22                 //Botão click encoder
/*GPIO23*/#define _GpioEncDT 23               //Botão Giro encoder
/*GPIO25  UG2Lib*/
/*GPIO26 */#define _GpioSensorVelocidade 26 // Potenciometro controle velocidade
/*GPIO27*/#define _GpioBackLight 27 // Iluminação do display
/*GPIO32 UG2Lib*/
/*GPIO33 UG2Lib*/
/*GPIO34 UG2Lib*/
/*GPIO35*/
/*GPIO36*/
/*GPIO39*/

String Rotacao [][200] {
  /*index 0 - 60-2*/{
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1",
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1",
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1",
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1",
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1",
    "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "0", "0", "F"
  },
  /*index "1" - Hall Continuo*/{
    "1", "0", "0", "1", "0", "0", "1", "0", "F"
  },

  /*index "2" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  },
  /*index "1" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  },
  /*index "1" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  },
  /*index "1" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  },
  /*index "1" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  },
  /*index "1" - PLD 6 Cil*/{
    "1", "1", "0", "F"
  }
};



volatile int _indiceItensMenuECU = 0;


String itensMenuECU[] =
{
  "60 - 2 sem Fase",     /*0*/
  "Hall Continuo",  /*1*/
  "Voltar"      /*8*/
};



// --- Variáveis Globais ---
int _indiceMenu = 0;
String itensMenu[] =
{
  "ECU Select",
  "RPM Controle",
  "Acelerador Setup",
  "Velocidade Setup",
  "Voltar"
};





String itensMenuParametros[5] =
{
  "aaaaaaaa",
  "bbbbbbbb",
  "cccccccc",
  "dddddddd",
  "Voltar"
};
String itensAjuste[3] =
{
  "RPM",
  "ACEL",
  "AMP"
};

unsigned leitura;
unsigned _TamnhoMenu = 0;

long oldPosition  = -999;
int OldRpmSet = 0;
int OldAcelSet = 0;
int OldAmpSet = 30;
int OlditensMenuECUSel = 0;

volatile int RpmSet;
volatile int RpmSetInjetor;
volatile int AcelSet;

int numeroItensAjuste;
int itensMenuECUSel = 1;
int itensMenuParametrosSel = 0;
int coluna;
int linha;
int interrupcao = 0;
int counter = 0; //
int _intervalo;  //
int teste = 0;   //
int pwm2Duty = 50;    //PWM 1 sem uso
int media = 0;   //Define a média das leituras
int _PMSDentes = 60;     //Define a dentes roda fonica
int _PMSFalha = 2;    //Define a dentes roda fonica
//int _SensorPMS = 1; //Define tipo sensor roda fonica 1=Hall 2 = Fonico
int _idMenu = 0;     //Define menu geral
int _idSubMenu = 0;    //Define menu geral
int _idMenuBobina = 0;    //Define menu geral
int _idMenuRelPrin = 0;   //Define menu geral
int _idMenuPMS = 0;;     //Define valor nuPMS
int _idMenuImmo = 0;
int _DisplayTempAgua = 0;
int _DisplayTempAr = 0;
int _DisplayMAP = 0;
//int _SinalRotacao = 20; //Recebe analogRead(_GpioSinalRotacao)
int _SinalVelocidade = analogRead(_GpioSensorVelocidade); //Recebe analogRead(_GpioSensorVelocidade)
int _clockDivisor = 12;

int media1 = 0;
int valor = 0;
int med = 0;


double _MenuTimer = 0;
int _HzSendorRotacao = 30;
double _PVCorrente = 0.0;
double _STCorrente = 0.590;
double _PVTensao = 12.85;

double medir = millis() + 500;
double _lagMenu = millis();

String _txtPMS = "60-2";
String _txtImmoStatus = "Ativo";
String _txtSensor = "Hall";
String _txtRP = "(-)";
String _txtFase = "Nao";
String _ECUModelo = "ME749-C3-1.4-Flex";

char *itensSubMenu[5];
char flag_enc = 1;
char pwm1Status = 0;
char pwm2Status = 0;

boolean    _ajusteRPM = 0;
boolean    _ajusteAcel = 0;
boolean    _ajusteAmp = 0;
boolean    _ajusteMenuECU = 0;

boolean    _flagEncoder = 0; //indica giro do emcoder
boolean    _flagMenu = 0;    //Habilta menu
boolean    _flagSubMenu = 0; //Habilta submenu
boolean    _flagSelect = 0;    //Habilta Botão Select
boolean    _fase = 1;      //Habilta sinal da fas
boolean    _encoder = 0;
volatile boolean    _acionaRpm = 0;

float ECT = 0.00;
float ACT = 0.00;

U8G2_ST7920_128X64_1_SW_SPI u8g2(U8G2_R0, /* Enable=*/25, /* RW=*/32, /* RS=*/33, /* reset=*/  34);

ESP32Encoder encoder;

// --- Protótipo das Funções ---
//TaskHandle_t Task1;
//void coreTaskZero( void * pvParameters );
//void coreTaskOne( void * pvParameters );
//void u8g_prepare();
void CarregaVariaveisMenu();
void startTimer();
void LerEncoder();
void TelaInicial();
void TelaPrincipal();
void TelaMenu();
void TelaMenuECU();
void TelaMenuParametros();
void draw(int tela);
void stopTimer();
void cb_timer();
void StrCpy (int *destino, int *origem);

TaskHandle_t InterfaceTasck;
TaskHandle_t TaskControleCorrente;
TaskHandle_t TaskRodaFonica;
TaskHandle_t TaskSaidaVelocidade;

//void InterfaceTasck( void * pvParameters );
void TaskControleCorrenteCode( void * pvParameters );
void SaidaVelocidade( void * pvParameters );
void RodaFonica( void * pvParameters );
void InterfaceCode( void * pvParameters );
// LED pins



void setup() {
  Serial.begin(115200);

  encoder.setCount(0);// set starting count value
  encoder.attachHalfQuad(_GpioEncCLK, _GpioEncDT); // Define GPIO do encode CLK e DT

  Serial.begin(115200);
  Serial.println("Inicio Sistema");

  u8g2.begin();
  u8g2.enableUTF8Print();

  pinMode(_SaidaFase, OUTPUT);
  pinMode(_SaidaRodaFonica, OUTPUT);
  pinMode(_GPIO_pista1, OUTPUT);
  pinMode(_GPIO_pista2, OUTPUT);
  pinMode(_GpioSaidaSinalVelocidade, OUTPUT);
  pinMode(_GpioBackLight, OUTPUT);

  pinMode(_GpioLigaSinalRotacao, INPUT);
  //pinMode(_GpioSinalRotacao, INPUT);
  pinMode(_GpioEncCLK, INPUT);
  pinMode(_GpioEncDT, INPUT);
  pinMode(_GpioEncSW, INPUT);
  pinMode(_GpioSensorVelocidade, INPUT);

  u8g2.setFontRefHeightExtendedText();
  u8g2.setFontPosTop();

  ledcAttachPin(_GPIO_pista1, 0);//Atribuimos o pino 23 ao canal 0.
  ledcSetup(0, 1000, 8);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.

  ledcAttachPin(_GPIO_pista2, 1);//Atribuimos o pino 22 ao canal 1.
  ledcSetup(1, 1000, 8);//Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.
  digitalWrite(_GpioBackLight, 1);

  xTaskCreatePinnedToCore(
    InterfaceCode,   /* Task function. */
    "InterfaceTasck",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    3,           /* priority of the task */
    &InterfaceTasck,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
    TaskControleCorrenteCode,   /* Task function. */
    "TaskControleCorrente",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &TaskControleCorrente,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(500);

  xTaskCreatePinnedToCore(
    SaidaVelocidade,   /* Task function. */
    "TaskSaidaVelocidade",     /* name of task. */
    2000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &TaskSaidaVelocidade,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */
  delay(500);



  xTaskCreatePinnedToCore(
    RodaFonica,   /* Task function. */
    "TaskRodaFonica",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &TaskRodaFonica,      /* Task handle to keep track of created task */
    0);          /* pin task to core 1 */
  delay(500);



}

void InterfaceCode( void * pvParameters ) {
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  while (true) {
    media = 0;
    TelaInicial();
    delay(2000);

simulando: /*Inicio Área de Simulação*/

    if (digitalRead(_GpioLigaSinalRotacao) == LOW) { /*Sinal da chave que liga sinal de rotação*/

      digitalWrite(_SaidaRodaFonica, 1);
      digitalWrite(_SaidaFase, 1);
      RpmSet = 0;
      OldRpmSet = 0;
    } else {
      RpmSetInjetor = map(analogRead(_GpioSensorRPM), 1, 4095, 3000, 50);
    }

    AcelSet = map(analogRead(_GpioSensorAcelerador), 1, 4095, 0, 100);
    ledcWrite(0, map(AcelSet, 0, 100, 25, 5));
    ledcWrite(1, map(AcelSet, 0, 100, 35, 16));
    _HzSendorRotacao = map(analogRead(_GpioSensorVelocidade), 1, 4095, 60, 300);


    if (!digitalRead(_GpioEncSW)) {/*Sinal da chave encoder acionada*/
      _flagMenu = 1;
      _MenuTimer += 1;
    } else {
      _lagMenu = millis() + 1000;
    }


    if (digitalRead(_GpioEncSW)& _MenuTimer > 1) {/*Sinal da chave encoder livre e variavel menu >1*/
      numeroItensAjuste += 1;
      _MenuTimer = 0;
      _MenuTimer = 1;
      _flagMenu = 0;
      _flagSubMenu = 0;
      _idMenu = 0;
      //_indiceItensMenuECU = 0;
      goto menuPrincipal;
      if (numeroItensAjuste > 2)numeroItensAjuste = 0;
    }

    TelaPrincipal();

    goto simulando;  /* ----------< Termino Área de Simulação >--------------*/

menuPrincipal:       /*------------< Inicio Area de Menu >---------------  */
    _flagMenu = 0;
    TelaMenu();
    while (!digitalRead(_GpioEncSW)) { /*Sinal da chave encoder acionada*/
      _flagMenu = 1;
    }

    if (digitalRead(_GpioEncSW) & _flagMenu == 1 & _idMenu == 4 ) {
      _flagMenu = 0;
      goto simulando;
    }

    if (_idMenu == 0 & _flagMenu == 1) {
      goto subMenu;
    }

    goto menuPrincipal; /**/

subMenu: /* Area de Sub menu */

    _flagMenu = 0;
    TelaMenuECU();

    while (!digitalRead(_GpioEncSW)) { /*Sinal da chave encoder acionada*/
      _flagMenu = 1;
    }
    if (digitalRead(_GpioEncSW) & _flagMenu == 1 ) {
      if (_idMenu < sizeof(itensMenuECU) / sizeof(String) - 1) {
        _flagMenu = 0;
        _indiceItensMenuECU = _idMenu;
        Serial.println("Grava");
        goto menuPrincipal;
      }
      _indiceItensMenuECU = sizeof(itensMenuECU) / sizeof(String) - 1;
      Serial.println("Voltar");
      goto menuPrincipal;
    }

    goto subMenu;
  }
} /* termino tarefa InterfaceCode*/

void SaidaVelocidade( void * pvParameters ) {
  Serial.print("SaidaVelocidade Rodando ");
  Serial.println(xPortGetCoreID());
  while (true) {

    TickType_t tasckDelay = 5 / portTICK_PERIOD_MS;
    vTaskDelay(tasckDelay);
  }
}

void RodaFonica( void * pvParameters ) {
  Serial.print("RodaFonica Rodando ");
  Serial.println(xPortGetCoreID());

  int fase = 0;
  int ciclo = 0;

  float periodo = 0;//millis();
  int alta, baixa;
  char teste;
  int rpm = 0;

  while (true) {

    if (digitalRead(_GpioLigaSinalRotacao)) {

      ets_delay_us(RpmSetInjetor);
      if (Rotacao[_indiceItensMenuECU][fase] == "F") {
        fase = 0;
        RpmSet =  (float)(1 / (ciclo / 100000000.0));
        ciclo = 0;

      } else {
        if (Rotacao[_indiceItensMenuECU][fase] == "1") {
          digitalWrite(_SaidaRodaFonica, 0);
        } else {
          digitalWrite(_SaidaRodaFonica, 1);
        }

        fase += 1;
        ciclo += RpmSetInjetor;
      }
    } else {
      fase = 0;
      RpmSet = 0;
      ciclo = 0;
    }

  }
}


void TaskControleCorrenteCode( void * pvParameters ) {
  Serial.print("TaskControleCorrenteCode ");
  Serial.println(xPortGetCoreID());

  while (true) {

    TickType_t tasckDelay = 5 / portTICK_PERIOD_MS;
    vTaskDelay(tasckDelay);
  }

}

/* ///////////////////////////////////////////////////////////// */
void TelaInicial() {

  u8g2.firstPage();
  do {
    u8g2.setFont(u8g_font_unifont);
    u8g2.drawStr(11, 9, "LH Mecatronica");
    u8g2.drawStr(11, 26, "Simudador ECU ");
    u8g2.drawStr(11, 46, "Linha Leve Ver. 2.0");
    u8g2.drawFrame(0, 0, 128, 64);
    u8g2.drawFrame(2, 2, 124, 60);
  } while (u8g2.nextPage());
}



/* /////////////////////////////////////////////////////////////*/

void TelaPrincipal()  //
{
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g_font_helvB08);
    u8g2.drawStr(0, 0, "RPM: ");

    u8g2.setCursor(34, 0);
    u8g2.print(RpmSet);

    u8g2.drawStr(60, 0, "Acel.:         %");

    u8g2.setCursor(95, 0);
    u8g2.print(AcelSet);

    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawHLine(0, 11, 128);
    u8g2.drawStr(0, 14, "P1> 1780mV P2> 380 mV");

    u8g2.setFont(u8g_font_helvB08);
    u8g2.drawHLine(0, 26, 128);
    u8g2.drawStr(0, 28, "ECU:");
    u8g2.setCursor(26, 28);
    //if (itensMenuECU[_indiceItensMenuECU] == "Voltar") {
    //    u8g2.print("Selecione ECU");
    //} else{
    u8g2.print(itensMenuECU[_indiceItensMenuECU]);
    //}
    u8g2.setFont(u8g2_font_6x10_tr);
    u8g2.drawHLine(0, 40, 128);
    u8g2.drawStr(0, 42, "Sensor Veloc.>>        KHz" );
    u8g2.setCursor(85 , 42);
    u8g2.print(_HzSendorRotacao);

  } while (u8g2.nextPage());

}


/*
   /////////////////////TelaMenuECU///////////////////////////////
*/

void TelaMenu()  //
{
  u8g2.firstPage();
  int _lengthString = sizeof(itensMenu) / sizeof(String);
  int _lengthEncoderMax = _lengthString * 4;
  _ajusteMenuECU = 1;

  if (encoder.getCount() < 0)encoder.setCount(0);
  if (encoder.getCount() >= _lengthEncoderMax)encoder.setCount(_lengthEncoderMax);
  itensMenuECUSel = map(encoder.getCount(), 0, _lengthEncoderMax, 0, _lengthString - 1 );
  int item = 0;
  if (itensMenuECUSel > 3) {
    item = itensMenuECUSel - 3;
  }
  do {
    int x = 0, y = 0;
    u8g2.setFont(u8g_font_helvB08);
    u8g2.setCursor(30, 0);
    u8g2.print("Menu Principal");
    u8g2.drawLine(x, y += 12, 128, y);
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.setDrawColor(1);
    y -= 9;
    for (int i = 0; i < 4; i++) {
      if (i + item == itensMenuECUSel) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(x, y += 12, 125, 11);
        u8g2.setDrawColor(0);
        u8g2.setCursor(x, y);
        u8g2.print(itensMenu[i + item]);
        u8g2.setDrawColor(1);
        _idMenu = i + item;

      } else {
        u8g2.setCursor(x, y += 12);
        u8g2.print(itensMenu[i + item]);
      }
    }
  } while (u8g2.nextPage());
}

void TelaMenuECU()  //
{
  u8g2.firstPage();
  int _lengthString = 8;//sizeof(itensMenuECU) / sizeof(String) - 1;
  int _lengthEncoderMax = _lengthString * 4;
  _ajusteMenuECU = 1;

  if (encoder.getCount() < 0)encoder.setCount(0);
  if (encoder.getCount() >= _lengthEncoderMax)encoder.setCount(_lengthEncoderMax);
  itensMenuECUSel = map(encoder.getCount(), 0, _lengthEncoderMax, 0, _lengthString );
  int item = 0;
  if (itensMenuECUSel > 3) {
    item = itensMenuECUSel - 3;
  }
  do {
    int x = 0, y = 0;
    u8g2.setFont(u8g_font_helvB08);
    u8g2.setCursor(30, 0);
    u8g2.print("Menu Principal");
    u8g2.drawLine(x, y += 12, 128, y);
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.setDrawColor(1);
    y -= 9;
    for (int i = 0; i < 4; i++) {
      if (i + item == itensMenuECUSel) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(x, y += 12, 125, 11);
        u8g2.setDrawColor(0);
        u8g2.setCursor(x, y);
        u8g2.print(itensMenuECU[i + item]);
        u8g2.setDrawColor(1);
        _idMenu = i + item;
        _indiceItensMenuECU = _idMenu;

      } else {
        u8g2.setCursor(x, y += 12);
        u8g2.print(itensMenuECU[i + item]);
      }
    }
  } while (u8g2.nextPage());

}
/*
   /////////////// TelaMenuParametros() ///////////////
*/
void TelaMenuParametros()  //
{
  u8g2.firstPage();


  if (encoder.getCount() < 0)encoder.setCount(0);
  if (encoder.getCount() >= 40)encoder.setCount(40);
  itensMenuParametrosSel = map(encoder.getCount(), 0, 40, 0, 4);

  int item = 0;
  if (itensMenuECUSel > 3) {
    item = itensMenuParametrosSel - 3;
  }
  do {
    int x = 0, y = 0;
    u8g2.setFont(u8g_font_helvB08);
    u8g2.drawStr(30, 0, "Parametros");
    u8g2.drawLine(x, y += 12, 128, y);
    u8g2.setFont(u8g2_font_6x10_mf);
    u8g2.setDrawColor(1);
    y -= 9;
    for (int i = 0; i < 4; i++) {
      if (i + item == itensMenuParametrosSel) {
        u8g2.setDrawColor(1);
        u8g2.drawBox(x, y += 12, 125, 11);
        u8g2.setDrawColor(0);
        u8g2.setCursor(x, y);
        u8g2.print(itensMenuParametros[i + item]);
        u8g2.setDrawColor(1);
        _idMenu = i + item;
      } else {
        u8g2.setCursor(x, y += 12);
        u8g2.print(itensMenuParametros[i + item]);
      }
    }
  } while (u8g2.nextPage());

}


/*
   ///////////////////draw//////////////////////
*/
/*void draw(int tela) //
  {
  u8g_prepare();
  switch (tela) {
    case 1:
      TelaInicial();
      break;
    case 2:
      TelaPrincipal();
      break;
    case 3:
      //TelaMenuPrincipal();
      break;
  }
  }**/


/*
   /////////////////LerEncoder///////////////////
*/
void LerEncoder() {
  _flagEncoder = 1;
  _MenuTimer = millis() + 5000;
  if (_intervalo < millis()) {
    if (digitalRead(_GpioEncCLK) == 0) {
      //Seral.println(14);
      interrupcao += 1;
      _intervalo = millis() + 200;
    }
    if (digitalRead(_GpioEncDT) == 0) {
      //Seral.println(13);
      interrupcao -= 1;
      _intervalo = millis() + 200;
    }
  }
}



/*
   //////////////////////u8g_prepare//////////////////////
*/
/*void u8g_prepare() //
  {
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.setFontRefHeightExtendedText();
  //u8g2.setDesetDefaultForegroundColor();
  u8g2.setFontPosTop();
  }*/

void CarregaVariaveisMenu() {
  switch (_idMenu) {
    case 0:
      switch (_idSubMenu) {
        case 0:
          _PMSDentes = 60;
          _PMSFalha = 2;
          _txtPMS = "60 -2";
          break;
        case 1:
          _PMSDentes = 30;
          _PMSFalha = 2;
          _txtPMS = "30 -2";
          break;
        case 2:
          _PMSDentes = 10;
          _PMSFalha = 2;
          _txtPMS = "10 -2";
          break;
        case 3:
          _PMSDentes = 4;
          _PMSFalha = 0;
          _txtPMS = "Hall";
          break;
      }
      break;
    case 1:
      switch (_idSubMenu) {
        case 0:
          _txtImmoStatus = "Ativo";
          _clockDivisor = 12;
          break;
        case 1:
          _txtImmoStatus = "Inativo";
          _clockDivisor = 1;
          break;
      }

    case 2:
      switch (_idSubMenu) {
        case 0:
          break;
        case 1:
          break;
      }
      break;
    case 3:
      switch (_idSubMenu) {
        case 0:
          _fase = 1;
          _txtFase = "SIM";
          break;
        case 1:
          _fase = 0;
          _txtFase = "NAO";
          break;
      }
      break;
    case 4:
      switch (_idSubMenu) {
        case 0:

          break;
        case 1:

          break;
      }
      break;
  }
}

void StrCpy (int *destino, int *origem)
{
  for (int i = 0; i < 120; ++i) {
    destino[i] = origem[i];
  }
}
void loop() {}
