#include <Thermistor.h> //INCLUSÃO DA BIBLIOTECA
#include <Wire.h> //INCLUSÃO DE BIBLIOTECA
#include <LiquidCrystal_I2C.h> //INCLUSÃO DE BIBLIOTECA

#define AnalogLDR A0
#define BrilhoMax 1024
#define BrilhoMin 500
#define AnalogUmidade A1
#define pinoRele 8 // Define o pino 8 como "pinoRele"
#define pino_led_vermelho 5
#define pino_led_amarelo 6
#define pino_led_verde 7

#define  botao  (1<<3)
boolean  bt1_f = 0x00; //variavel para tratar debounce

int Brilho = 0;
int Leitura = 0;
int LeituraAnterior = 0;
int umidade;
//Variáveis padronizadas pelos modos do sistema:
int tempo_vazao=1;
int intervaloRega;
int umidade_limite; 

Thermistor temp(2);

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3, POSITIVE); //ENDEREÇO DO I2C E DEMAIS INFORMAÇÕES

//Armazena o valor (tempo) da ultima vez que foi mostrado no lcd
unsigned long previousMillisTemperatura = 0;
unsigned long previousMillisLuminosidade = 0;
unsigned long previousMillisUmidade = 0;
unsigned long previousMillisRega = 0;
//Intervalo de tempo entre acionamentos do lcd (em milisegundos)
const long intervaloUmidade = 6000;
const long intervaloTemperatura = 4000;
const long intervaloLuminosidade = 2000;


uint32_t timer;

void setup()  
{  
  lcd.begin (16,2); //SETA A QUANTIDADE DE COLUNAS(16) E O NÚMERO DE LINHAS(2) DO DISPLAY
  lcd.setBacklight(HIGH); //LIGA O BACKLIGHT (LUZ DE FUNDO)
  pinMode(AnalogUmidade, INPUT);
  pinMode(pinoRele, OUTPUT); // Declara o pinoRele como Saída
  pinMode(pino_led_vermelho, OUTPUT);
  pinMode(pino_led_amarelo, OUTPUT);
  pinMode(pino_led_verde, OUTPUT);
  DDRD &= ~botao;  // configura pino 3 como entrada - botão
  PORTD |= botao; // habilita o pull up interno
}  

void loop()  
{  
 // ------------------------------ Tratamento do botão e modos ---------------------------------------------------------//
    if(!(PIND&botao)) bt1_f = 0x01;
      if((PIND&botao) && bt1_f && tempo_vazao < 3) //se continuar em nível alto, é porque o botão foi pressionado de fato 
      {
         bt1_f = 0x00;
         tempo_vazao++;
         lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
         lcd.print("Nível Selecionado:   "); //IMPRIME O TEXTO NO DISPLAY LCD
         lcd.setCursor(0,1); //SETA A POSIÇÃO DO CURSOR
         lcd.print(tempo_vazao);
         lcd.print("          ");
      } 
      if((PIND&botao) && bt1_f && tempo_vazao >=3) 
      {
        bt1_f = 0x00;
        tempo_vazao = 1;
        lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
        lcd.print("Nível Selecionado: "); //IMPRIME O TEXTO NO DISPLAY LCD
        lcd.setCursor(0,1); //SETA A POSIÇÃO DO CURSOR
        lcd.print(tempo_vazao);
        lcd.print("          ");
      }
 //Alternando o tempo de vazão baseado no modo selecionado por botao
  if(tempo_vazao==1) //planta úmida
  {
    intervaloRega  = 3500;
    umidade_limite = 500; 
  }
  else if(tempo_vazao==2) // planta normal 
  {
    intervaloRega  = 2500;
    umidade_limite = 700; 
  }
  else if (tempo_vazao==3){ // planta seca (suculenta por exemplo)
    intervaloRega  = 1500;
    umidade_limite = 850; 
  }
  // ------------------------------ FIM Tratamento do botão e modos ----------------------------------------------------//
  
  // ------------------------------ SENSOR LDR ------------------------------------------------------------------------//
  Leitura = analogRead(AnalogLDR);  //Lê o valor fornecido pelo LDR 

  Brilho = map(Leitura, 1023, 0, 0, 100);
  unsigned long currentMillis = millis();
  //Verifica se o intervalo de display no lcd já foi atingido
  if (currentMillis - previousMillisLuminosidade >= intervaloLuminosidade)
  {
  //Armazena o valor da ultima vez que o LCD mostrou
  previousMillisLuminosidade = currentMillis;
  lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
  lcd.print("Luminosidade:       "); //IMPRIME O TEXTO NO DISPLAY LCD
  lcd.setCursor(0,1); //SETA A POSIÇÃO DO CURSOR
  lcd.print(Brilho); //IMPRIME O TEXTO NO DISPLAY LCD
  lcd.print("%      ");
  }
  LeituraAnterior = Leitura;
  // ------------------------------ FIM SENSOR LDR -----------------------------------------------------------------//

  // ------------------------------ SENSOR TEMPERATURA ------------------------------------------------------------//
  
  int temperature = temp.getTemp(); //VARIÁVEL DO TIPO INTEIRO QUE RECEBE O VALOR DE TEMPERATURA CALCULADO PELA BIBLIOTECA

  if (currentMillis - previousMillisTemperatura>= intervaloTemperatura)
  {
  previousMillisTemperatura = currentMillis;
  lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
  lcd.print("Temperatura:         "); //IMPRIME O TEXTO NO MONITOR SERIAL
  lcd.setCursor(0,1); //SETA A POSIÇÃO DO CURSOR
  lcd.print(temperature); //IMPRIME NO MONITOR SERIAL A TEMPERATURA MEDIDA
  lcd.println("*C             "); //IMPRIME O TEXTO NO MONITOR SERIAL
  }

  // ------------------------------ FIM SENSOR TEMPERATURA -------------------------------------------------------//

   // ------------------------------ SENSOR DE UMIDADE ----------------------------------------------------------//
  //Le o valor do pino A1 do sensor
  umidade = analogRead(AnalogUmidade);
  int Porcento = map(umidade, 1023, 0, 0, 100); // Relaciona o valor analógico à porcentagem
  if (currentMillis - previousMillisUmidade>= intervaloUmidade)
  {
  previousMillisUmidade = currentMillis;
  lcd.setCursor(0,0); //SETA A POSIÇÃO DO CURSOR
  lcd.print("Umidade:         "); //IMPRIME O TEXTO NO MONITOR SERIAL
  lcd.setCursor(0,1); //SETA A POSIÇÃO DO CURSOR
  lcd.print(Porcento); //IMPRIME NO MONITOR SERIAL A TEMPERATURA MEDIDA
  lcd.print("%      ");
  }

  //Solo umido, acende o led verde
  if (umidade > 0 && umidade < umidade_limite)
  {
    apaga_leds();
    digitalWrite(pino_led_verde, HIGH);
  }
 
   //Solo seco, acende led vermelho
  if (umidade>= umidade_limite && umidade < 1024)
  {
    apaga_leds();
    digitalWrite(pino_led_vermelho, HIGH);
  }
  
   if (umidade > umidade_limite ) 
   { //Se o valor analógico lido for maior que o pré-estabelecido para rega
      digitalWrite(pinoRele, HIGH); // Altera o estado do pinoRele para nível alto
    if(millis() - timer > intervaloRega && umidade < umidade_limite)
     {
       apaga_leds();
       digitalWrite(pino_led_amarelo, HIGH);
       digitalWrite(pinoRele, LOW); // Altera o estado do pinoRele para nível Baixo  
       timer = millis();         //Atualiza a referência
      }
    }
    else
    {
      digitalWrite(pinoRele, LOW);
    }
}
 // ------------------------------ FIM SENSOR DE UMIDADE -------------------------------------------------------//}

void apaga_leds()
{
  digitalWrite(pino_led_vermelho, LOW);
  digitalWrite(pino_led_amarelo, LOW);
  digitalWrite(pino_led_verde, LOW);
}
