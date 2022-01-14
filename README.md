# software-de-controle-e-monitoramento-de-um-biorreator-de-celulas-animais

#include <CheapStepper.h> // 
#include <ESP8266WiFi.h> // Importa a Biblioteca ESP8266WiFi
#include <PubSubClient.h> // Importa a Biblioteca PubSubClient
#include <DallasTemperature.h>
#include <OneWire.h>
#define wifi

#define ONE_WIRE_BUS 5                          //D1 pin of nodemcu

OneWire oneWire(ONE_WIRE_BUS);
 
DallasTemperature sensors(&oneWire);            // Pass the oneWire reference to Dallas Temperature.

// next, declare the stepper
// and connect pins 8,9,10,11 to IN1,IN2,IN3,IN4 on ULN2003 board

#define TopicMotor      "rolo/motor"     //tópico MQTT de escuta
#define TopicMotorFlag  "rolo/flag"
#define TopicSpeed      "rolo/speed"
#define TopicTemp       "rolo/temp"
#define TopicWiFi       "rolo/wifi"

#define ID_MQTT  "Rolo"     //id mqtt (para identificação de sessão)

char speed_stepper = 20;
const int stepsPerRevolution = 200;


char msg[5];


// WIFI
//const char* SSID = "UnivapWifi"; // SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "universidade"; // Senha da rede WI-FI que deseja se conectar

//const char* SSID = "IPDWifi"; // SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "pesquisa"; // Senha da rede WI-FI que deseja se conectar

const char* SSID = "HOME_DSK"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "probesserv"; // Senha da rede WI-FI que deseja se conectar

// MQTT
const char* BROKER_MQTT = "iot.eclipse.org"; //URL do broker MQTT que se deseja utilizar
//const char* BROKER_MQTT = "broker.hivemq.com"; //URL do broker MQTT que se deseja utilizar
//const char* BROKER_MQTT = "172.17.76.197"; //URL do broker MQTT que se deseja utilizar
//const char* BROKER_MQTT = "192.168.43.3"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883; // Porta do Broker MQTT

//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient

char EstadoSaida = '0';  //variável que armazena o estado atual da saída
int flagOut = 0;
unsigned int passo = 0;

void setup() {

    //inicializações:
    delay(2000);
    sensors.begin();
    InitOutput();
    initSerial();
#ifdef wifi
    initWiFi();
    initMQTT();
#endif

  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(D8, OUTPUT);


  digitalWrite(D5, 0);
  digitalWrite(D6, 0);
  digitalWrite(D7, 0);
  digitalWrite(D8, 0);
  // let's run the stepper at 12rpm (if using 5V power) - the default is ~16 rpm...

  //stepper.setRpm(speed_stepper);

  // let's print out the RPM to make sure the setting worked
 
  Serial.begin(115200);
  //Serial.print("stepper RPM: "); Serial.print(stepper.getRpm());
  Serial.println();

}

void loop() {

  char msg_temp[10];

    //int stepsLeft = stepper.getStepsLeft();
#ifdef wifi
    //garante funcionamento das conexões WiFi e ao broker MQTT
    VerificaConexoesWiFIEMQTT();
#endif 
    //envia o status de todos os outputs para o Broker no protocolo esperado
    //EnviaEstadoOutputMQTT();

  // we need to call run() during loop()
  // in order to keep the stepper moving
  // if we are using non-blocking moves
  sensors.requestTemperatures();                // Send the command to get temperatures  
  Serial.println("Temperature is: ");
  Serial.println(sensors.getTempCByIndex(0));

  sprintf (msg_temp, "%3.1f", sensors.getTempCByIndex(0));
  MQTT.publish(TopicTemp, msg_temp);

    if(flagOut)
    {
      digitalWrite(D5, 1);
      Serial.println("MOTOR ON");
      delay(50);
      
#ifdef wifi
          MQTT.publish(TopicMotorFlag, "L");
#endif   
    }else
    {
#ifdef wifi
      MQTT.publish(TopicMotorFlag, "D");
#endif
      digitalWrite(D5, 0);
    }

  ////////////////////////////////
  // now the stepper is moving, //
  // let's do some other stuff! //
  ////////////////////////////////

  // let's check how many steps are left in the current move:
 
 

  // if the current move is done...
#ifdef wifi
    //keep-alive da comunicação com broker MQTT
    MQTT.loop();
#endif
}

//Função: inicializa comunicação serial com baudrate 115200 (para fins de monitorar no terminal serial
//        o que está acontecendo.
//Parâmetros: nenhum
//Retorno: nenhum
void initSerial()
{
    Serial.begin(115200);
}
 
//Função: inicializa e conecta-se na rede WI-FI desejada
//Parâmetros: nenhum
//Retorno: nenhum
void initWiFi()
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
    
    reconectWiFi();
}
 
//Função: inicializa parâmetros de conexão MQTT(endereço do
//        broker, porta e seta função de callback)
//Parâmetros: nenhum
//Retorno: nenhum
void initMQTT()
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
 
//Função: função de callback
//        esta função é chamada toda vez que uma informação de
//        um dos tópicos subescritos chega)
//Parâmetros: nenhum
//Retorno: nenhum
void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
    String msg;
    char msg_speed[5];
 
    //obtem a string do payload recebido
    for(int i = 0; i < length; i++)
    {
       char c = (char)payload[i];
       msg += c;
    }
  
    //toma ação dependendo da string recebida:
    //verifica se deve colocar nivel alto de tensão na saída D0:
    //IMPORTANTE: o Led já contido na placa é acionado com lógica invertida (ou seja,
    //enviar HIGH para o output faz o Led apagar / enviar LOW faz o Led acender)
    if (msg.equals("L"))
    {
        digitalWrite(D0, HIGH);
        EstadoSaida = '1';
        flagOut = 1;
        Serial.println("MOTOR ON");
    }
 
    //verifica se deve colocar nivel alto de tensão na saída D0:
    if (msg.equals("D"))
    {
        digitalWrite(D0, LOW);
        EstadoSaida = '0';
        flagOut = 0;
        Serial.println("MOTOR OFF");
    }

    if (msg.equals("A"))
    {
        speed_stepper+=5;
        if(speed_stepper > 180) speed_stepper = 180;
        Serial.println("SPEED MOTOR UP");
        Serial.println(speed_stepper);
        digitalWrite(D6, 1);
        delay(10);
        digitalWrite(D6, 0);
    }

    if (msg.equals("B"))
    {
        speed_stepper-=5;
        if(speed_stepper < 0) speed_stepper = 0;
        Serial.println("SPEED MOTOR DOWN");
        Serial.println(speed_stepper);
        digitalWrite(D7, 1);
        delay(10);
        digitalWrite(D7, 0);
    }
    sprintf (msg_speed, "%d", speed_stepper);
    MQTT.publish(TopicSpeed, msg_speed);
    
}
 
//Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
//        em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectMQTT()
{
    while (!MQTT.connected())
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT))
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TopicMotor);
            MQTT.publish(TopicWiFi, "1");
        }
        else
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            MQTT.publish(TopicWiFi, "0");
            delay(2000);
        }
        Serial.println();
        Serial.print("Conectado com sucesso na rede ");
        Serial.println(SSID);
        Serial.println("IP obtido: ");
        Serial.println(WiFi.localIP());
    }
    MQTT.publish(TopicWiFi, "1");
}
 
//Função: reconecta-se ao WiFi
//Parâmetros: nenhum
//Retorno: nenhum
void reconectWiFi()
{
    //se já está conectado a rede WI-FI, nada é feito.
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
    Serial.println("Conectando...");    
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
    
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        Serial.print(".");
    }
  
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}
 
//Função: verifica o estado das conexões WiFI e ao broker MQTT.
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected()){
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
        MQTT.publish(TopicWiFi, "0");

    }else
        MQTT.publish(TopicWiFi, "1");
    
     reconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}
 
//Função: envia ao Broker o estado atual do output
//Parâmetros: nenhum
//Retorno: nenhum
void EnviaEstadoOutputMQTT(void)
{
    if (EstadoSaida == '0')
      MQTT.publish(TopicMotorFlag, "D");
 
    if (EstadoSaida == '1')
      MQTT.publish(TopicMotorFlag, "L");
 
    Serial.println("- Estado da saida D0 enviado ao broker!");
    delay(1000);
}
 
//Função: inicializa o output em nível lógico baixo
//Parâmetros: nenhum
//Retorno: nenhum
void InitOutput(void)
{
    //IMPORTANTE: o Led já contido na placa é acionado com lógica invertida (ou seja,
    //enviar HIGH para o output faz o Led apagar / enviar LOW faz o Led acender)
    pinMode(D0, OUTPUT);
    digitalWrite(D0, HIGH);         
}
#include <Stepper.h>

#define motor 2
#define up    3
#define down  4
#define cw    9

char speed_stepper = 60;
const int stepsPerRevolution = 200;

Stepper stepper(stepsPerRevolution, 5,6,7,8);

void setup() {
  // put your setup code here, to run once:
  pinMode(motor, INPUT);
  pinMode(up, INPUT);
  pinMode(down, INPUT);
  pinMode(cw, INPUT);
  stepper.setSpeed(speed_stepper);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  //stepper.step(-stepsPerRevolution);
  if(digitalRead(motor)) stepper.step(stepsPerRevolution);
  if(digitalRead(up))
  {
    if(speed_stepper > 150) speed_stepper = 150;else
    speed_stepper += 5;
  }

  if(digitalRead(down))
  {
    if(speed_stepper <1) speed_stepper = 0;else
    speed_stepper -= 5;
  }
  delay(10);
  stepper.setSpeed(speed_stepper);
}
