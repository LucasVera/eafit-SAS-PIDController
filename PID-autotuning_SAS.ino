/********************************************************
 * Controlador de temperatura y humedad
 * 
 * 
 * Este código muestra el uso de un controlador de humedad
 * automático basado en la librería PID_v1, en un board
 * estilo Arduino con el chip ESP8266, el cual permite
 * hacer conexiones por wifi y crear servidores web en una
 * red local.
 * 
 * El dispositivo recibe los setpoints a través de
 * endpoints y basado en estos setpoint y en la lectura de
 * las variables a través de sensores, crea una acción de
 * control que envía por sus salidas de PWM.
 ********************************************************/

#include <PID_v1.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

/*
 * T_PIN: Pin donde estara conectado el sensor de temperatura
 * H_PIN: Pin donde estara conectado el sensor de humedad
*/ 
#define T_INPUT_PIN 0
#define H_INPUT_PIN 1
#define T_OUTPUT_PIN 3
#define H_OUTPUT_PIN 5


// Parametros para conectarse a la red WiFi
const char* ssid = "Nombre wifi";
const char* password = "Contraseña wifi";
ESP8266WebServer server(80);

// Se definen las variables de los sistemas
double T_setpoint, T_input, T_output;
double H_setpoint, H_input, H_output;

// Se definen los parametros agresivos y conservativos para ambos
// controladores
double T_aggKp=0.5, T_aggKi=0.025, T_aggKd=0.005;
double T_consKp=0.125, T_consKi=0.00625, T_consKd=0.0.00125;

double H_aggKp=1, H_aggKi=0.5, H_aggKd=0.75;
double H_consKp=0.25, H_consKi=0.125, H_consKd=0.1875;

// Inicializa los controladores con sus parametros iniciales
PID T_PID(&T_input, &T_output, T_setpoint, T_consKp, T_consKi, T_consKd, DIRECT);
PID H_PID(&H_input, &H_output, H_setpoint, H_consKp, H_consKi, H_consKd, DIRECT);

void setup()
{
  // Configurar los pines
  pinMode(T_INPUT_PIN, INPUT);
  pinMode(H_INPUT_PIN, INPUT);
  pinMode(T_OUTPUT_PIN, OUTPUT);
  pinMode(H_OUTPUT_PIN, OUTPUT);
  T_setpoint = 0;
  H_setpoint = 0;

  // Inicializamos el led built-in que viene en el arduino/esp8266.
  // Solo se usa como indicador visual cuando se está transmitiendo
  // datos por wifi.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);

  // Conectarse a WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Conectado a ");
  Serial.println(ssid);
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  // Rutas del servidor:
  // - reportar datos
  // - actualizar setpoint de temperatura
  // - actualizar setpoint de humedad
  // - manejar not-found
  server.on("/get-state", handleGetState);
  server.on(UriBraces("/t-setpoint/{}"), handleNewSetpoint_T);
  server.on(UriBraces("/h-setpoint/{}"), handleNewSetpoint_H);
  server.onNotFound(handleNotFound);

  // Inicia los controladores
  T_PID.SetMode(AUTOMATIC);
  H_PID.SetMode(AUTOMATIC);

  // Se apaga el led, indicando que el setup termino
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  // Se maneja una peticion en caso de que haya una y se reporta
  // estado al wifi gateway
  server.handleClient();
  MDNS.update();

  // Se toman las lecutras actuales de los sensores
  T_input = analogRead(T_INPUT_PIN);
  H_input = analogRead(H_INPUT_PIN);

  // ------- Temperatura ---------
  // Se calcula el error actual
  double T_gap = abs(T_setpoint - T_input);
  if (T_gap < 10)
  {
    // Estamos cerca al set-point. Usar constantes conservativas
    T_PID.SetTunings(T_consKp, T_consKi, T_consKd);
  }
  else
  {
     // Estamos lejos del set-point. Usar constantes agresivas
     T_PID.SetTunings(T_aggKp, T_aggKi, T_aggKd);
  }

  // La librería realiza los calculos correspondientes a la accion de control
  T_PID.Compute();
  analogWrite(T_OUTPUT_PIN, T_output);

  // ------- Humedad ---------
  // Se calcula el error actual
  double H_gap = abs(H_setpoint - H_input);
  if (H_gap < 10)
  {
    // Estamos cerca al set-point. Usar constantes conservativas
    H_PID.SetTunings(H_consKp, H_consKi, H_consKd);
  }
  else
  {
     // Estamos lejos del set-point. Usar constantes agresivas
     H_PID.SetTunings(H_aggKp, H_aggKi, H_aggKd);
  }

  // La librería realiza los calculos correspondientes a la accion de control
  H_PID.Compute();
  analogWrite(H_OUTPUT_PIN, H_output);
}

// Esta funcion es el handler del servidor y sirve para reportar valores actuales
void handleGetState() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Reportando datos por wifi");
  String message = ""
  message += "{"
  message += "\"temperature\": " + String(T_input, 4) + ","
  message += "\"humidity\": " + String(H_input, 4)
  message += "}"
  server.send(200, "application/json", message);
  digitalWrite(LED_BUILTIN, LOW);
}

// recibe un nuevo setpoint de temperatura
void handleNewSetpoint_T() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Recibiendo nuevo setpoint de temperatura");
  String newSetpoint = server.pathArg(0);
  T_setpoint = newSetpoint.toDouble();
  server.send(200, "text/plain", "T - Done");
  digitalWrite(LED_BUILTIN, LOW);
}

// recibe un nuevo setpoint de humedad
void handleNewSetpoint_H() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Recibiendo nuevo setpoint de humedad");
  String newSetpoint = server.pathArg(0);
  H_setpoint = newSetpoint.toDouble();
  server.send(200, "text/plain", "H - Done");
  digitalWrite(LED_BUILTIN, LOW);
}

// Funcion para hacer el handle de peticiones not found
void handleNotFound() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Manejando una peticion 'Not found'");
  server.send(404, "text/plain", "Not found");
  digitalWrite(LED_BUILTIN, LOW);
}
