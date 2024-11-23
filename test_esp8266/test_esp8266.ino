#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>

const int led = 1;

const char* ssid = "Q.Truong"; // SSID WiFi của bạn
const char* password = "123456667"; // Mật khẩu WiFi của bạn
const char* mqtt_server = "604b372efa0e444ebbe062ca5d3e2243.s1.eu.hivemq.cloud"; // Máy chủ MQTT
const int mqtt_port = 8883; // Cổng MQTT
const char* mqtt_username = "truong"; // Tên người dùng MQTT
const char* mqtt_password = "Lhh12345@@"; // Mật khẩu MQTT

WiFiClientSecure espClient;
PubSubClient client(espClient);

float straightSpeed = 0;
float rotationSpeed = 0;
float Vx = 0;
float Vw = 0;
int internet  = 0;
int status = 0;
bool isEnabled = false; // Biến để theo dõi trạng thái "ON"

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  // digitalWrite(led,1);


}


// Callback to handle received messages
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  // Kiểm tra tin nhắn để trích xuất tốc độ di chuyển thẳng và tốc độ quay
  if (message.startsWith("STRAIGHT_SPEED:")) {
    String speedValue = message.substring(15);
    straightSpeed = speedValue.toFloat();
  } else if (message.startsWith("ROTATION_SPEED:")) {
    String speedValue = message.substring(15);
    rotationSpeed = speedValue.toFloat();
  } else if (message == "ENABLE") {
    isEnabled = true; // Kích hoạt điều khiển
    status = 1; Vx=0;Vw=0;
    gui_uart();
    // Serial.println("Robot enabled");
  } else if (message == "DISABLE") {
    isEnabled = false; // Vô hiệu hóa điều khiển
    status=0;
    Vx = 0;
    Vw = 0;
    gui_uart();
    // Serial.println("Robot disabled");
  }

  if (isEnabled) {
    if (message == "UP") {
      Vx = straightSpeed;
      Vw = 0;
      gui_uart();
    } else if (message == "DOWN") {
      Vx = -straightSpeed;
      Vw = 0;
      gui_uart();
    } else if (message == "LEFT") {
      Vw = rotationSpeed;
      Vx = 0;
      gui_uart();
    } else if (message == "RIGHT") {
      Vw = -rotationSpeed;
      Vx = 0;
      gui_uart();
    } else if (message == "STOP") {
      Vx = 0;
      Vw = 0;
      gui_uart();
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientID = "ESPClient-";
    clientID += String(random(0xffff), HEX);
    
    if (client.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
      client.subscribe("esp8266/client"); // Đăng ký chủ đề MQTT
        internet = 1; Vx=0;Vw=0;
        gui_uart();
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      // no_internet();
      internet = 0; Vx=0;Vw=0;
      gui_uart();
      Serial.println(" Trying again in 5 seconds");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  // pinMode(led,OUTPUT);
  // digitalWrite(led,0);
  internet=0;
  gui_uart();
  setup_wifi();
  
  espClient.setInsecure(); // Bỏ qua xác minh SSL
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

}

void gui_uart() {
  String inputData = String(Vx) + ";" + String(Vw) + ";" +String(internet) + ";" + String(status);
  Serial.println(inputData); // In dữ liệu để gửi ra UART
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  
  client.loop(); // Duy trì kết nối MQTT
}