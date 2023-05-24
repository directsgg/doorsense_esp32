#include <Arduino.h>
#include <AsyncUDP.h>
#include <WiFi.h>
#include "AudioTools.h"
#include "AudioLibs/Communication.h"
#define GPIO_TRANSMIT_BUTTON 12

// camera
#include "esp_camera.h"
#include "esp_https_server.h"
#include "esp_timer.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

const char *ssid = "Nexxt_50E438";
const char *password = "otrointernet";
// udp
UDPStream udpStream(ssid, password);
IPAddress udpAddress(255, 255, 255, 255);
const int udpPort = 12345;

// audio i2s
I2SStream audioStreamI2S;
ConverterScaler<int16_t> volume(10.0, 0, 32767); // volume control - amplificar x2
StreamCopy copierUDPtoI2SOut(audioStreamI2S, udpStream);
StreamCopy copierI2SInToUDP(udpStream, audioStreamI2S);

// para controlar el estado del programa
AsyncUDP udpControlState;
// 0x00 - no hacer nada
// 0x0F - microfono
// 0xF0 - bocina
byte estadoControl = 0x00;

// door bell api key
const char *host = "maker.ifttt.com";
const char *privateKey = "cbjc4tNWaK-y3NqWKnZPGa";
// door bell config variables
int buttonState;           // the current reading from the input pin
int lastButtonState = LOW; // the previous reading from the input pin
long lastDebounceTime = 0; // the last time the output pin was toggled
long debounceDelay = 50;   // the debounce time; increase if the output flickers

static esp_err_t init_camera(void)
{
  camera_config_t configCamera;
  configCamera.ledc_channel = LEDC_CHANNEL_0;
  configCamera.ledc_timer = LEDC_TIMER_0;
  configCamera.pin_d0 = Y2_GPIO_NUM;
  configCamera.pin_d1 = Y3_GPIO_NUM;
  configCamera.pin_d2 = Y4_GPIO_NUM;
  configCamera.pin_d3 = Y5_GPIO_NUM;
  configCamera.pin_d4 = Y6_GPIO_NUM;
  configCamera.pin_d5 = Y7_GPIO_NUM;
  configCamera.pin_d6 = Y8_GPIO_NUM;
  configCamera.pin_d7 = Y9_GPIO_NUM;
  configCamera.pin_xclk = XCLK_GPIO_NUM;
  configCamera.pin_pclk = PCLK_GPIO_NUM;
  configCamera.pin_vsync = VSYNC_GPIO_NUM;
  configCamera.pin_href = HREF_GPIO_NUM;
  configCamera.pin_sscb_sda = SIOD_GPIO_NUM;
  configCamera.pin_sscb_scl = SIOC_GPIO_NUM;
  configCamera.pin_pwdn = PWDN_GPIO_NUM;
  configCamera.pin_reset = RESET_GPIO_NUM;
  configCamera.xclk_freq_hz = 20000000;
  configCamera.pixel_format = PIXFORMAT_JPEG;
  // init with high specs to pre-allocate larger buffers
  // if (psramFound())
  //{
  //  configCamera.frame_size = FRAMESIZE_UXGA;
  //  configCamera.jpeg_quality = 10;
  //  configCamera.fb_count = 2;
  //  configCamera.grab_mode = CAMERA_GRAB_WHEN_EMPTY; // CAMERA_GRAB_LATEST. Sets when buffers should be filled
  //}
  // else
  //{
  configCamera.frame_size = FRAMESIZE_SVGA;
  configCamera.jpeg_quality = 12;
  configCamera.fb_count = 1;

  //}

  // camera init
  esp_err_t err = esp_camera_init(&configCamera);
  if (err != ESP_OK)
  {
    Serial.print("Camera init failed with error: ");
    Serial.println(err);
    while (true)
    {
    };
    return err;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the blightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  Serial.println("Camera init");
  return ESP_OK;
}

esp_err_t jpg_stream_httpd_handler(httpd_req_t *req)
{
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  char *part_buf[128];
  static int64_t last_frame = 0;
  if (!last_frame)
  {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK)
  {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "X-Framerate", "60");

  while (true)
  {
    fb = esp_camera_fb_get();
    if (!fb)
    {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
      break;
    }
    else
    {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
    }
    if (fb->format != PIXFORMAT_JPEG)
    {
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      if (!jpeg_converted)
      {
        Serial.println("JPEG compression failed");
        res = ESP_FAIL;
      }
    }
    else
    {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK)
    {
      size_t hlen = snprintf((char *)part_buf, 128, _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK)
    {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (fb)
    {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    else if (_jpg_buf)
    {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK)
    {
      Serial.println("Send frame failed");
      break;
    }

    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
  }
  last_frame = 0;

  return res;
}

httpd_handle_t start_live_stream_server(void)
{
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;
  config.server_port += 1;
  config.ctrl_port += 1;
  httpd_uri_t stream_uri = {
      .uri = "/stream",
      .method = HTTP_GET,
      .handler = jpg_stream_httpd_handler,
      .user_ctx = NULL};

  httpd_handle_t stream_httpd = NULL;

  if (httpd_start(&stream_httpd, &config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.print("Starting stream server on port: ");
    Serial.println(config.server_port);
  }
  return stream_httpd;
}

void initUDPControlState()
{
  const int portUdpControlState = 12344;
  if (udpControlState.listen(portUdpControlState))
  {
    Serial.print("UDP Listening on IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(" on Port: ");
    Serial.println(portUdpControlState);
    udpControlState.onPacket([](AsyncUDPPacket packet)
                             {
        Serial.print("UDP Packet Type: ");
        Serial.print(packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast"
                                                                              : "Unicast");
        Serial.print(", From: ");
        Serial.print(packet.remoteIP());
        Serial.print(":");
        Serial.print(packet.remotePort());
        Serial.print(", To: ");
        Serial.print(packet.localIP());
        Serial.print(":");
        Serial.print(packet.localPort());
        Serial.print(", Length: ");
        Serial.print(packet.length());
        Serial.print(", Data: ");
        Serial.write(packet.data(), packet.length());
        Serial.println();

        char* tmpStr = (char*) malloc(packet.length() + 1);
        mempcpy(tmpStr, packet.data(), packet.length()); 
        tmpStr[packet.length()] = '\0'; // ensure null termination
        String mensaje = String(tmpStr);
        free(tmpStr); // String(char*) creates a copy so we can delete our one
        Serial.print("Mensaje string: ");
        Serial.println(mensaje);
        if (mensaje == "ip") {
          String m = WiFi.localIP().toString();
          packet.print(m);
        } });
  }
}

static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
  char buf;
  int ret;

  if ((ret = httpd_req_recv(req, &buf, 1)) <= 0)
  {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }
  /*
  para hacer una consulta en curl
  curl -X PUT -d "t" 192.168.0.114:80/ctrl
  */

  // para controlar el estado del programa
  // 0x00 - no hacer nada
  // 0x0F - microfono
  // 0xF0 - bocina

  // generar el control
  if (buf == 'm')
  {
    estadoControl = 0x0F;
  }
  else if (buf == 'b')
  {
    estadoControl = 0xF0;
  }
  else
  {
    estadoControl = 0x00;
  }
  Serial.print("mensaje http put: ");
  Serial.println(buf);

  // respond with empty body
  httpd_resp_send(req, NULL, 0);
  return ESP_OK;
}

static httpd_handle_t startWebServerControl(void)
{
  httpd_handle_t server = NULL;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.lru_purge_enable = true;

  httpd_uri_t ctrl_uri = {
      .uri = "/ctrl",
      .method = HTTP_PUT,
      .handler = ctrl_put_handler,
      .user_ctx = NULL};
  // start the httpd server
  Serial.print("Starting web server control on port ");
  Serial.print(config.server_port);
  if (httpd_start(&server, &config) == ESP_OK)
  {
    // registrar URI handlers
    httpd_register_uri_handler(server, &ctrl_uri);
    Serial.println(": iniciado");
    return server;
  }
  Serial.println(": fallo al iniciar");
  return NULL;
}

void send_event_door_bell(const char *event)
{
  Serial.print("Connecting to ");
  Serial.println(host);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 80;
  if (!client.connect(host, httpPort))
  {
    Serial.println("Connection failed");
    return;
  }

  // We now create a URI for the request
  String url = "/trigger/";
  url += event;
  url += "/with/key/";
  url += privateKey;

  Serial.print("Requesting URL: ");
  Serial.println(url);

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Connection: close\r\n\r\n");
  while (client.connected())
  {
    if (client.available())
    {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    else
    {
      // No data yet, wait a bit
      delay(50);
    };
  }

  Serial.println();
  Serial.println("closing connection");
  client.stop();
}

void setup()
{

  // put your setup code here, to run once:
  Serial.begin(115200);
  AudioLogger::instance().begin(Serial, AudioLogger::Info);

  // define udpStream address and port -> open output
  udpStream.begin(udpAddress, udpPort);

  auto configI2S = audioStreamI2S.defaultConfig(RXTX_MODE);
  configI2S.sample_rate = 16000;
  configI2S.channels = 1;
  configI2S.bits_per_sample = 16;
  configI2S.i2s_format = I2S_STD_FORMAT;
  configI2S.port_no = I2S_NUM_1;
  configI2S.buffer_size = 1024;
  configI2S.buffer_count = 4;
  configI2S.pin_ws = 14;
  configI2S.pin_bck = 15;
  configI2S.pin_data = 13;
  configI2S.pin_data_rx = 2;
  audioStreamI2S.begin(configI2S);

  // control
  initUDPControlState();
  startWebServerControl();

  // camara
  init_camera();
  start_live_stream_server();

  // configurar boton de transmision
  pinMode(GPIO_TRANSMIT_BUTTON, INPUT_PULLDOWN);
}

void loop()
{
  // para controlar el estado del programa
  // 0x00 - no hacer nada
  // 0x0F - microfono
  // 0xF0 - bocina

  // necesita comenzar a transmitir o recibir
  if (estadoControl == 0x0F)
  {
    Serial.println("Iniciar transmision de audio");
    unsigned long start_time = millis();
    while (millis() - start_time < 500)
    {
      // esperar al menos un 1/2 segundo
    }
    while (estadoControl == 0x0F)
    {
      copierI2SInToUDP.copy(volume);
    }
    Serial.println("Finalizar transmision de audio");
  }
  if (estadoControl == 0xF0)
  {
    Serial.println("Recibir audio");
    unsigned long start_time = millis();
    while (millis() - start_time < 1000)
    {
      // esperar al menos un segundo
    }
    while (estadoControl == 0xF0)
    {
      copierUDPtoI2SOut.copy();
    }
    Serial.println("Finalizar recepcion de audio");
  }

  // door bell
  while (WiFi.status() == WL_DISCONNECTED)
  {
    ESP.restart();
    Serial.print("Connection Lost");
  }
  int reading = digitalRead(GPIO_TRANSMIT_BUTTON);
  if (reading != lastButtonState)
  {

    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay)
  {
    // if the button state has changed:
    if (reading != buttonState)
    {
      Serial.print("Button now ");
      Serial.println(HIGH == reading ? "HIGH" : "LOW");
      buttonState = reading;

      // When the button is in the HIGH state (pulled high) the button has been pressed so send the event.
      if (buttonState == HIGH)
      {
        send_event_door_bell("button_pressed");
        Serial.print("button pressed");
      }
    }
  }

  // save the reading.  Next time through the loop,
  lastButtonState = reading;
}
