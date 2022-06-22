/*



  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

https://create.arduino.cc/projecthub/mariogianota/nextion-3-5-lcd-digital-compass-for-arduino-uno-4b12a0#widget-comments


https://github.com/vlaate/vladsc/blob/master/vlaDSC.ino
*/

#include "config.h"

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <Encoder.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

QMC5883LCompass compass;

long AzPos = 0;
long AltPos = 0;
long AzPosPrev = -999;
Encoder encAz(enc_1A, enc_1B); // 15 NOK, 2 OK
Encoder encAlt(enc_2A, enc_2B);

ESP8266WebServer server_http(80);
WiFiServer server(4030); // 4030 is the default port Skysafari uses for WiFi connection to telescopes
WiFiClient remoteClient; // represents the connection to the remote app (Skysafari)

#define RADIAN_TO_STEPS 162.33804f
#define STEPS_IN_FULL_CIRCLE 10200 // number of steps in a full circle, should match the skysafari setting for basic encoder
#define EMA_WEIGHT 0.05f           // weight coefficient for new values vs old values used in the exponential moving average smoothing algorithm

const double pi = 3.14159265358979324;

double cos_phi, sin_phi;
long TSL;
long AR_tel_s, DEC_tel_s;
char txAR[10];
char txDEC[11];
long Az_tel_s, Alt_tel_s;

unsigned long t_ciclo_acumulado = 0, t_ciclo;
unsigned long seg_sideral = 1003;

/*
   This sample sketch demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

/*
  U8g2lib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.

*/

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R3, /* reset=*/U8X8_PIN_NONE);

void u8g2_prepare(void)
{
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

void u8g2_box_frame(uint8_t a)
{
  u8g2.drawStr(0, 0, "drawBox");
  u8g2.drawBox(5, 10, 20, 10);
  u8g2.drawBox(10 + a, 15, 30, 7);
  u8g2.drawStr(0, 30, "drawFrame");
  u8g2.drawFrame(5, 10 + 30, 20, 10);
  u8g2.drawFrame(10 + a, 15 + 30, 30, 7);
}

void u8g2_disc_circle(uint8_t a)
{
  u8g2.drawStr(0, 0, "drawDisc");
  u8g2.drawDisc(10, 18, 9);
  u8g2.drawDisc(24 + a, 16, 7);
  u8g2.drawStr(0, 30, "drawCircle");
  u8g2.drawCircle(10, 18 + 30, 9);
  u8g2.drawCircle(24 + a, 16 + 30, 7);
}

void u8g2_r_frame(uint8_t a)
{
  u8g2.drawStr(0, 0, "drawRFrame/Box");
  u8g2.drawRFrame(5, 10, 40, 30, a + 1);
  u8g2.drawRBox(50, 10, 25, 40, a + 1);
}

void u8g2_string(uint8_t a)
{
  u8g2.setFontDirection(0);
  u8g2.drawStr(30 + a, 31, " 0");
  u8g2.setFontDirection(1);
  u8g2.drawStr(30, 31 + a, " 90");
  u8g2.setFontDirection(2);
  u8g2.drawStr(30 - a, 31, " 180");
  u8g2.setFontDirection(3);
  u8g2.drawStr(30, 31 - a, " 270");
}

void u8g2_line(uint8_t a)
{
  u8g2.drawStr(0, 0, "drawLine");
  u8g2.drawLine(7 + a, 10, 40, 55);
  u8g2.drawLine(7 + a * 2, 10, 60, 55);
  u8g2.drawLine(7 + a * 3, 10, 80, 55);
  u8g2.drawLine(7 + a * 4, 10, 100, 55);
}

void u8g2_triangle(uint8_t a)
{
  uint16_t offset = a;
  u8g2.drawStr(0, 0, "drawTriangle");
  u8g2.drawTriangle(14, 7, 45, 30, 10, 40);
  u8g2.drawTriangle(14 + offset, 7 - offset, 45 + offset, 30 - offset, 57 + offset, 10 - offset);
  u8g2.drawTriangle(57 + offset * 2, 10, 45 + offset * 2, 30, 86 + offset * 2, 53);
  u8g2.drawTriangle(10 + offset, 40 + offset, 45 + offset, 30 + offset, 86 + offset, 53 + offset);
}

void u8g2_ascii_1()
{
  char s[2] = " ";
  uint8_t x, y;
  u8g2.drawStr(0, 0, "ASCII page 1");
  for (y = 0; y < 6; y++)
  {
    for (x = 0; x < 16; x++)
    {
      s[0] = y * 16 + x + 32;
      u8g2.drawStr(x * 7, y * 10 + 10, s);
    }
  }
}

void u8g2_ascii_2()
{
  char s[2] = " ";
  uint8_t x, y;
  u8g2.drawStr(0, 0, "ASCII page 2");
  for (y = 0; y < 6; y++)
  {
    for (x = 0; x < 16; x++)
    {
      s[0] = y * 16 + x + 160;
      u8g2.drawStr(x * 7, y * 10 + 10, s);
    }
  }
}

void u8g2_extra_page(uint8_t a)
{
  u8g2.drawStr(0, 0, "Unicode");
  u8g2.setFont(u8g2_font_unifont_t_symbols);
  u8g2.setFontPosTop();
  u8g2.drawUTF8(0, 24, "☀ ☁");
  switch (a)
  {
  case 0:
  case 1:
  case 2:
  case 3:
    u8g2.drawUTF8(a * 3, 36, "☂");
    break;
  case 4:
  case 5:
  case 6:
  case 7:
    u8g2.drawUTF8(a * 3, 36, "☔");
    break;
  }
}

uint8_t draw_state = 0;

void draw(void)
{
  u8g2_prepare();
  switch (draw_state >> 3)
  {
  case 0:
    u8g2_box_frame(draw_state & 7);
    break;
  case 1:
    u8g2_disc_circle(draw_state & 7);
    break;
  case 2:
    u8g2_r_frame(draw_state & 7);
    break;
  case 3:
    u8g2_string(draw_state & 7);
    break;
  case 4:
    u8g2_line(draw_state & 7);
    break;
  case 5:
    u8g2_triangle(draw_state & 7);
    break;
  case 6:
    u8g2_ascii_1();
    break;
  case 7:
    u8g2_ascii_2();
    break;
  case 8:
    u8g2_extra_page(draw_state & 7);
    break;
  }
}

void update_display(int a)
{
  u8g2_prepare();
  char angle_array[] = "    "; // better to start with an empty string to avoid garbage
  dtostrf(a, 3, 0, angle_array);
  angle_array[3] = char(176);

  u8g2.drawStr(2, 2, angle_array);

  char Alt_array[] = "       "; // better to start with an empty string to avoid garbage
  char aZ_array[] = "       ";  // better to start with an empty string to avoid garbage
  dtostrf(AzPos, 3, 0, aZ_array);
  u8g2.drawStr(2, 15, "Ve:");
  u8g2.drawStr(25, 15, aZ_array);
  dtostrf(AltPos, 3, 0, Alt_array);
  u8g2.drawStr(2, 30, "Ho:");
  u8g2.drawStr(25, 30, Alt_array);

  // u8g2.drawStr(2, 45, "RA:");
  u8g2.drawStr(2, 45, txAR);

  // u8g2.drawStr(2, 50, "DEC:");
  u8g2.drawStr(2, 60, txDEC);

  // u8g2.drawStr(2, 65, "DEC:");
  // u8g2.drawStr(20, 65, txDEC);

  if (gps.time.isValid())
  {
    char time[] = "  "; // better to start with an empty string to avoid garbage
    dtostrf(gps.time.minute(), 1, 0, time);
    u8g2.drawStr(40, 2, time);
    char time2[] = "  "; // better to start with an empty string to avoid garbage
    dtostrf(gps.time.second(), 1, 0, time2);
    u8g2.drawStr(60, 2, time2);
  }
  else
  {
    u8g2.drawStr(40, 2, "<NC>");
    // Serial.print(F("INVALID"));
  }
}

void AZ_to_EQ()
{
  double delta_tel, sin_h, cos_h, sin_A, cos_A, sin_DEC, cos_DEC;
  double H_telRAD, h_telRAD, A_telRAD;
  long H_tel;
  long arHH, arMM, arSS;
  long decDEG, decMM, decSS;
  char sDEC_tel;

  A_telRAD = (Az_tel_s / 3600.0) * pi / 180.0;
  h_telRAD = (Alt_tel_s / 3600.0) * pi / 180.0;
  sin_h = sin(h_telRAD);
  cos_h = cos(h_telRAD);
  sin_A = sin(A_telRAD);
  cos_A = cos(A_telRAD);
  delta_tel = asin((sin_phi * sin_h) + (cos_phi * cos_h * cos_A));
  sin_DEC = sin(delta_tel);
  cos_DEC = cos(delta_tel);
  DEC_tel_s = long((delta_tel * 180.0 / pi) * 3600.0);

  while (DEC_tel_s >= 324000)
  {
    DEC_tel_s = DEC_tel_s - 324000;
  }
  while (DEC_tel_s <= -324000)
  {
    DEC_tel_s = DEC_tel_s + 324000;
  }

  H_telRAD = acos((sin_h - (sin_phi * sin_DEC)) / (cos_phi * cos_DEC));
  H_tel = long((H_telRAD * 180.0 / pi) * 240.0);

  if (sin_A >= 0)
  {
    H_tel = 86400 - H_tel;
  }
  AR_tel_s = TSL - H_tel;

  while (AR_tel_s >= 86400)
  {
    AR_tel_s = AR_tel_s - 86400;
  }
  while (AR_tel_s < 0)
  {
    AR_tel_s = AR_tel_s + 86400;
  }

  // Serial.print("AR_tel_s: ");
  // Serial.println(AR_tel_s);
  // Serial.print("DEC_tel_s: ");
  // Serial.println(DEC_tel_s);

  arHH = AR_tel_s / 3600;
  arMM = (AR_tel_s - arHH * 3600) / 60;
  arSS = (AR_tel_s - arHH * 3600) - arMM * 60;
  decDEG = abs(DEC_tel_s) / 3600;
  decMM = (abs(DEC_tel_s) - decDEG * 3600) / 60;
  decSS = (abs(DEC_tel_s) - decDEG * 3600) - decMM * 60;
  (DEC_tel_s < 0) ? sDEC_tel = 45 /* - */ : sDEC_tel = 43 /* + */;

  sprintf(txAR, "%02dh%02dm%02ds", int(arHH), int(arMM), int(arSS));
  sprintf(txDEC, "%c%02d%c%02d'%02d\"", sDEC_tel, int(decDEG), char(176), int(decMM), int(decSS));
}

void readEncoder()
{
  long newPosition1 = encAlt.read();
  long newPosition2 = encAz.read();
  if (newPosition1 == AltPos && newPosition2 == AzPos)
  {
    return;
  }

  AltPos = newPosition1;
  AzPos = newPosition2;

  // Serial.print("ENC 1 (ALT): ");
  // Serial.println(newPosition1);
  // Serial.print("Alt_tel_s: ");
  // Serial.println(Alt_tel_s);
  // Serial.print("ENC 2 (AZ): ");
  // Serial.println(newPosition2);
  // Serial.print("Az_tel_s: ");
  // Serial.println(Az_tel_s);

  if (AzPos >= pulses_enc2 || AzPos <= -pulses_enc2)
  {
    AzPos = 0;
  }
  int enc1 = AltPos / 1500;
  long encoder1_temp = AltPos - (enc1 * 1500);
  long map1 = enc1 * map(1500, 0, pulses_enc1, 0, 324000);
  int enc2 = AzPos / 1500;
  long encoder2_temp = AzPos - (enc2 * 1500);
  long map2 = enc2 * map(1500, 0, pulses_enc2, 0, 1296000);

  Alt_tel_s = map1 + map(encoder1_temp, 0, pulses_enc1, 0, 324000);
  Az_tel_s = map2 + map(encoder2_temp, 0, pulses_enc2, 0, 1296000);

  if (Az_tel_s < 0)
    Az_tel_s = 1296000 + Az_tel_s;
  if (Az_tel_s >= 1296000)
    Az_tel_s = Az_tel_s - 1296000;

  AZ_to_EQ();

  // Serial.print("txAR: ");
  // Serial.println(txAR);
  // Serial.print("txDEC: ");
  // Serial.println(txDEC);
}

void handle_config_page()
{
  Serial.print("txAR: ");
  Serial.println(txAR);
  Serial.print("txDEC: ");
  Serial.println(txDEC);
  server_http.send(200, "text/plain", "HELLO");
}

void handle_set_pole_ha()
{
  poleH_HH = server_http.arg(0).toInt();
  poleH_MM = server_http.arg(1).toInt();
  poleH_SS = server_http.arg(2).toInt();
  TSL = poleAR_HH * 3600 + poleAR_MM * 60 + poleAR_SS + poleH_HH * 3600 + poleH_MM * 60 + poleH_SS;
  while (TSL >= 86400)
    TSL = TSL - 86400;
  Serial.printf("%02dh%02dm%02ds", int(poleH_HH), int(poleH_MM), int(poleH_SS));

  server_http.send(200, "text/plain", "OK");
}

void setup(void)
{
  Serial.begin(GPSBaud);
  Serial.println("\nESP266 boot");

  WiFi.begin(ssid, password); // Connect to the network
  Serial.print("Connecting to ");
  Serial.print(ssid);
  Serial.println(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED)
  { // Wait for the Wi-Fi to connect
    delay(1000);
    Serial.print(++i);
    Serial.print(' ');
  }
  Serial.println('\n');
  Serial.println("Connection established!");
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());

  // tcp listener to receive incoming connections from Skysafari:
  server.begin();
  server.setNoDelay(true);

  server_http.on("/", handle_config_page);
  server_http.on("/set-pole-ha", handle_set_pole_ha);
  server_http.begin();

  Wire.begin(0, 2);
  // Serial.print("Start DobDSC");
  compass.init();
  compass.setSmoothing(10, true);
  // compass.setCalibration(-1488, 1417, -1366, 1645, -251, 2418);
  compass.setCalibration(-906, 453, -657, 880, -913, 0);

  sprintf(txAR, " -");
  sprintf(txDEC, " -");

  u8g2.begin();

  cos_phi = cos((((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);
  sin_phi = sin((((latHH * 3600) + (latMM * 60) + latSS) / 3600.0) * pi / 180.0);

  TSL = poleAR_HH * 3600 + poleAR_MM * 60 + poleAR_SS + poleH_HH * 3600 + poleH_MM * 60 + poleH_SS;
  while (TSL >= 86400)
    TSL = TSL - 86400;
}

void attendTcpRequests()
{
  // check for new or lost connections:
  if (server.hasClient())
  {
    Serial.println("hasClient!");
    if (!remoteClient || !remoteClient.connected())
    {
      if (remoteClient)
      {
        Serial.print("Client Disconnected\n");
        remoteClient.stop();
      }
      remoteClient = server.available();
      Serial.print("Inbound connection from: ");
      Serial.println(remoteClient.remoteIP());
      //  remoteClient.flush();
      remoteClient.setNoDelay(true);
    }
  }

  // when we have a new incoming connection from Skysafari:
  while (remoteClient.available())
  {
    byte skySafariCommand = remoteClient.read();

    if (skySafariCommand == 81) // 81 is ascii for Q, which is the only command skysafari sends to "basic encoders"
    {
      char encoderResponse[20];
      // int iAzimuthReading = imu.smoothAzimuthReading;
      // long iAzimuthReading = encAz.read();
      long iAzimuthReading = AzPos;
      int iAltitudeReading = 0;
      sprintf(encoderResponse, "%i\t%i\t\n", iAzimuthReading, iAltitudeReading);
      //      Serial.println(encoderResponse);
    }
    else if (skySafariCommand == 72) // 'H' - request for encoder resolution, e.g. 10000-10000\n
    {
      char response[20];
      // Resolution on both axis is equal
      snprintf(response, 20, "%u-%u", STEPS_IN_FULL_CIRCLE, STEPS_IN_FULL_CIRCLE);
      //      Serial.println(response);

      remoteClient.println(response);
    }
    else if (skySafariCommand == -100) // 'H' - request for encoder resolution, e.g. 10000-10000\n
    {
      // rset AZ
      encAz.write(0);
    }
    else
    {
      Serial.print("*****");
      Serial.println(skySafariCommand);
    }
  }
}

void printHex(uint8_t num)
{
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

void loop(void)
{
  t_ciclo = millis();
  if (t_ciclo_acumulado >= seg_sideral)
  {
    TSL++;
    t_ciclo_acumulado = t_ciclo_acumulado - seg_sideral;
    if (TSL >= 86400)
    {
      TSL = TSL - 86400;
    }
  }
  int heading;

  // Read compass values
  compass.read();

  readEncoder();

  server_http.handleClient();

  // Return Azimuth reading
  heading = compass.getAzimuth();
  heading += 1.43; // magnetic declination

  if (remoteClient && AR_tel_s != AzPosPrev)
  {
    AzPosPrev = AR_tel_s;
    // float fl = (float) newPosition / 600;
    byte RA_buf[4];
    float AR_tel_h = (AR_tel_s / 3600.0);
    uint AR_tel_map = (AR_tel_h / 12.0) * 0x80000000;
    RA_buf[0] = (byte)(AR_tel_map & 0xFF);
    RA_buf[1] = (byte)((AR_tel_map >> 8) & 0xFF);
    RA_buf[2] = (byte)((AR_tel_map >> 16) & 0xFF);
    RA_buf[3] = (byte)((AR_tel_map >> 24) & 0xFF);
    byte DEC_buf[4];
    float DEC_tel_deg = (DEC_tel_s / 3600.0);
    long DEC_tel_map = (DEC_tel_deg / 90.0) * 0x40000000;
    DEC_buf[0] = (byte)(DEC_tel_map & 0xFF);
    DEC_buf[1] = (byte)((DEC_tel_map >> 8) & 0xFF);
    DEC_buf[2] = (byte)((DEC_tel_map >> 16) & 0xFF);
    DEC_buf[3] = (byte)((DEC_tel_map >> 24) & 0xFF);
    // https://free-astro.org/images/b/b7/Stellarium_telescope_protocol.txt
    byte info[] = {
        24, 0,                                      // LENGTH (2 bytes,integer): length of the message
        0, 0,                                       // TYPE   (2 bytes,integer): 0
        0, 0, 0, 0, 0, 0, 0, 0,                     // TIME   (8 bytes,integer): current time on the server computer in microseconds since 1970.01.01 UT. Currently unused.
                                                    // 0, 0, 0, 128,
        RA_buf[0], RA_buf[1], RA_buf[2], RA_buf[3], // RA     (4 bytes,unsigned integer): right ascension of the telescope (J2000)
        //  a value of 0x100000000 = 0x0 means 24h=0h,
        //  a value of 0x80000000 means 12h
        DEC_buf[0], DEC_buf[1], DEC_buf[2], DEC_buf[3], // DEC    (4 bytes,signed integer): declination of the telescope (J2000)
                                                        // a value of -0x40000000 (-1073741824) means -90degrees,
                                                        // a value of 0x0 means 0degrees,
                                                        // a value of 0x40000000 (1073741824) means 90degrees
        0, 0, 0, 0                                      // STATUS (4 bytes,signed integer): status of the telescope, currently unused.
                                                        // status = 0 means ok,
                                                        // status < 0 means some error};
    };
    remoteClient.write((byte *)&info, sizeof(info));

    // Serial.println("AR_tel_h: ");
    // Serial.println(AR_tel_h);
    // Serial.println("AR_tel_map: ");
    // Serial.println(AR_tel_map);
    // Serial.print("RA hex: ");
    // int i;
    // for (i = 0; i < sizeof(RA_buf); i++)
    // {
    //   printHex(RA_buf[i]);
    // }
    // Serial.println();
    // Serial.print(".");
    // Serial.write((byte *)&info, sizeof(info));

    // remoteClient.print(txAR);
  }

  // Serial.print("A: ");
  // Serial.print(heading);
  // Serial.println();

  while (Serial.available() > 0)
    if (gps.encode(Serial.read()))
    {
    }

  // picture loop
  u8g2.clearBuffer();
  // draw();
  update_display(heading);
  u8g2.sendBuffer();

  // increase the state
  draw_state++;
  if (draw_state >= 9 * 8)
    draw_state = 0;

  // deley between each page
  // delay(100);
  attendTcpRequests(); // gets priority to prevent timeouts on Skysafari. Measured AVG execution time = 18ms
  yield();

  t_ciclo = millis() - t_ciclo;
  t_ciclo_acumulado = t_ciclo_acumulado + t_ciclo;
}
