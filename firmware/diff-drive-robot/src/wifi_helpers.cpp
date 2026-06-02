#include "wifi_helpers.h"

#include <ArduinoOTA.h>

#include "WiFi.h"

void wifi_setup(const char *ssid, const char *passphrase)
{
  WiFi.begin(ssid, passphrase);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nConnected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// run wifi_setup() and ota_setup() in setup() and then call ota_handle() in loop() to enable OTA updates
void ota_setup()
{
  // Configure OTA Device Name
  ArduinoOTA.setHostname("diff-bot-esp32");

  // Optional: Add basic security passwords for wireless flashes
  // ArduinoOTA.setPassword("diffbot123");

  ArduinoOTA.onStart([]()
                     {
                       // Safe-fail: Stop your motors right here before code overwriting begins!
                       // analogWrite(PWM_LEFT, 0);
                       // analogWrite(PWM_RIGHT, 0);
                       // digitalWrite(STBY_PIN, LOW);
                     });

  ArduinoOTA.begin();
}

void ota_handle()
{
  ArduinoOTA.handle();
}
