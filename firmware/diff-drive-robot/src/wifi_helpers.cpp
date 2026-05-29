#include "wifi_helpers.h"

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
