#include <WiFi.h>
#include "HttpServer.h"
#include "ArduinoJson.h"

String prepareHtmlPage(String payload)
{
  String htmlPage =
     String("HTTP/1.1 200 OK\r\n") +
            "Content-Type: application/json\r\n" +
            "\r\n"+
            payload +
            "\r\n";
  return htmlPage;
}

JsonObject InitializeData(){
  //Alocar memoria para o objeto
  const size_t CAPACITY = JSON_OBJECT_SIZE(6);
  StaticJsonDocument<CAPACITY> doc;

  // Criar objeto
  JsonObject Data = doc.to<JsonObject>();
  JsonArray Aceleration = Data.createNestedArray("Aceleration");
  Data["temperature"] = 0;
  Data["bmp_instant"] = 0;
  Data["bmp_med"] = 0;
  Data["satuaration_med"] = 0;
  Data["valid_satuaration"] = 0;
  Data["batery"] = 0;
  return Data;  
}

void manageRequest(String target, WiFiClient *client, String payload){
  while (client->connected())
    {
      // read line by line what the client (web browser) is requesting
      if (client->available())
      {
        String line = client->readStringUntil('\r');
        //Serial.print(line);
        // wait for end of client's request, that is marked with an empty line
        if (line.length() == 1 && line[0] == '\n')
        {
          client->println(prepareHtmlPage(payload));
          
          delay(20);

          //Serial.println("Client disonnected");
          //Serial.println(prepareHtmlPage(payload));
          break;
        }
      }
      else{
        //Serial.println("Conection lost!");
        break;
      }
    }
    return;
  }
