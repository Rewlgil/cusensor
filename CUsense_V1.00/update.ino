void checkUpdate(){
  String inf;
  Serial.println("checking for update...");
  tft.setTextColor(TFT_WHITE); tft.println("Checking for update...");
  HTTPClient httpClient;
  httpClient.begin( versionINF );
  int httpCode = httpClient.GET();
  if( httpCode == 200 ) {
    String json_code = httpClient.getString();
    Serial.print("JSON Code: "); Serial.println(json_code);
    StaticJsonDocument<140> code;
    DeserializationError error = deserializeJson(code, json_code);
    if (error) {
      Serial.print("deserializeJson() failed with code ");
      Serial.println(error.c_str());
      tft.setTextColor(TFT_RED); tft.print("deserializeJson failed with code "); tft.println(error.c_str());
      return;
    }
    float available_version = code["version"];
    String code_url = code["code_url"];
    Serial.printf("available version: %.2f\ncurrent version: %.2f\n", available_version, current_version);
    tft.setTextColor(TFT_GREEN);
    tft.print("available version: "); tft.println(available_version, 2);
    tft.print("current version: "); tft.println(current_version, 2);
    Serial.print("code_url: "); Serial.println(code_url);
//    tft.print("code_url: "); tft.println(code_url);
    if(round(current_version * 100) < round(available_version * 100)){
      Serial.println("Updating...");
      tft.setTextColor(TFT_BLUE); tft.println("Updating...");
      t_httpUpdate_return ret = ESPhttpUpdate.update(code_url);

      switch(ret) {
        case HTTP_UPDATE_FAILED:
            Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
            tft.setTextColor(TFT_RED);
            tft.print("HTTP_UPDATE_FAILD Error "); tft.println(ESPhttpUpdate.getLastError());
            tft.println(ESPhttpUpdate.getLastErrorString().c_str());  delay(3000);  ESP.restart();
            break;
        case HTTP_UPDATE_NO_UPDATES:
            Serial.println("HTTP_UPDATE_NO_UPDATES");
            tft.setTextColor(TFT_RED);  tft.println("HTTP_UPDATE_NO_UPDATES");
            break;
        case HTTP_UPDATE_OK:
            Serial.println("HTTP_UPDATE_OK");
            tft.setTextColor(TFT_GREEN);  tft.println("HTTP_UPDATE_OK");
            break;
      }
      return;
    }
    else{
      Serial.println("Already on latest version");
      tft.setTextColor(TFT_GREEN);  tft.println("Already on latest version");
      return;
      }
  }
  else{
    Serial.printf("Firmware version check failed, got HTTP response code %d\n", httpCode); 
    tft.setTextColor(TFT_RED);  tft.print("Firmware version check failed, got HTTP response code ");  tft.println(httpCode);
    return;
    }
  httpClient.end();
}
