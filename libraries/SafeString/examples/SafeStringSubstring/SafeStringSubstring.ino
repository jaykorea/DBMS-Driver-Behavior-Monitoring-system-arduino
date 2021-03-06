/*
  SafeString substring()
  Examples of SafeString substring

  by Matthew Ford
  Copyright(c)2020 Forward Computing and Control Pty. Ltd.
  This example code is in the public domain.

  www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
*/

#include "SafeString.h"
createSafeString(stringOne, 30, "Content-Type: text/html");

void setup() {
  // Open serial communications and wait a few seconds
  Serial.begin(9600);
  for (int i = 10; i > 0; i--) {
    Serial.print(' '); Serial.print(i);
    delay(500);
  }
  Serial.println();

  Serial.println(F("SafeString substring() usage"));
  Serial.println(F("SafeString::setOutput(Serial); // verbose == true"));
  // see the SafeString_ConstructorAndDebugging example for debugging settings
  SafeString::setOutput(Serial); // enable full debugging error msgs
  Serial.println();

  stringOne.debug();
  Serial.println();
  createSafeString(substr, 6);
  substr.debug(F("createSafeString(substr, 6); => "));
  stringOne.substring(substr, 19);
  substr.debug(F("stringOne.substring(substr,19); => "));
  if (substr == "html") {
    Serial.println("substr is 'html'");
  }
  Serial.println();

  stringOne.substring(substr, 14, 17);
  substr.debug(F("stringOne.substring(substr,14,17); => "));
  if (stringOne.substring(substr, 14, 17) == "text") { // substring return reference to result SafeString
    Serial.println("substr is 'text'");
  }
  Serial.println();

  Serial.println(F("Error checking.."));
  Serial.println();

  Serial.println(F("stringOne.substring(substr,23);  beginIdx == length() is OK "));
  stringOne.substring(substr, 23);
  substr.debug();
  Serial.println();

  Serial.println(F("stringOne.substring(substr,22,23);  endIdx == length() is OK "));
  stringOne.substring(substr, 22, 23);
  substr.debug();
  Serial.println();

  Serial.println(F("stringOne.substring(substr,23,23);  beginIdx == endIdx == length() is OK "));
  stringOne.substring(substr, 23, 23);
  substr.debug();
  Serial.println();

  Serial.println(F("stringOne.substring(substr,19,19);  beginIdx == endIdx is OK "));
  stringOne.substring(substr, 19, 19);
  substr.debug();
  Serial.println();


  Serial.println(F("stringOne.substring(substr,19,5);"));
  stringOne.substring(substr, 19, 5);
  Serial.println(F(" contents of result substring are unchanged on errors"));
  substr.debug(F(" result: "));
  Serial.println();

  Serial.println(F("stringOne.substring(substr,8);"));
  stringOne.substring(substr, 8);
  Serial.println();

  Serial.println(F("stringOne.substring(substr,0); works on an empty stringOne"));
  Serial.println(F("stringOne = \"\";"));
  stringOne = "";
  stringOne.substring(substr, 0);
  substr.debug(F("stringOne.substring(substr,0); => "));

}

void loop() {
}
