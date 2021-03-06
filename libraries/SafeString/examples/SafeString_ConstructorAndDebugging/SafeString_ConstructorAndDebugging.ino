
/*
  SafeString constructors and assignments
  Examples of how to create SafeStrings and how to assign SafeStrings from other data types

  by Matthew Ford
  Mods Copyright(c)2020 Forward Computing and Control Pty. Ltd.
  Modified from String Examples by Tom Igoe
  This example code is in the public domain.

  www.forward.com.au/pfod/ArduinoProgramming/SafeString/index.html
*/

#include "SafeString.h"

void setup() {
  // Open serial communications and wait a few seconds
  Serial.begin(9600);
  for (int i = 10; i > 0; i--) {
    Serial.print(' '); Serial.print(i);
    delay(500);
  }
  Serial.println();
  Serial.println(F("SafeString Constructor and Debugging"));
  Serial.println(F("// Set Stream to send SafeString error messages and debug( ) output to"));
  Serial.println(F("SafeString::setOutput(Serial); // defaults to verbose error messages"));
  SafeString::setOutput(Serial); // enable error messages and debug() output

  Serial.println();
  Serial.println(F("Use the createSafeString(  ); macro to create SafeStrings"));
  Serial.println(F(" This macro creates the char array and wraps it in a SafeString object and sets its name for debugging"));
  createSafeString(stringOne, 26);
  Serial.println();
  Serial.println(F("When the C++ pre-processor processes createSafeString(stringOne,26) it generates the following two lines"));
  Serial.println(F("char stringOne_SAFEBUFFER[26+1];"));
  Serial.println(F("// create the char array to hold strings of upto length 26. Plus 1 for terminating '\\0'"));
  Serial.println(F("SafeString stringOne(sizeof(stringOne_SAFEBUFFER),stringOne_SAFEBUFFER,\"stringOne\", \"\");"));
  Serial.println(F("// wrap it in an SafeString object and set its name for debugging"));
  Serial.println();
  Serial.println(F("Once the SafeString has been created you can assign values to it, e.g."));
  Serial.println(F(" stringOne = \"Hello SafeString!!\";"));
  stringOne = "Hello SafeString!!";

  Serial.println();
  Serial.println(F("You can also create a SafeString and set its initial value, e.g. "));
  Serial.println(F(" createSafeString(stringTwo,26,\"Hello SafeString Two\");"));
  Serial.println(F("Initial values have to be explicit strings e.g.  \" ... \""));
  createSafeString(stringTwo, 26, "Hello SafeString Two");
  Serial.println();

  Serial.println(F("If setOutput() has been called (as it has been in this sketch), then the debug() method will output the details of the SafeString, e.g."));
  Serial.println(F(" stringOne.debug();"));
  stringOne.debug();
  Serial.println(F(" stringTwo.debug();"));
  stringTwo.debug();
  Serial.println();

  Serial.println(F(" debug(false) will suppress the outputing the contents of the SafeString, e.g."));
  Serial.println(F(" stringOne.debug(false);"));
  stringOne.debug(false);
  Serial.println();


  Serial.println(F(" You can add a debug title, either \"...\" or F(\"...\") or a SafeString, to the debug output, e.g. "));
  Serial.println(F(" stringOne.debug(F(\"After assigning Hello SafeString!!\"));\"));"));
  stringOne.debug(F("After assigning Hello SafeString!!"));
  Serial.println();


  Serial.println(F(" The title can be another SafeString"));
  createSafeString(stitle, 20);
  int count = 5;
  stitle = "After ";
  stitle += count;
  stitle += " loops";
  Serial.println(F(" Having built up this SafeString stitle "));
  stitle.debug();
  Serial.println(F(" stringOne.debug(stitle);"));
  stringOne.debug(stitle);
  Serial.println();

  Serial.println(F("For Examples of assignment see the SafeStringAssignmentAndAppendOperator example."));
  Serial.println(F("Also see the SafeStringPrint example for extra format control."));
  Serial.println();

  Serial.println(F("Error checking..."));
  Serial.println();
  Serial.println(F("SafeString has extensive error checking which avoids crashing your program"));
  Serial.println(F("  and, if setOutput() has been called, it will output detailed error messages "));
  Serial.println();


  Serial.println(F("Create a SafeString that can hold 2 chars and try to set an initial value of abc  e.g. "));
  Serial.println(F(" createSafeString(str1, 2, \"abc\");"));
  Serial.println(F(" If setOutput( ) has been called, then with the default verbose error messaages you will get."));
  createSafeString(str1, 2, "abc");
  Serial.println(F("NOTE: The SafeString is still valid but just empty since the requested initial value would not fit."));
  Serial.println();


  Serial.println(F("Set verbose to false to get more compact error messages"));
  Serial.println(F(" SafeString::setVerbose(false);"));
  SafeString::setVerbose(false);
  Serial.println(F("and again try"));
  Serial.println(F(" createSafeString(str2, 2, \"abc\");"));
  createSafeString(str2, 2, "abc");
  Serial.println();

  Serial.println(F("Set verbose back to true"));
  Serial.println(F(" SafeString::setVerbose(true);"));
  SafeString::setVerbose(true);
  Serial.println();

  Serial.println(F(" Error messages and debug() output are off by default until you call SafeString::setOutput( ); "));
  Serial.println(F("    stringOne.debug(); will not produce any output until you call SafeString::setOutput( )"));
  Serial.println(F(" You can turn off all error messages and debug() output using SafeString::turnOutputOff();"));
  Serial.println(F(" but the error checks are still performed and keep the SafeStrings valid"));
  Serial.println(F(" You can remove all the error message from the code by commenting out #define SSTRING_DEBUG at the top of the SafeString.h file"));
  Serial.println(F("   The error checks are still performed and keep the SafeStrings valid"));
  Serial.println(F("   If #define SSTRING_DEBUG is commented out,  debug( ) still works, but without the variable name"));

  Serial.println();
  Serial.println(F(" Unit tests for SafeString constructor."));
  Serial.println(F(" The SafeString constructor is not normally used as createSafeString( ) is easier and defines the buffer for you."));
  char *nullBuffer = NULL;
  Serial.println(F("Check passing NULL pointer for the buffer to SafeString object constructor, prints error msg but does not blow up program."));
  Serial.println(F("SafeString testStr1(4, nullBuffer, \"testValue\");"));
  SafeString testStr1(4, nullBuffer, "testValue");
  Serial.println(F("The resulting testStr1 is valid, but with zero capacity."));
  testStr1.debug(F("testStr1.debug(); => "));
  Serial.println();

  Serial.println(F("Check passing NULL initial value."));
  char buffer[4];
  Serial.println(F("SafeString testStr2(4, buffer, NULL);"));
  SafeString testStr2(4, buffer, NULL);
  testStr2.debug(F("testStr2.debug(); => "));

}

void loop() {

}
