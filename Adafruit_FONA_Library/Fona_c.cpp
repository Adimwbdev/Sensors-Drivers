  Author: A. Dimitrakopoulos
  Date: 2020
  
//	This is a library for use with the FONA GPS/GSM module but customised to work with the Waveshare_SIM7600X GNSS/GSM module 


#include "Fona_c.h"
#if defined(ESP8266)
  // ESP8266 doesn't have the min and max functions natively available like
  // AVR libc seems to provide.  Include the STL algorithm library to get these.
  // Unfortunately algorithm isn't available in AVR libc so this is ESP8266
  // specific (and likely needed for ARM or other platforms, but they lack
  // software serial and are currently incompatible with the FONA library).
  #include <algorithm>
  using namespace std;
#endif


Adafruit_FONA::Adafruit_FONA(int8_t rst)
{
  _rstpin = rst;

  apn = F("FONAnet");
  apnusername = 0;
  apnpassword = 0;
  mySerial = 0;
  httpsredirect = false;
  useragent = F("FONA");
  ok_reply = F("OK");
  error_reply = F("ERROR");
}

uint8_t Adafruit_FONA::type(void) {
  return _type;
}

boolean Adafruit_FONA::begin(Stream &port) {
  mySerial = &port;

  pinMode(_rstpin, OUTPUT);
  digitalWrite(_rstpin, HIGH);
  delay(10);
  digitalWrite(_rstpin, LOW);
  delay(100);
  digitalWrite(_rstpin, HIGH);

  DEBUG_PRINTLN(F("Attempting to open comm with ATs"));
  // give 7 seconds to reboot
  int16_t timeout = 7000;

  while (timeout > 0) {
    while (mySerial->available()) mySerial->read();
    if (sendCheckReply(F("AT"), ok_reply))
      break;
    while (mySerial->available()) mySerial->read();
    if (sendCheckReply(F("AT"), F("AT")))
      break;
    delay(500);
    timeout-=500;
  }

  if (timeout <= 0) {
#ifdef ADAFRUIT_FONA_DEBUG
    DEBUG_PRINTLN(F("Timeout: No response to AT... last ditch attempt."));
#endif
    sendCheckReply(F("AT"), ok_reply);
    delay(100);
    sendCheckReply(F("AT"), ok_reply);
    delay(100);
    sendCheckReply(F("AT"), ok_reply);
    delay(100);
  }

  // turn off Echo!
  sendCheckReply(F("ATE0"), ok_reply);
  delay(100);
//
//  if (! sendCheckReply(F("ATE0"), ok_reply)) {
//    return false;
//  }

//  // turn on hangupitude
//  sendCheckReply(F("AT+CVHU=0"), ok_reply);
//
//  delay(100);
  flushInput();

  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("ATI");

  mySerial->println("ATI");
  readline(500, true);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

//  //  ####
//
//  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("Availiable Networks");
//
//  mySerial->println("AT+COPS=?");
//  readline(500, true);
//
//  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);
//
////  ####

//Delay to Give time to Connect to the Network
for(int i=0 ; i++ ;i<9){
 delay(15000);
 }


//  ####

  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("Network info");

  mySerial->println("AT+CREG?");
  readline(500, true);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

//  ####

  //  ####

  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("Provider info");

  mySerial->println("AT+CSPN?");
  readline(500, true);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

//  ####

//
//  if (prog_char_strstr(replybuffer, (prog_char *)F("SIM808 R14")) != 0) {
//    _type = FONA808_V2;
//  } else if (prog_char_strstr(replybuffer, (prog_char *)F("SIM808 R13")) != 0) {
//    _type = FONA808_V1;
//  } else if (prog_char_strstr(replybuffer, (prog_char *)F("SIM800 R13")) != 0) {
//    _type = FONA800L;
//  } else if (prog_char_strstr(replybuffer, (prog_char *)F("SIMCOM_SIM5320A")) != 0) {
//    _type = FONA3G_A;
//  } else if (prog_char_strstr(replybuffer, (prog_char *)F("SIMCOM_SIM5320E")) != 0) {
//    _type = FONA3G_E;
//  }
//
//  if (_type == FONA800L) {
//    // determine if L or H
//
//  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("AT+GMM");
//
//    mySerial->println("AT+GMM");
//    readline(500, true);
//
//  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);
//
//
//    if (prog_char_strstr(replybuffer, (prog_char *)F("SIM800H")) != 0) {
//      _type = FONA800H;
//    }
//  }

//####
 DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN("(AT+CGMM) Model Identification:");

    mySerial->println("AT+CGMM");
    readline(500, true);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);
//####


#if defined(FONA_PREF_SMS_STORAGE)
    sendCheckReply(F("AT+CPMS=" FONA_PREF_SMS_STORAGE "," FONA_PREF_SMS_STORAGE "," FONA_PREF_SMS_STORAGE), ok_reply);
#endif

  return true;
}

/********* Serial port ********************************************/
boolean Adafruit_FONA::setBaudrate(uint16_t baud) {
  return sendCheckReply(F("AT+IPREX="), baud, ok_reply);
}

/********* Real Time Clock ********************************************/

boolean Adafruit_FONA::readRTC(uint8_t *year, uint8_t *month, uint8_t *date, uint8_t *hr, uint8_t *min, uint8_t *sec) {
  uint16_t v;
  sendParseReply(F("AT+CCLK?"), F("+CCLK: "), &v, '/', 0);
  *year = v;

  DEBUG_PRINTLN(*year);
}

boolean Adafruit_FONA::enableRTC(uint8_t i) {
  if (! sendCheckReply(F("AT+CLTS="), i, ok_reply))
    return false;
  return sendCheckReply(F("AT&W"), ok_reply);
}

/********* IMEI **********************************************************/

uint8_t Adafruit_FONA::getIMEI(char *imei) {
  getReply(F("AT+GSN"));

  // up to 15 chars
  strncpy(imei, replybuffer, 15);
  imei[15] = 0;

  readline(); // eat 'OK'

  return strlen(imei);
}


/********* NETWORK *******************************************************/

uint8_t Adafruit_FONA::getNetworkStatus(void) {
  uint16_t status;

  if (! sendParseReply(F("AT+CREG?"), F("+CREG: "), &status, ',', 1)) return 0;

  return status;
}


uint8_t Adafruit_FONA::getRSSI(void) {
  uint16_t reply;

  if (! sendParseReply(F("AT+CSQ"), F("+CSQ: "), &reply) ) return 0;

  return reply;
}

/********* SMS **********************************************************/

uint8_t Adafruit_FONA::getSMSInterrupt(void) {
  uint16_t reply;

  if (! sendParseReply(F("AT+CFGRI?"), F("+CFGRI: "), &reply) ) return 0;

  return reply;
}

boolean Adafruit_FONA::setSMSInterrupt(uint8_t i) {
  return sendCheckReply(F("AT+CFGRI="), i, ok_reply);
}

int8_t Adafruit_FONA::getNumSMS(void) {
  uint16_t numsms;

  // get into text mode
  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return -1;

  // ask how many sms are stored
  if (sendParseReply(F("AT+CPMS?"), F(FONA_PREF_SMS_STORAGE ","), &numsms))
    return numsms;
  if (sendParseReply(F("AT+CPMS?"), F("\"SM\","), &numsms))
    return numsms;
  if (sendParseReply(F("AT+CPMS?"), F("\"SM_P\","), &numsms))
    return numsms;
  return -1;
}

// Reading SMS's is a bit involved so we don't use helpers that may cause delays or debug
// printouts!
boolean Adafruit_FONA::readSMS(uint8_t i, char *smsbuff,
			       uint16_t maxlen, uint16_t *readlen) {
  // text mode
  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;

  // show all text mode parameters
  if (! sendCheckReply(F("AT+CSDH=1"), ok_reply)) return false;

  // parse out the SMS len
  uint16_t thesmslen = 0;


  DEBUG_PRINT(F("AT+CMGR="));
  DEBUG_PRINTLN(i);


  //getReply(F("AT+CMGR="), i, 1000);  //  do not print debug!
  mySerial->print(F("AT+CMGR="));
  mySerial->println(i);
  readline(1000); // timeout

  //DEBUG_PRINT(F("Reply: ")); DEBUG_PRINTLN(replybuffer);
  // parse it out...


  DEBUG_PRINTLN(replybuffer);


  if (! parseReply(F("+CMGR:"), &thesmslen, ',', 11)) {
    *readlen = 0;
    return false;
  }

  readRaw(thesmslen);

  flushInput();

  uint16_t thelen = min(maxlen, (uint16_t)strlen(replybuffer));
  strncpy(smsbuff, replybuffer, thelen);
  smsbuff[thelen] = 0; // end the string


  DEBUG_PRINTLN(replybuffer);

  *readlen = thelen;
  return true;
}

// Retrieve the sender of the specified SMS message and copy it as a string to
// the sender buffer.  Up to senderlen characters of the sender will be copied
// and a null terminator will be added if less than senderlen charactesr are
// copied to the result.  Returns true if a result was successfully retrieved,
// otherwise false.
boolean Adafruit_FONA::getSMSSender(uint8_t i, char *sender, int senderlen) {
  // Ensure text mode and all text mode parameters are sent.
  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;
  if (! sendCheckReply(F("AT+CSDH=1"), ok_reply)) return false;


  DEBUG_PRINT(F("AT+CMGR="));
  DEBUG_PRINTLN(i);


  // Send command to retrieve SMS message and parse a line of response.
  mySerial->print(F("AT+CMGR="));
  mySerial->println(i);
  readline(1000);


  DEBUG_PRINTLN(replybuffer);


  // Parse the second field in the response.
  boolean result = parseReplyQuoted(F("+CMGR:"), sender, senderlen, ',', 1);
  // Drop any remaining data from the response.
  flushInput();
  return result;
}
//boolean Adafruit_FONA::sendSMS(char *smsaddr, char *smsmsg) {
//
////  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;
////  if (! sendCheckReply(F("AT"), ok_reply,10000)) return false;
////    if (!sendCheckReply(F("AT+CMGF=1"), error_reply)) DEBUG_PRINTLN(F("Kolise i fasi"));
//}
boolean Adafruit_FONA::sendSMS(char *smsaddr, char *smsmsg) {
  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;
//  if (! sendCheckReply(F("AT+CMGF=1"), error_reply)) return false;
  DEBUG_PRINT(F("Till here!"));

  char sendcmd[30] = "AT+CMGS=\"";

  strncpy(sendcmd+9, smsaddr, 30-9-2);  // 9 bytes beginning, 2 bytes for close quote + null
  sendcmd[strlen(sendcmd)] = '\"';

  if (! sendCheckReply(sendcmd, F("> "))) return false;

  DEBUG_PRINT(F("> ")); DEBUG_PRINTLN(smsmsg);

  mySerial->println(smsmsg);
  mySerial->println();
  mySerial->write(0x1A);

  DEBUG_PRINTLN("^Z");

  if ( (_type == FONA3G_A) || (_type == FONA3G_E) ) {
    // Eat two sets of CRLF
    readline(200);
    //DEBUG_PRINT("Line 1: "); DEBUG_PRINTLN(strlen(replybuffer));
    readline(200);
    //DEBUG_PRINT("Line 2: "); DEBUG_PRINTLN(strlen(replybuffer));
  }
  readline(10000); // read the +CMGS reply, wait up to 10 seconds!!!
  //DEBUG_PRINT("Line 3: "); DEBUG_PRINTLN(strlen(replybuffer));
  if (strstr(replybuffer, "+CMGS") == 0) {
    return false;
  }
  readline(1000); // read OK
  //DEBUG_PRINT("* "); DEBUG_PRINTLN(replybuffer);

  if (strcmp(replybuffer, "OK") != 0) {
    return false;
  }

  return true;
}

boolean Adafruit_FONA::sendSMS(char *smsaddr, String smsmsg) {
  if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;
//  if (! sendCheckReply(F("AT+CMGF=1"), error_reply)) return false;
  DEBUG_PRINT(F("Till here!"));

  char sendcmd[30] = "AT+CMGS=\"";

  strncpy(sendcmd+9, smsaddr, 30-9-2);  // 9 bytes beginning, 2 bytes for close quote + null
  sendcmd[strlen(sendcmd)] = '\"';

  if (! sendCheckReply(sendcmd, F("> "))) return false;

  DEBUG_PRINT(F("> ")); DEBUG_PRINTLN(smsmsg);

  mySerial->println(smsmsg);
  mySerial->println();
  mySerial->write(0x1A);

  DEBUG_PRINTLN("^Z");

  if ( (_type == FONA3G_A) || (_type == FONA3G_E) ) {
    // Eat two sets of CRLF
    readline(200);
    //DEBUG_PRINT("Line 1: "); DEBUG_PRINTLN(strlen(replybuffer));
    readline(200);
    //DEBUG_PRINT("Line 2: "); DEBUG_PRINTLN(strlen(replybuffer));
  }
  readline(10000); // read the +CMGS reply, wait up to 10 seconds!!!
  //DEBUG_PRINT("Line 3: "); DEBUG_PRINTLN(strlen(replybuffer));
  if (strstr(replybuffer, "+CMGS") == 0) {
    return false;
  }
  readline(1000); // read OK
  //DEBUG_PRINT("* "); DEBUG_PRINTLN(replybuffer);

  if (strcmp(replybuffer, "OK") != 0) {
    return false;
  }

  return true;
}


boolean Adafruit_FONA::deleteSMS(uint8_t i) {
    if (! sendCheckReply(F("AT+CMGF=1"), ok_reply)) return false;
  // read an sms
  char sendbuff[12] = "AT+CMGD=000";
  sendbuff[8] = (i / 100) + '0';
  i %= 100;
  sendbuff[9] = (i / 10) + '0';
  i %= 10;
  sendbuff[10] = i + '0';

  return sendCheckReply(sendbuff, ok_reply, 2000);

}


/********* TIME **********************************************************/

boolean Adafruit_FONA::enableNetworkTimeSync(boolean onoff) {
  if (onoff) {
    if (! sendCheckReply(F("AT+CLTS=1"), ok_reply))
      return false;
  } else {
    if (! sendCheckReply(F("AT+CLTS=0"), ok_reply))
      return false;
  }

  flushInput(); // eat any 'Unsolicted Result Code'

  return true;
}

boolean Adafruit_FONA::enableNTPTimeSync(boolean onoff, FONAFlashStringPtr ntpserver) {
  if (onoff) {
    if (! sendCheckReply(F("AT+CNTPCID=1"), ok_reply))
      return false;

    mySerial->print(F("AT+CNTP=\""));
    if (ntpserver != 0) {
      mySerial->print(ntpserver);
    } else {
      mySerial->print(F("pool.ntp.org"));
    }
    mySerial->println(F("\",0"));
    readline(FONA_DEFAULT_TIMEOUT_MS);
    if (strcmp(replybuffer, "OK") != 0)
      return false;

    if (! sendCheckReply(F("AT+CNTP"), ok_reply, 10000))
      return false;

    uint16_t status;
    readline(10000);
    if (! parseReply(F("+CNTP:"), &status))
      return false;
  } else {
    if (! sendCheckReply(F("AT+CNTPCID=0"), ok_reply))
      return false;
  }

  return true;
}

boolean Adafruit_FONA::getTime(char *buff, uint16_t maxlen) {
  getReply(F("AT+CCLK?"), (uint16_t) 10000);
  if (strncmp(replybuffer, "+CCLK: ", 7) != 0)
    return false;

  char *p = replybuffer+7;
  uint16_t lentocopy = min(maxlen-1, (int)strlen(p));
  strncpy(buff, p, lentocopy+1);
  buff[lentocopy] = 0;

  readline(); // eat OK

  return true;
}

/********* GPS **********************************************************/

boolean Adafruit_FONA::enableGPS(boolean onoff) {
  uint16_t state;


  sendCheckReply("AT+CGPS?", F("> "));
  DEBUG_PRINT(F("Reply_Buffer")); DEBUG_PRINTLN(replybuffer);
  sendCheckReply("AT+CGPS=1,1", ok_reply);



//  // first check if its already on or off
//
//  if (_type == FONA808_V2) {
//    if (! sendParseReply(F("AT+CGNSPWR?"), F("+CGNSPWR: "), &state) )
//      return false;
//  } else {
//    if (! sendParseReply(F("AT+CGPSPWR?"), F("+CGPSPWR: "), &state))
//      return false;
//  }
//
//  if (onoff && !state) {
//    if (_type == FONA808_V2) {
//      if (! sendCheckReply(F("AT+CGNSPWR=1"), ok_reply))  // try GNS command
//	return false;
//    } else {
//      if (! sendCheckReply(F("AT+CGPSPWR=1"), ok_reply))
//	return false;
//    }
//  } else if (!onoff && state) {
//    if (_type == FONA808_V2) {
//      if (! sendCheckReply(F("AT+CGNSPWR=0"), ok_reply)) // try GNS command
//	return false;
//    } else {
//      if (! sendCheckReply(F("AT+CGPSPWR=0"), ok_reply))
//	return false;
//    }
//  }
  return true;
}

/**************************GPS positoning (Custom)**************************/
bool Adafruit_FONA::GPSPositioning(){
//    boolean answer = false;
//    uint16_t state;
//    uint16_t *gps;
//    uint16_t v;
//    char a[20];
//    char lege[20]="CGPSINFO: ,,,,,,,,";
//    a[0]=0;
//
//        DEBUG_PRINTLN("Prin");
//        getReply(F("AT+CGPSINFO"));
//
//        for(int i=1;i<20;i++){
//        a[i] = replybuffer[i];
//        }
//
//        if(strcmp(a,"CGPSINFO: ,,,,,,,,")==0) DEBUG_PRINTLN("E re dikie mou");
//
//
//    return true;
//
//    uint8_t answer = 0;
    bool answer = false;
    bool RecNull = true;
    int i = 0;
    char callerIDbuffer[32] = {'6','9','7','2','4','5','2','4','9','8','\0'};
//    String smsbuffer;


//    memset(RecMessage, '\0', 200);    // Initialize the string

//     while(RecNull)
//    {
//        answer =  getReply("AT+CGPSINFO", "+CGPSINFO: ", 1000);
        answer =  sendCheckReply(F("AT+CGPSINFO"), F("+CGPSINFO: "));    // start GPS session, standalone mode
        Serial.print("Meta tin sendparsereply");
//        answer =  sendParseReply(F("AT+CGPSINFO"), F("+CGPSINFO: "),v);
//        DEBUG_PRINT("H answer einai:");
//        DEBUG_PRINTLN(answer);
//        DEBUG_PRINT("To replyBuffer einai:");
//        DEBUG_PRINTLN(replybuffer);

//        Serial.println("ReblyBuffer einai");
//        Serial.print(replybuffer);
//        if (strstr(replybuffer,",,,,,,,,") != NULL){
//        Serial.println("Ola koble");
//        }


        while(strstr(replybuffer,",,,,,,,,") != NULL){
        delay(2000); // Waiting to find shutterlite
        sendCheckReply(F("AT+CGPSINFO"), F("+CGPSINFO: "));
        Serial.println("To replyBuffer tha einai:");
        Serial.print(replybuffer);
        Serial.println("meta to loop");
        }

        String smsbuffer = replybuffer;
        Serial.println("To smsbuffer einai:");
        Serial.println(smsbuffer);
        Serial.println("To megethos tis smsbuffer einai:");
        Serial.println(smsbuffer.length());

        sendSMS(callerIDbuffer,smsbuffer);

//        if (answer == 1)
//        {
//            answer = 0;
//            while(Serial.available() == 0);
//             this loop reads the data of the GPS
//            do{
//                 if there are data in the UART input buffer, reads it and checks for the asnwer
//                if(Serial.available() > 0){
//                    RecMessage[i] = Serial.read();
//                    i++;
//                     check if the desired answer (OK) is in the response of the module
//                    Serial.println("To RecMessage einai:");
//                    Serial.println(RecMessage);
//                    if (strstr(RecMessage, "OK") != NULL)
//                    {
//                        answer = 1;
//                    }
//                }
//            }while(answer == 0);    // Waits for the asnwer with time out
//
//            RecMessage[i] = '\0';
//            Serial.print(RecMessage);
//            Serial.print("\n");
//
//            if (strstr(RecMessage, ",,,,,,,,") != NULL)
//            {
//                memset(RecMessage, '\0', 200);    // Initialize the string
//                i = 0;
//                answer = 0;
//                delay(1000);
//
//            }
//            else
//            {
//                RecNull = false;
//                sendATcommand("AT+CGPS=0", "OK:", 1000);
//                sendCheckReply("AT+CGPS=0", "OK:");
//
//            }
//        }
//        else
//        {
//            Serial.print("error \n");
//            return false;
//        }
//        delay(2000);





}


/********* TCP (Custom) *********************************************/

boolean Adafruit_FONA::TCPconnect(char *server, char *port) {

uint8_t status;
uint16_t buff;
if (sendCheckReply(F("AT+NETOPEN"), ok_reply))
mySerial->println(F("NET IS OPEN"));
char cmnd[42]= "AT+CIPOPEN=1,\"TCP\",\"";
strncpy(cmnd+20,server,11);
//strncpy(cmnd+20,server,10);
char num = strlen(cmnd);
strncpy(cmnd+num,"\",",sizeof(server));
num = strlen(cmnd);
strncpy(cmnd+num,port,sizeof(port)+2);
buff = getReply(cmnd, 100000000);
mySerial->println(buff);
//if (! sendParseReply(cmnd, F("+CIPOPEN: "), &status, ',', 0)) return 0;

return 1;

}

boolean Adafruit_FONA::TCPgpsHello(void) {

uint8_t *buff;
char cmnd[15] = "AT+CIPSEND=1,6";
char mssg[6] = "Hello!";
//if (sendCheckReply(F("AT+CIPSEND=1,5"), ok_reply))
buff = getReply(cmnd, 1000);
buff = getReply(mssg,1000);
mySerial->println(*buff);

}

boolean Adafruit_FONA::TCPgps(void) {

uint8_t *buff;
bool answer = false;

answer =  sendCheckReply(F("AT+CGPSINFO"), F("+CGPSINFO: "));
while(strstr(replybuffer,",,,,,,,,") != NULL){
        delay(2000); // Waiting to find shutterlite
        sendCheckReply(F("AT+CGPSINFO"), F("+CGPSINFO: "));
        Serial.println("To replyBuffer tha einai:");
        Serial.print(replybuffer);
        Serial.println("meta to loop");
        }

        String smsbuffer = replybuffer;
char cmnd[16] = "AT+CIPSEND=1,71";
//char mssg[71] = replybuffer;
char mssg[57] = "Hello motherfucker How are you today with all this shit??";
//if (sendCheckReply(F("AT+CIPSEND=1,5"), ok_reply))
//char mssg[10] = "Hello you";
Serial.println("Edw to vlepei akoma to replybuffer->");
Serial.println(replybuffer);
Serial.println("kai to length einai:");
Serial.println(sizeof(replybuffer));
buff = getReply(cmnd, 1000);
mySerial->println(smsbuffer);
//buff = getReply(mssg,5000);
//buff = getReply(replybuffer,5000);
mySerial->println(*buff);

}



boolean Adafruit_FONA::TCPgps(char* mssg, bool flag) {
uint16_t state;
uint8_t *buff;




char cmnd[15] = "AT+CIPSEND=1,5";
//char mssg[6] = "Hello!";
char num = strlen(cmnd);
//if (sendCheckReply(F("AT+CIPSEND=1,5"), ok_reply))
buff = getReply(cmnd, 1000);
//sendParseReply(F("AT+CGPSINFO"), F("+CGPSINFO: "), &state);
//DEBUG_PRINTLN("ReplyBuffer->");
//DEBUG_PRINTLN(replybuffer);
buff = getReply(mssg,1000);
//mySerial->println(*buff);


return true;
}

boolean Adafruit_FONA::TCPclose(void) {
if (sendCheckReply(F("AT+CIPCLOSE=1"), ok_reply))
mySerial->println(F("NET IS CLOSED"));

return true;
}




/********* HELPERS *********************************************/

boolean Adafruit_FONA::expectReply(FONAFlashStringPtr reply,
                                   uint16_t timeout) {
  readline(timeout);

  DEBUG_PRINT(F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}

/********* LOW LEVEL *******************************************/

inline int Adafruit_FONA::available(void) {
  return mySerial->available();
}

inline size_t Adafruit_FONA::write(uint8_t x) {
  return mySerial->write(x);
}

inline int Adafruit_FONA::read(void) {
  return mySerial->read();
}

inline int Adafruit_FONA::peek(void) {
  return mySerial->peek();
}

inline void Adafruit_FONA::flush() {
  mySerial->flush();
}

void Adafruit_FONA::flushInput() {
    // Read all available serial input to flush pending data.
    uint16_t timeoutloop = 0;
    while (timeoutloop++ < 40) {
        while(available()) {
            read();
            timeoutloop = 0;  // If char was received reset the timer
        }
        delay(1);
    }
}

uint16_t Adafruit_FONA::readRaw(uint16_t b) {
  uint16_t idx = 0;

  while (b && (idx < sizeof(replybuffer)-1)) {
    if (mySerial->available()) {
      replybuffer[idx] = mySerial->read();
      idx++;
      b--;
    }
  }
  replybuffer[idx] = 0;

  return idx;
}

uint8_t Adafruit_FONA::readline(uint16_t timeout, boolean multiline) {
  uint16_t replyidx = 0;

  while (timeout--) {
    if (replyidx >= 254) {
      //DEBUG_PRINTLN(F("SPACE"));
      break;
    }

    while(mySerial->available()) {
      char c =  mySerial->read();
      if (c == '\r') continue;
      if (c == 0xA) {
        if (replyidx == 0)   // the first 0x0A is ignored
          continue;

        if (!multiline) {
          timeout = 0;         // the second 0x0A is the end of the line
          break;
        }
      }
      replybuffer[replyidx] = c;
      //DEBUG_PRINT(c, HEX); DEBUG_PRINT("#"); DEBUG_PRINTLN(c);
      replyidx++;
    }

    if (timeout == 0) {
      //DEBUG_PRINTLN(F("TIMEOUT"));
      break;
    }
    delay(1);
  }
  replybuffer[replyidx] = 0;  // null term
  return replyidx;
}

uint8_t Adafruit_FONA::getReply(char *send, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN(send);

  mySerial->println(send);

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

uint8_t Adafruit_FONA::getReply(FONAFlashStringPtr send, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINTLN(send);


  mySerial->println(send);

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

// Send prefix, suffix, and newline. Return response (and also set replybuffer with response).
uint8_t Adafruit_FONA::getReply(FONAFlashStringPtr prefix, char *suffix, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINT(prefix); DEBUG_PRINTLN(suffix);


  mySerial->print(prefix);
  mySerial->println(suffix);

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

// Send prefix, suffix, and newline. Return response (and also set replybuffer with response).
uint8_t Adafruit_FONA::getReply(FONAFlashStringPtr prefix, int32_t suffix, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINT(prefix); DEBUG_PRINTLN(suffix, DEC);


  mySerial->print(prefix);
  mySerial->println(suffix, DEC);

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

// Send prefix, suffix, suffix2, and newline. Return response (and also set replybuffer with response).
uint8_t Adafruit_FONA::getReply(FONAFlashStringPtr prefix, int32_t suffix1, int32_t suffix2, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINT(prefix);
  DEBUG_PRINT(suffix1, DEC); DEBUG_PRINT(','); DEBUG_PRINTLN(suffix2, DEC);


  mySerial->print(prefix);
  mySerial->print(suffix1);
  mySerial->print(',');
  mySerial->println(suffix2, DEC);

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

// Send prefix, ", suffix, ", and newline. Return response (and also set replybuffer with response).
uint8_t Adafruit_FONA::getReplyQuoted(FONAFlashStringPtr prefix, FONAFlashStringPtr suffix, uint16_t timeout) {
  flushInput();


  DEBUG_PRINT(F("\t---> ")); DEBUG_PRINT(prefix);
  DEBUG_PRINT('"'); DEBUG_PRINT(suffix); DEBUG_PRINTLN('"');


  mySerial->print(prefix);
  mySerial->print('"');
  mySerial->print(suffix);
  mySerial->println('"');

  uint8_t l = readline(timeout);

  DEBUG_PRINT (F("\t<--- ")); DEBUG_PRINTLN(replybuffer);

  return l;
}

boolean Adafruit_FONA::sendCheckReply(char *send, char *reply, uint16_t timeout) {

  if (! getReply(send, timeout) )
	  return false;
//  DEBUG_PRINTLN("TARAVONIA");
  for (uint8_t i=0; i<strlen(replybuffer); i++) {
  DEBUG_PRINT(replybuffer[i], HEX); DEBUG_PRINT(" ");
//  DEBUG_PRINT(replybuffer[i]); DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();

  for (uint8_t i=0; i<strlen(reply); i++) {
    DEBUG_PRINT(reply[i], HEX); DEBUG_PRINT(" ");
  }
  DEBUG_PRINTLN();

  return (strcmp(replybuffer, reply) == 0);
}

boolean Adafruit_FONA::sendCheckReply(FONAFlashStringPtr send, FONAFlashStringPtr reply, uint16_t timeout) {
	int a = 5;
    DEBUG_PRINT("sendCheckReply:");

	if (! getReply(send, timeout) )
		return false;
//	DEBUG_PRINT("replybuffer:");
//	DEBUG_PRINTLN(replybuffer);
    DEBUG_PRINT("String comparison:");
	a = prog_char_strcmp(replybuffer, (prog_char*)reply);
    DEBUG_PRINTLN(replybuffer);
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}

boolean Adafruit_FONA::sendCheckReply(char* send, FONAFlashStringPtr reply, uint16_t timeout) {

  if (! getReply(send, timeout) )
	  return false;
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}


// Send prefix, suffix, and newline.  Verify FONA response matches reply parameter.
boolean Adafruit_FONA::sendCheckReply(FONAFlashStringPtr prefix, char *suffix, FONAFlashStringPtr reply, uint16_t timeout) {


   getReply(prefix, suffix, timeout);
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}

// Send prefix, suffix, and newline.  Verify FONA response matches reply parameter.
boolean Adafruit_FONA::sendCheckReply(FONAFlashStringPtr prefix, int32_t suffix, FONAFlashStringPtr reply, uint16_t timeout) {


  getReply(prefix, suffix, timeout);
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}

// Send prefix, suffix, suffix2, and newline.  Verify FONA response matches reply parameter.
boolean Adafruit_FONA::sendCheckReply(FONAFlashStringPtr prefix, int32_t suffix1, int32_t suffix2, FONAFlashStringPtr reply, uint16_t timeout) {


  getReply(prefix, suffix1, suffix2, timeout);
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}

// Send prefix, ", suffix, ", and newline.  Verify FONA response matches reply parameter.
boolean Adafruit_FONA::sendCheckReplyQuoted(FONAFlashStringPtr prefix, FONAFlashStringPtr suffix, FONAFlashStringPtr reply, uint16_t timeout) {


  getReplyQuoted(prefix, suffix, timeout);
  return (prog_char_strcmp(replybuffer, (prog_char*)reply) == 0);
}


boolean Adafruit_FONA::parseReply(FONAFlashStringPtr toreply,
          uint16_t *v, char divider, uint8_t index) {
  char *p = prog_char_strstr(replybuffer, (prog_char*)toreply);  // get the pointer to the voltage
  if (p == 0) return false;
  p+=prog_char_strlen((prog_char*)toreply);
  //DEBUG_PRINTLN(p);
  for (uint8_t i=0; i<index;i++) {
    // increment dividers
    p = strchr(p, divider);
    if (!p) return false;
    p++;
    //DEBUG_PRINTLN(p);

  }
  *v = atoi(p);

  return true;
}

boolean Adafruit_FONA::parseReply(FONAFlashStringPtr toreply,
          char *v, char divider, uint8_t index) {
  uint8_t i=0;
  char *p = prog_char_strstr(replybuffer, (prog_char*)toreply);
  if (p == 0) return false;
  p+=prog_char_strlen((prog_char*)toreply);

  for (i=0; i<index;i++) {
    // increment dividers
    p = strchr(p, divider);
    if (!p) return false;
    p++;
  }

  for(i=0; i<strlen(p);i++) {
    if(p[i] == divider)
      break;
    v[i] = p[i];
  }

  v[i] = '\0';

  return true;
}

// Parse a quoted string in the response fields and copy its value (without quotes)
// to the specified character array (v).  Only up to maxlen characters are copied
// into the result buffer, so make sure to pass a large enough buffer to handle the
// response.
boolean Adafruit_FONA::parseReplyQuoted(FONAFlashStringPtr toreply,
          char *v, int maxlen, char divider, uint8_t index) {
  uint8_t i=0, j;
  // Verify response starts with toreply.
  char *p = prog_char_strstr(replybuffer, (prog_char*)toreply);
  if (p == 0) return false;
  p+=prog_char_strlen((prog_char*)toreply);

  // Find location of desired response field.
  for (i=0; i<index;i++) {
    // increment dividers
    p = strchr(p, divider);
    if (!p) return false;
    p++;
  }

  // Copy characters from response field into result string.
  for(i=0, j=0; j<maxlen && i<strlen(p); ++i) {
    // Stop if a divier is found.
    if(p[i] == divider)
      break;
    // Skip any quotation marks.
    else if(p[i] == '"')
      continue;
    v[j++] = p[i];
  }

  // Add a null terminator if result string buffer was not filled.
  if (j < maxlen)
    v[j] = '\0';

  return true;
}

boolean Adafruit_FONA::sendParseReply(FONAFlashStringPtr tosend,
				      FONAFlashStringPtr toreply,
				      uint16_t *v, char divider, uint8_t index) {

  getReply(tosend);


  if (! parseReply(toreply, v, divider, index)) return false;
  readline(); // eat 'OK'
  				      Serial.print("meta tin readline");

  return true;
}

