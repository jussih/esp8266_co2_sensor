  #ifdef DEBUG
    #define DEBUG4(x,y,z,a) Serial.print(x);Serial.print(" ");Serial.print(y);Serial.print(" ");Serial.print(z);Serial.print(" ");Serial.println (a)
    #define DEBUG3(x,y,z) Serial.print(x);Serial.print(" ");Serial.print(y);Serial.print(" ");Serial.println (z)
    #define DEBUG2(y,z) Serial.print(y);Serial.print(" ");Serial.println(z)
    #define DEBUG(z) Serial.println(z)
    #define DEBUGln  Serial.println("")
  #else
    #define DEBUG4(x,y,z,a)
    #define DEBUG3(x,y,z)
    #define DEBUG2(x,y)
    #define DEBUG(x)
    #define DEBUGln
  #endif
