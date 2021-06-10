String id_microcontroler = "B";

String first_value;
String x_str;
String id_search_microcontroler;
int ind;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.setTimeout(10); 
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {

  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) 
  {    
    x_str                       = Serial.readString();
    ind                         = x_str.indexOf('/');  //finds location of first 
    first_value                 = x_str.substring(0, ind);
    id_search_microcontroler    = x_str.substring(ind+1);

    if(first_value == "1")
    {
      String message_pong       = "1/"+id_microcontroler+"\n";
      digitalWrite(LED_BUILTIN, HIGH);  
      Serial.print(message_pong);   
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);  
    }
  }
}