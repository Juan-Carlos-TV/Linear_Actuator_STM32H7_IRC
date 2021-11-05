float corriente = 0.0;
float resistencia = 1000;
int led1 = 13; 
int led2 = 12;
float val = 0.0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 1; 1 <= 20; i++){
    corriente = (float) analogRead(0)*(5/1023.0)/resistencia+corriente;
  }
  val = corriente*1000/20;
  Serial.print(val,3);
  Serial.println(" mA");
  if (val >= 2.6 || val <= 2.4){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
  }
  else{
    digitalWrite(led1, LOW);
    digitalWrite(led2, HIGH);
  }
  delay(1000);
  corriente = 0;
  val = 0;
}