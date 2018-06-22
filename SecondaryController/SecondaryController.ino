#define EXTEND 11
#define RETRACT 9
#define STEERING_TOP 875
#define STEERING_DOWN 130

int targetPos = 600;
int deadBand = 2;

void setup() {
  Serial.begin(115200);
  pinMode(A1, INPUT);
  //pinMode(3, OUTPUT);
  //pinMode(5, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  int curPos = analogRead(A1);
  

  if (Serial.available() > 0)
    targetPos = Serial.parseInt();

  int delta = abs(targetPos - curPos);

  if (curPos < (targetPos - deadBand)) {
    analogWrite(EXTEND, delta < 25 ? 128 : 0);
    analogWrite(RETRACT, 255);
  } else if (curPos > (targetPos + deadBand)) {
    analogWrite(EXTEND, 255);
    analogWrite(RETRACT, delta < 25 ? 128 : 0);
  } else {
    analogWrite(EXTEND, 255);
    analogWrite(RETRACT, 255);
  }
  Serial.print(curPos);
  Serial.print("  ");
  Serial.print(targetPos);
  Serial.print("  ");
  Serial.println(map(curPos, STEERING_DOWN, STEERING_TOP, 1000,2000));
}
