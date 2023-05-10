 void linefollower(){
int nowtime;
unsigned long lasttime = 0;
int right;
int left;
int middle;
middle = analogRead(A1);
right =analogRead(A0);
left = analogRead(A2);
  nowtime = millis();
if (nowtime - lasttime >= 1000){
  lasttime = nowtime;
  Serial.println("working");
Serial.println(right);
Serial.println(middle);
Serial.println(left);}

// if(middle> 950 || right >950 || left >950 || middle<200 && right<200 && left <200){
//   STOP();
// }
// else{

if(middle - right >= 10  && middle - left >= 10){
  Forward(90);
}
if(right - middle >= 10 && right- left>= 10){
Right(90);
}
if(left- middle>= 10 && left - right>=10 ){
  Left(90);
}
// }
}