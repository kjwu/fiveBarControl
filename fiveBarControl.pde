import processing.serial.*;

import controlP5.*;

ControlP5 cp5;

int l0 = 25; // Length between origin of the two motors
int l1 = 132; // Length from motor to passive joints
int l2 = 156; // Length from passive joints to end effector
int l3 = 158; // Length between the origin Y to the center of the plate
float scaleFactor = 2;
float[] angles = {PI/2, PI/2};
float currentX;
float currentY;
float targetX;
float targetY;
float centerX;
float centerY;
int boundary = 127;
boolean record = false;
boolean play = false;

boolean simulate = false;
boolean performance = false;
int nextPos = 0;

ArrayList<PVector> path = new ArrayList<PVector>();
int pathSize = 200; // Change this for path length

Serial myPort;  // Create object from Serial class

boolean transducerON = false;


byte[] byteData = new byte[5]; // 1 flag byte + 4 data bytes
boolean dataSent = false;

void setup() {
  size(800, 800);
  frameRate(20);
  
  
  if (!simulate) {
    String portName = Serial.list()[7];
    myPort = new Serial(this, portName, 115200);
    myPort.buffer(5); // Buffer 4 bytes before triggering serialEvent

    printArray(Serial.list());
  }
  
  // Initialize path array
  //for (int i = 0; i < pathSize; i++) {
    path.add( new PVector(0,l3));
  //}

  currentX = 0;
  currentY = 0;
}

void draw() {
  background(0);
  
  pushMatrix();
  if(record){
    fill(0,200,200);
  }else{
    fill(0,50,50);
  }
  rect(40,20,50,20);
  fill(255);
  text("record",45,35);
  popMatrix();
  
  pushMatrix();
  if(play){
    fill(0,200,200);
  }else{
    fill(0,50,50);
  }
  rect(100,20,50,20);
  fill(255);
  text("play",105,35);
  
  text("press 'r' for record, 'p' for play, 'c' for clear", 170,35);
  popMatrix();
  

  centerX = width/2;
  centerY = height / 2;

  if (dist(mouseX, mouseY, centerX, centerY) < boundary*scaleFactor) {
    targetX = (mouseX-centerX)/scaleFactor;
    targetY = (mouseY-centerY)/scaleFactor;
  }


  



  int len = path.size();

  if(record){
    if (len > 0) {
      if (dist(targetX, targetY+l3, path.get(len - 1).x, path.get(len - 1).y) > 1) {
        if (path.size() > pathSize) {
          path.remove(0);
        }
  
        path.add(new PVector(targetX, targetY+(l3)));
        currentX = targetX;
        currentY = targetY;
      }
    }
  }
  
  
  if(play){
    println(nextPos);
    if (nextPos < len - 1) {
      nextPos = (nextPos + 1);
    }else{
      nextPos = 0;
    }
     currentX = path.get(nextPos).x;
     currentY = path.get(nextPos).y-l3;
  }

  if(!record &&!play){
      if (dist(targetX, targetY, currentX, currentY) < 50) {
      currentX = targetX;
      currentY = targetY;
    }
  }



  pushMatrix();
  noFill();
  translate(centerX, centerY);
  scale(scaleFactor);

  if (!performance) {
    ellipse(0, 0, 20, 20);
    ellipse(0, 0, boundary * 2, boundary * 2);
    rect(-170, -170, 340, 340);
    cursor();
  } else {
    noFill();
    stroke(255);
    strokeWeight(.2);

    ellipse(currentX, currentY, 10, 10);
    line(currentX + 3, currentY, currentX + 8, currentY);
    line(currentX - 3, currentY, currentX - 8, currentY);
    line(currentX, currentY + 3, currentX, currentY + 8);
    line(currentX, currentY - 3, currentX, currentY - 8);

    noCursor();
    fill(0);
    noStroke();
  }




  translate(0, -l3);
  
  for (int i = 0; i < len; i++) {
    stroke(255);
    if(!performance) circle(path.get(i).x,path.get(i).y,5);
    if (i > 0) {
      strokeWeight(0.5);
      if(!performance)line(path.get(i).x, path.get(i).y, path.get(i-1).x, path.get(i-1).y);
    }
  }

  
  stroke(255);
  plotPlot(currentX, currentY + l3);
  popMatrix();

  if (performance) {
    pushMatrix();
    fill(0);
    noStroke();
    rect(0, 0, width/2 - boundary * scaleFactor, height);
    rect(0, 0, width, height/2 - boundary * scaleFactor);
    rect(width, 0, -width/2 + boundary * scaleFactor, height);
    rect(0, height, width, -height/2 + boundary * scaleFactor);

    popMatrix();
  }
  
  
 
  
  if (!simulate) {
    if (!dataSent) {
      sendData(int(360-degrees(angles[0])), int(180-degrees(angles[1])));
      dataSent = true;
    }
  }
}

void plotArms(float shoulder1, float shoulder2, float efx, float efy) {
  float[] p1 = {-l0 + l1 * cos(shoulder1), l1 * sin(shoulder1)};
  float[] p2 = {l0 + l1 * cos(shoulder2), l1 * sin(shoulder2)};

  if (!performance) {
    line(-l0, 0, p1[0], p1[1]);
    line(p1[0], p1[1], efx, efy);
    fill(255, 0, 0);
    ellipse(efx, efy, 5, 5);

    line(l0, 0, p2[0], p2[1]);
    line(p2[0], p2[1], efx, efy);
  }
}

void plotPlot(float efx, float efy) {
  angles = calcAngles(efx, efy);
  plotArms(angles[0], angles[1], efx, efy);
}

float[] calcAngles(float x, float y) {
  float beta1 = atan2(y, (l0 + x));
  float beta2 = atan2(y, (l0 - x));

  float alpha1Calc = (sq(l1) + (sq(l0 + x) + sq(y)) - sq(l2)) / (2 * l1 * sqrt(sq(l0 + x) + sq(y)));
  float alpha2Calc = (sq(l1) + (sq(l0 - x) + sq(y)) - sq(l2)) / (2 * l1 * sqrt(sq(l0 - x) + sq(y)));

  if (alpha1Calc > 1 || alpha2Calc > 1) {
    return new float[] {PI/2, PI/2};
  }

  float alpha1 = acos(alpha1Calc);
  float alpha2 = acos(alpha2Calc);

  float shoulder1 = beta1 + alpha1;
  float shoulder2 = PI - beta2 - alpha2;

  return new float[] {shoulder1, shoulder2};
}

void keyPressed() {
  if (key == 'f' || key == 'F') {
    performance = !performance;
  }
  
  if (key == 't' || key == 'T'){
        transducerON = !transducerON;
    sendData('T', transducerON ? 1 : 0);
  }
  
  if (key == 'r' || key == 'R'){
        record = !record;
        play = false;
  }
  
  if (key == 'p' || key == 'P'){
        play = !play;
        record = false;
  }
  
  if (key == 'c' || key == 'C'){
    path.clear();
    path.add(new PVector(currentX, currentY+(l3)));
    nextPos = 0;
  }
  
}


void sendData(int firstVal, int secondVal) {
  // Create a byte array with 5 bytes (1 flag byte + 4 data bytes)
  byteData[0] = (byte) 0xFF; // Flag byte
  byteData[1] = (byte) (firstVal >> 8); // High byte of first value
  byteData[2] = (byte) (firstVal & 0xFF); // Low byte of first value
  byteData[3] = (byte) (secondVal >> 8); // High byte of second value
  byteData[4] = (byte) (secondVal & 0xFF); // Low byte of second value

  // Send the byte array to the Arduino
  myPort.write(byteData);
}

void serialEvent(Serial p) {

  if (p.read() == 0xCC) {


    byte[] incomingData = new byte[4];
    p.readBytes(incomingData);

    //println(incomingData[0]+","+incomingData[1]+","+incomingData[2]+","+incomingData[3]);
    int receivedFirstValue = (incomingData[0] << 8) | (incomingData[1] & 0xFF);
    int receivedSecondValue = (incomingData[2] << 8) | (incomingData[3] & 0xFF);

    if (receivedFirstValue <1023 && receivedSecondValue<1023) {
      println("Received - First value: " + receivedFirstValue + ", Second value: " + receivedSecondValue);
    }
  }
  // Reset the flag to send new values  
  dataSent = false;
}

//void toggle(boolean theFlag) {
//  if(theFlag==true) {
//    col = color(255);
//  } else {
//    col = color(100);
//  }
//  println("a toggle event.");
//}
