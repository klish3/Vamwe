
import processing.serial.*;
import SimpleOpenNI.*;
SimpleOpenNI kinect;
Serial myPort;

FlowField flowfield;
mouseLocation mouselo;

int numboids = 300;

PVector com = new PVector();                                   
PVector com2d = new PVector(); 


String[] flexValuesStr;
float[] flexValuesInt = new float[11];

ArrayList boids, boids2, boids3, boids4;
float concen;
float medita;


// switch
boolean debug = false; //debug mode
boolean trail = false; //trails
boolean colorFlow=true; // segmentation of moving objects?
boolean human = false;
boolean ui = false;

int lastMillis, lastMillis2, lastMillisGamma, lastMillisSs=0;

/*boolean sketchFullScreen() {
 return true;
 }*/
void setup() {
  // screen and video
  size(displayWidth, displayHeight);
  //kinect
  kinect =  new SimpleOpenNI(this); 
  kinect.enableUser();
  kinect.enableRGB();
  //Flowfield
  flowfield = new FlowField(10);
  //Mouse Location
  mouselo = new mouseLocation();//flowfield data where ever mouse is, theta/arraynumber

  //Serial List
  //myPort = new Serial(this, Serial.list()[0],9600);
  //myPort.bufferUntil('\n');
  //println( Serial.list());

  //Boid ArrayList
  boids = new ArrayList();
  boids2 = new ArrayList();
  boids3 = new ArrayList();
  boids4 = new ArrayList();

  // Make a whole bunch of boids with random maxspeed and maxforce values
  for (int i = 0; i < numboids; i++) {
    boids.add(new Boid(new PVector(random(width), random(height)), random(0.1, 0.5)));
    boids3.add(new Boid3(new PVector(random(width), random(height)), random(0.1, 0.5)));
  }
  for (int j = 0; j < numboids*2; j++) {
    boids2.add(new Boid2(new PVector(random(width), random(height)), random(0.1, 0.5)));
    boids4.add(new Boid4(new PVector(random(width), random(height)), random(0.1, 0.5)));
  }
  //graphic setup
  background(0);
  ellipseMode(CENTER);
  smooth();
}

void draw() {
  // Background changing function, changes background (lgamma,0,0,20) 
  //where lgamma = 0 - 100 Low gamma waves (Gamma =  multi-sensory processing
  bg();
  //buttons n =SepandSee, m = flow, l/k = Ui, z/x = debug
  keys();
  //Mouse location vector for debuging 
  PVector ms=new PVector(mouseX, mouseY);

  if (debug) flowfield.display();
  //
  for (int i = 0; i < boids.size(); i++) {
    Boid b = (Boid) boids.get(i);
    Boid2 b2 = (Boid2) boids2.get(i);
    Boid3 b3 = (Boid3) boids3.get(i);
    Boid4 b4 = (Boid4) boids4.get(i);
    //clear every 3min
    if (millis()>lastMillis2+180000) {//every 180sec 0.001mili = 1sec
      lastMillis2 = millis();
      background(0);
    }

    //change flowfield every 20sec
    if (millis()>lastMillis+20000) {//every 20sec 0.001mili = 1sec
      lastMillis = millis();
      flowfield.init();
    }
    //flow state
    if (key=='m') { 
      b.follow(flowfield);
      b2.follow(flowfield);
      b3.follow(flowfield);
      b4.follow(flowfield);
    }
    //sep and seek state
    if (key=='n') { 
      b.separate(boids);
      b2.separate(boids2);
      b3.separate(boids3);
      b4.separate(boids4);
      if (human) {
        b3.applyBehaviors(boids3, com);
      }
    }
    //run all 
    b.run();
    b2.run();
    b3.run();
    b4.run();
  }

  if (kinect.isInit()) {
    kinect.update();
  }

  //User
  int[] userList = kinect.getUsers();
  for (int i=0; i<userList.length;i++) {
    if (kinect.isTrackingSkeleton(userList[i])) {

      human=true;
      if (kinect.getCoM(userList[i], com))
      {
        //converts coordinates to real coordinate, like the size of the screen.
        kinect.convertRealWorldToProjective(com, com2d);
        //mouselo.display(com);
        mouselo.mouseblock(flowfield, com);
      }
    }
  }

  if (ui)userInterface(com);
}

//pulls brainwaves into array for use
void serialEvent(Serial myPort) {
  String flexSerial = myPort.readString();
  flexValuesInt = float(split(flexSerial, ','));
  if (flexValuesInt[0] == 200) {
    //print("Received string over serial: ");
    //println("Flex Serials "+flexSerial);
  }
} 
//new User
void onNewUser(SimpleOpenNI curKinect, int userId) {

  println("New User "+ userId) ;
  curKinect.startTrackingSkeleton(userId);
}
//Lost User
void onLostUser(SimpleOpenNI curContext, int userId)
{
  println("onLostUser - userId: " + userId);
  flowfield.init(); 
  human=false;
}

//change background function
void bg() {
  if (millis()>lastMillisGamma+10000) {//every 10sec 0.001mili = 1sec
    lastMillisGamma=millis();
    rectMode(CORNER);
    float lgamma = map( flexValuesInt[9], 0, 15000, 0, 100);
    noStroke();
    fill(lgamma, 0, 0, 20);
    rect(0, 0, width, height);
    println(lgamma);
  }
}
//buttons n =SepandSee, m = flow, l/k = Ui, z/x = debug
void keys() {

  if (key=='x') debug=true;
  if (key=='z') debug=false;
  if (key==' ') background(0);
  if (key=='l') ui=true;
  if (key=='k') ui=false;
}


//User Interface for checking brain waves and if a human is presenta and where 
void userInterface(PVector dot) {

  float z = map(dot.z, 0, 1500, -300, height-500);
  float x = map(dot.x, -500, 500, 0, width+100);

  pushMatrix();

  fill(255, 80);
  stroke(200);
  //translate(0,250);
  //rotate(PI+PI/2);
  ellipse(0, 0, 600, 600);
  noStroke();
  fill(0);
  textSize(20);
  text("Human", 10, 30); 
  if (human) {
    fill(0, 255, 0);
    ellipse(100, 22, 15, 15); 
    fill(0);
    text("Attention:"+flexValuesInt[1], 10, 65);
    text("Meditation:"+flexValuesInt[2], 10, 90);
    text("Position "+int(x)+" "+int(z), 10, 115);
  }
  else {
    fill(255, 0, 0);
    ellipse(100, 25, 15, 15);
    fill(200);
    text("Attention:", 10, 65);
    text("Meditation:", 10, 90);
    text("Position ", 10, 115);
  }

  if (key=='m') {

    text("Flow Field", 10, 140);
  }
  if (key=='n') {

    text("Separate and Seek", 10, 140);
  }


  fill(0);
  text("Signal: "+flexValuesInt[0], 130, 30); 

  if ( flexValuesInt[0]==0) {
    fill(0, 255, 0);
    ellipse(260, 22, 15, 15);
  }
  if ( flexValuesInt[0]>=1 && flexValuesInt[0]<=100) {
    fill(255, 183, 0);
    ellipse(260, 22, 15, 15);
  }
  if ( flexValuesInt[0]>=100 && flexValuesInt[0]<=200) {
    fill(255, 0, 0);
    ellipse(260, 22, 15, 15);
  }
  textSize(18);
  fill(150);
  text("Vamwe", 10, 165);
  textSize(16);
  text("Tawanda Kanyangarara", 10, 185);


  popMatrix();
}




