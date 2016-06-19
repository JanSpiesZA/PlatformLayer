//Platform layer for MBot

import processing.serial.*;
Serial myPort;

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

// Kinect Library object
Kinect kinect;

float maxKinectDetectNormal = 400.0;  //Maximum distance we are going to use the kinect to detect distance, measured in cm's
float maxKinectDetectTooFar = 800.0;
float kinectFOW = 60.0;


// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

int maxParticles = 0;
//Particle[] particle = new Particle[maxParticles];


int tileSize = 20;      //Occupancy grid size in cm's

float worldMapHeight = maxKinectDetectNormal;  //  the amount of pixels of the png file in the x and y direction
float tempWorldWidth = 2 * int(tan(radians(kinectFOW/2))*worldMapHeight);  //To be used as the actual distance of the world map x axis, measured in cm 
float worldMapWidth = ceil(tempWorldWidth / tileSize) * tileSize;

int screenHeight = 600;


float scaleFactor = screenHeight / worldMapHeight;
int screenWidth = int(worldMapWidth * scaleFactor);


int maxHistogramX = int(worldMapWidth/tileSize);
int maxHistogramY = int(worldMapHeight/tileSize);
float scaledtileSize = tileSize * scaleFactor;

float xOffset = screenWidth/2;

int skip=10;

PVector robotPos = new PVector(screenWidth/2, screenHeight-60, -PI/2);
PVector goalPos = new PVector (50 * scaleFactor,0 * scaleFactor);    //Position of goal, in robot frame
float robotDiameter = 46 * scaleFactor; //Physical diameter of robot measured in cm's

float x_temp = 0.0;
float y_temp = 0.0;

float goalX = screenWidth / 2;            //Goal's X and Y coordinates, set up by clicking with the mouse on the screen
float goalY = screenHeight / 2;
float startX = 0;          //Starting point for straight line to goal used by Bug algorithm families
float startY = 0;
float safeZone = 20.0 * scaleFactor;          //Safe area around target assumed the robot reached its goal;
float moveSpeed = 0;
float moveAngle = 0;
float moveGain = 0.01;
float turnGain = 0.1;
float errorAngle = 0.0;

float sensorX = robotPos.x + 0.0;
float sensorY = robotPos.y + 0.0;
float sensorPhi = robotPos.z + 0.0;



int oldMillis, newMillis, oldTxMillis;

PVector targetPos = new PVector(50.0, 0.0, 0.0);

PVector vectorFWD = new PVector(0,-50 * scaleFactor);
PVector vectorAO = new PVector();
PVector vectorAOFWD = new PVector();

int delta_t = 500;      //Time waited between sending serial data

boolean allowTX = false;    //Allows data to be transmitted to the driverlayer


Tile tile[][] = new Tile[maxHistogramX][maxHistogramY];

void setup()
{  
  kinect = new Kinect(this);
  kinect.initDepth();
  //kinect.initVideo();
  
  // Lookup table for all possible depth values (0 - 2047)
  for (int i = 0; i < depthLookUp.length; i++) {
    depthLookUp[i] = rawDepthToMeters(i);
  } 
  
  //size(200,200);
  surface.setResizable(true);
  surface.setSize(screenWidth,screenHeight);
  
  //Initialise 2D occupancy map from Kinect view
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y < maxHistogramY; y++)
    {
      tile[x][y] = new Tile(int(x*scaledtileSize + scaledtileSize/2), int(y*scaledtileSize + scaledtileSize/2));
    }
  }
  
  printArray(Serial.list());
  myPort = new Serial(this, "COM3", 115200);  
  delay(5000);      //Delay to make sure the serial port initialises correctly
  myPort.write("<v00\r");
  delay(1000);
  myPort.write("<w0\r");
  delay(1000);
}

void draw()
{   
  clear();
  
  //println("worldWidth: "+worldMapWidth+", worldHeight: "+worldMapHeight);
  //println("screenWidth: "+screenWidth+", screenHeight: "+screenHeight);
  //println("x: "+maxHistogramX+", y:"+maxHistogramY+", scaleFactor: "+scaleFactor);
  ////image(kinect.getVideoImage(), 0, 0);
  //stroke(128);
  
  drawWorld();
  
  ////drawTarget();
  plotRobot();
  
  resetNodes();
  
  drawPixels();
  
  calcVecAO();
  calcVecAOFWD();  
  
  drawVectors();
  
  
  //Print Kinect Sensor Field of View (FOW) lines on display
  float deltaX = tan(radians(kinectFOW/2)) * 400;
  line(robotPos.x, robotPos.y, robotPos.x - deltaX, 0);
  line(robotPos.x, robotPos.y, robotPos.x + deltaX, 0);
  
  float fwdV = vectorAOFWD.mag();
  
  println(fwdV);
  
  
  float angleVectorAOFWD = atan2(vectorAOFWD.y, vectorAOFWD.x); 
  
  
  float errorAngle = angleVectorAOFWD - robotPos.z;
  
  //println("angle of vectorAOFWD: "+angleVectorAOFWD);
  if (errorAngle < (-PI)) errorAngle += 2*PI;
  if (errorAngle > (PI)) errorAngle -= 2*PI;
  
  
  
  errorAngle = min (errorAngle, 1.0);
  errorAngle = max (errorAngle, -1.0);
  
  String tempAngle = nf(errorAngle,1,2);   //Format errorAngle to 2 decimal points
  errorAngle = float(tempAngle);
  errorAngle *= -1000.0;        //Invert turning Angle and multiply by 1000 to convert floating point to intgere
  //println("errorAngle: "+errorAngle);
  
  oldMillis = newMillis;
  newMillis = millis();
  textSize(16);  
  textAlign(LEFT, TOP);
  fill(0);
  text("refresh rate (hz): "+1000/(newMillis - oldMillis),5,screenHeight - 90);
  text("avoid angle: "+str(errorAngle),5,screenHeight - 60);
  //String txString = "<w"+str(errorAngle)+"\r";  
  //String txString = "<v"+str(1.20)+"\r";
  
  //Tx serial data only once every delta_t milliseconds
  int time = millis();
  int interval = time - oldTxMillis;
  if (interval > delta_t)
  {
    if (allowTX)
    {
      println("TXING!!!");
      //text(txString,5,screenHeight - 30);
      //print(txString);
      //myPort.write(txString);
      myPort.write("<w"+str(errorAngle)+"\r");
      delay(1);                //Delay after first serial comms
      myPort.write("<v"+str(fwdV)+"\r");   
      
      oldTxMillis = time;
      //myPort.clear();
    } 
    else
    {
      println("NO TX!");
      
    }
  }
}

void calcVecAOFWD()
{  
  vectorAOFWD = PVector.add(vectorFWD, vectorAO); 
}

void calcVecAO()
{
  vectorAO.mult(0);
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y < maxHistogramY; y++)
    {
      //If any tile's gravity is greater than 0, add that vector to the avoid obstacle vector
      if (tile[x][y].gravity != 0)
      {
        vectorAO.add(tile[x][y].field);              
      }
    }
  }
}

void drawVectors()
{  
  //Draws a vector pointing straight forward
  strokeWeight(2);
  stroke(0);
  line(robotPos.x, robotPos.y, robotPos.x + vectorFWD.x, robotPos.y + vectorFWD.y);
  
  //Draws a vector pointing away from all the obstacles
  strokeWeight(4);
  stroke(255,0,0);
  line(robotPos.x, robotPos.y, robotPos.x + vectorAO.x, robotPos.y + vectorAO.y);
  
  strokeWeight(2);
  stroke(0,255,0);
  line(robotPos.x, robotPos.y, robotPos.x + vectorAOFWD.x, robotPos.y + vectorAOFWD.y);
  
}

void drawPixels()
{
 // Get the raw depth as array of integers
 int[] depth = kinect.getRawDepth();
  
 for (int x = 0; x < kinect.width; x += skip) 
 {
   for (int y = 0; y < kinect.height; y += skip) 
   {
    int offset = x + y*kinect.width;

     // Convert kinect data to world xyz coordinate
     int rawDepth = depth[offset];
     PVector v = depthToWorld(x, y, rawDepth);
     
    v.x *= 100;      //Convert depth value from meters into mm
    v.z *= 100;
    
    v.x *= scaleFactor;  //Multiply values with scale factor 
    v.z *= scaleFactor;
         
     //transRot(robotPos.x, robotPos.y, robotPos.z, v.x, -v.z);     
     
     if (v.z > 0 && v.z < maxKinectDetectNormal*scaleFactor)    //Test for any invalid depth values      
     {
        fill(255);     
        //ellipse((x_temp*scaleFactor),y_temp*scaleFactor,5,5);
        
        ellipse(robotPos.x + v.x, robotPos.y - v.z ,5,5);
        updateGravity(robotPos.x + v.x,robotPos.y - v.z );
        
        transRot(robotPos.x, robotPos.y, robotPos.z, sensorX, sensorY);
        ellipse(x_temp,y_temp,10,10);
     }
   }
 }  
}


//Draws the tiles of the world
//Each element is drawn seperately in order to control the layering effect better.
//  If not seperately new tile will overwrite the previous drawn tile and visual data will be lost
void drawWorld()
{
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y <  maxHistogramY; y++)
    {
      tile[x][y].update();
      tile[x][y].drawTile();
    }
  }  
  
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y <  maxHistogramY; y++)
    {
      tile[x][y].drawTileText();      
    }
  }  
  
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y <  maxHistogramY; y++)
    {
      tile[x][y].drawTileForce();     
    }
  }  
}

void updateGravity(float _x, float _y)
{
  //Determine histogram weigths
  //for (int p = 0; p < maxParticles; p ++)
  //{
  //  histo[int(particle[p].x/tileSize)][int(particle[p].y/tileSize)].gravity ++;   
  //}  
  int x = int(_x/(tileSize*scaleFactor));
  if (x < 0) x = 0;
  if (x > maxHistogramX) x = maxHistogramX-1;
  
  int y = int(_y/(tileSize*scaleFactor));
  if (y < 0) y = 0;
  if (y > maxHistogramY) y = maxHistogramY-1;
  
  tile[x][y].gravity++;
  tile[x][y].update();
}

void resetNodes()
{
  for(int y = 0; y < maxHistogramY; y++)
  for(int x = 0; x < maxHistogramX; x++)
  {
    tile[x][y].clearGravity();
  }
}

void plotRobot()
{
  fill(255);
  ellipse (robotPos.x, robotPos.y, robotDiameter, robotDiameter);
  //float deltaX = goalX - myrobot.x;
  //float deltaY = goalY - myrobot.y;  
  //float targetAngle = atan2(deltaY, deltaX);    
  //float distanceToTarget = sqrt(pow(deltaX,2) + pow(deltaY,2));
  
  //errorAngle = targetAngle - myrobot.heading;
  //if (errorAngle < -PI) errorAngle += (2*PI);
  //if (errorAngle > PI) errorAngle -= (2*PI);  
  
  //moveAngle = min (myrobot.maxTurnRate, (turnGain * errorAngle));  //P controller to turn towards goal
  //moveSpeed = min (myrobot.maxSpeed ,(moveGain * (distanceToTarget))); 
  ////myrobot.move(moveAngle,moveSpeed);    
  //myrobot.display();
}

void keyPressed()
{
  if (key == 'x')
  {
    allowTX = !allowTX;
    myPort.write("<w0\r");
    delay(1);
    myPort.write("<v0\r");
  }  
}

void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();  
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{
  goalX = mouseX;
  goalY = mouseY;
  startX = robotPos.x;
  startY = robotPos.y; 
}
/////////////////////////////////////////////////////////////////////////////////////////////////
void drawTarget()
{
  stroke(0);    //Draw in black
  strokeWeight(2);
  fill(255,0,0);
  ellipse (goalX,goalY, safeZone*3,safeZone*3);
  fill(255);
  ellipse (goalX,goalY, safeZone*2,safeZone*2);
  //stroke(255,0,0);
  fill(255,0,0);
  ellipse (goalX,goalY, safeZone,safeZone);
  stroke(0);    //Draw in black
  strokeWeight(1);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////
//Rotates and translates an X and Y coordinate onto a local frame using the local frame's X,Y and HEADING
//Returns x_temp and y_temp which must be allocated to other relevant variables in order to prevent overwriting of their values
void transRot (float x_frame, float y_frame, float phi_frame, float x_point, float y_point)
{
 x_temp = cos(phi_frame) * x_point - sin(phi_frame)*y_point + x_frame; //Uses transformation and rotation to plot sensor gloablly 
 y_temp = sin(phi_frame) * x_point + cos(phi_frame)*y_point + y_frame;
}
///////////////////////////////////////////////////////////////////////////////////////////////