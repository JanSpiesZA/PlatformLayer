//Platform layer for MBot

import org.openkinect.freenect.*;
import org.openkinect.processing.*;

// Kinect Library object
Kinect kinect;

float maxKinectDetectNormal = 300.0;  //Maximum distance we are going to use the kinect to detect distance, measured in cm's
float maxKinectDetectTooFar = 800.0;


// We'll use a lookup table so that we don't have to repeat the math over and over
float[] depthLookUp = new float[2048];

int maxParticles = 0;
//Particle[] particle = new Particle[maxParticles];


int tileSize = 20;      //Occupancy grid size in cm's

float worldMapHeight = maxKinectDetectNormal;  //  the amount of pixels of the png file in the x and y direction
float tempWorldWidth = 2 * int(tan(radians(45))*worldMapHeight);  //To be used as the actual distance of the world map x axis, measured in cm 
float worldMapWidth = ceil(tempWorldWidth / tileSize) * tileSize;

int screenHeight = 600;


float scaleFactor = screenHeight / worldMapHeight;
int screenWidth = int(worldMapWidth * scaleFactor);


int maxHistogramX = int(worldMapWidth/tileSize);
int maxHistogramY = int(worldMapHeight/tileSize);
float scaledtileSize = tileSize * scaleFactor;

float xOffset = screenWidth/2;

int skip=10;

Robot myrobot = new Robot();

PVector robotPos = new PVector(screenWidth/2, screenHeight);
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

float sensorX = 0.0;
float sensorY = 30.0;
float sensorPhi = 0.0;




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
      tile[x][y] = new Tile();
    }
  }
}

void draw()
{   
  clear();
  println("worldWidth: "+worldMapWidth+", worldHeight: "+worldMapHeight);
  println("screenWidth: "+screenWidth+", screenHeight: "+screenHeight);
  println("x: "+maxHistogramX+", y:"+maxHistogramY+", scaleFactor: "+scaleFactor);
  ////image(kinect.getVideoImage(), 0, 0);
  //stroke(128);
  
  
  
  
  drawWorld();
  
  //fill(255,0,0);
  //drawTarget();
  plotRobot();
  
  //resetNodes();
  
  //drawPixels();
  
  
  
  
  
  
  
  
  
}

void drawPixels()
{
 // Get the raw depth as array of integers
 int[] depth = kinect.getRawDepth();
 
 transRot(myrobot.x, myrobot.y, myrobot.heading, sensorX, sensorY);
    ellipse(x_temp,y_temp,10,10);
  
 for (int x = 0; x < kinect.width; x += skip) 
 {
   for (int y = 0; y < kinect.height; y += skip) 
   {
     int offset = x + y*kinect.width;

     // Convert kinect data to world xyz coordinate
     int rawDepth = depth[offset];
     PVector v = depthToWorld(x, y, rawDepth);
     
    v.x *= 100;      //Convert mm measurements into cm's
    v.z *= 100;
    
    v.x *= scaleFactor;
    v.z *= scaleFactor;
         
     transRot(myrobot.x, myrobot.y, myrobot.heading, v.x, -v.z);     
     
     if (v.z > 0 && v.z < 400)    //Test for any invalid depth values      
     {
        fill(255);     
        ellipse((x_temp*scaleFactor),y_temp*scaleFactor,5,5);
        updateGravity((x_temp*scaleFactor),y_temp*scaleFactor);
        
        transRot(myrobot.x, myrobot.y, myrobot.heading, sensorX, sensorY);
        ellipse(x_temp,y_temp,10,10);
     }
     
     //if (v.z > 0 && v.z < 400)    //Test for any invalid depth values      
     //{
     //fill(255);
     //ellipse((v.x*scaleFactor+xOffset),height - v.z*scaleFactor,5,5);
     //updateGravity((v.x+worldMapWidth/2),worldMapHeight - v.z);
     //}
   }
 }  
}


//Draws the tiles of the histogram with the gravity being represented by the amount of data in each node
void drawWorld()
{
  for (int x = 0; x < maxHistogramX; x++)
  {
    for (int y = 0; y <  maxHistogramY; y++)
    {      
      fill(tile[x][y].gravityCol);
      rect((x*scaledtileSize),y*scaledtileSize,scaledtileSize,scaledtileSize);
      fill(255);
      text(tile[x][y].gravity, (x*scaledtileSize+scaledtileSize/2), y*scaledtileSize+scaledtileSize/2);
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
  int x = int(_x/tileSize);
  if (x < 0) x = 0;
  if (x > maxHistogramX) x = maxHistogramX-1;
  
  int y = int(_y/tileSize);
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

void mousePressed()
{
  if (mousePressed && (mouseButton == LEFT)) changeGoal();  
}

//Change the goal location everytime the mouse is clicked
void changeGoal()
{
  goalX = mouseX;
  goalY = mouseY;
  startX = myrobot.x;
  startY = myrobot.y; 
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