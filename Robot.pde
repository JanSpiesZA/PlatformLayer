class Robot
{
  float x = screenWidth/2;
  float y = screenHeight;
  float heading = 0.0;
  float diameter = 40 * scaleFactor;  //diameter of chassis
  float noseLength = diameter/2;
  float[] state = {0,0,0};
  float maxSpeed = 1 * scaleFactor;
  float maxTurnRate = 5 * scaleFactor;
  
  Robot()
  {    
  }
  
  void update(float _x, float _y)
  {
    fill(255,0,0);
    ellipse(_x,_y,40,40);
  }
  
  void display()
  {
    ellipse(x,y,diameter,diameter);
    float noseX = x + noseLength * cos(heading);
    float noseY = y + noseLength * sin(heading);
    line (x, y, noseX, noseY);    
  }
  
  //Moves the robot  
  void move(float turnAngle, float distance)
  { 
    heading += turnAngle;  //Add the turnAngle value to the current heading
    if (heading >= (2*PI)) heading -= (2*PI);
    if (heading <= (-2*PI)) heading += (2*PI);    
    
    float newX = x + distance * cos(heading);
    float newY = y + distance * sin(heading);
    x = newX;
    y = newY;
    
    state[0] = x;
    state[1] = y;
    state[2] = heading;    
  }
}