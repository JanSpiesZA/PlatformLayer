class Tile
{
  int gravity = 0;    //Number of points in tile
  color gravityCol = color(gravity);
  //int[] arrayXY = {0,0};
  //int[] worldXY = {0,0};
  PVector field;
  PVector tilePos;
  float force =0.0;
  
  
  Tile()
  {
    gravityCol = color(200,150,150);
    field = new PVector();
    tilePos = new PVector();
  }
  
  Tile(int _tileX, int _tileY)
  {
    gravityCol = color(150,200,150);
    field = new PVector();
    tilePos = new PVector();
    tilePos.x = _tileX;
    tilePos.y = _tileY;
  }
  
  void clearGravity()
  {
    gravity = 0;
    gravityCol = color(200,150,150);
  }
  
  void update()
  { 
    switch(gravity)
    {
      case 0:
      {
        gravityCol = color(150,200,150); //color(min(gravity,255)); //map(gravity,0,2048,0,128));
        break;
      }
      default:
        {
          field.x = robotPos.x - tilePos.x;
          field.y = robotPos.y - tilePos.y;
          float distance = PVector.dist(robotPos, tilePos);
          field.normalize();
          
          force = pow(distance,2);
          
          field.div(force);
          field.mult(gravity * 1000);
          
          gravityCol = color(200,150,150);
          break;
        }
    }      
  }
  
  void tileDraw()
  {    
    stroke(150);        //Lines between tiles are black
    strokeWeight(1);  //Stroke weight makes the lines very light
    rectMode(CENTER);
    fill(gravityCol,200);
    rect(tilePos.x, tilePos.y, scaledtileSize, scaledtileSize);  //Draws a rectangle to indicate the tile
    
    //Draws a flowfield indicator
    stroke(0);
    line (tilePos.x, tilePos.y, tilePos.x + field.x, tilePos.y + field.y);
    
    //Writes the gravity value in the tile
    fill(255);
    text(gravity, tilePos.x, tilePos.y);
  }  
}