class HistNode
{
  int gravity = 0;    //Number of points in tile
  color gravityCol = color(gravity);
  int[] arrayXY = {0,0};
  int[] worldXY = {0,0};
  
  HistNode()
  {
    gravityCol = color(200,150,150);
  }
  
  void clearGravity()
  {
    gravity = 0;
    gravityCol = color(200,150,150);
  }
  
  void update()
  {
    gravityCol = color(min(gravity,255)); //map(gravity,0,2048,0,128));
  }
}