class mouseLocation {


  mouseLocation() {
  }

  void display(PVector dot) {
    float z = map(dot.z, 0, 1500, -300, height-500);
    float x = map(dot.x, -500, 500, 0, width+100);
    println(dot.x);
    fill(0);
    ellipse(x, z, 20, 20);
    //println(dot.x+","+dot.y+","+dot.z+","+z);
  }

  void mouseblock( FlowField flow, PVector dot ) {
    PVector[][] newGrid = flow.lookuparea(dot, 10);

    for (int i=0; i<newGrid.length/2;i++) {
      for (int j=0; j<newGrid[i].length/2;j++) {
        for (int k=newGrid.length/2; k<newGrid.length;k++) {
          for (int l=newGrid[i].length/2; l<newGrid[i].length;l++) {


            //ellipse( newGrid[i][j].x, newGrid[i][j].y,10,10);

            //top left
            newGrid[i][j].x = 1;
            newGrid[i][j].y = -1;

            //top right
            newGrid[k][j].x = 1;
            newGrid[k][j].y = 1;

            //bottom left
            newGrid[i][l].x = -1;
            newGrid[i][l].y = -1;

            //bottom right
            newGrid[k][l].x = -1;
            newGrid[k][l].y = 1;
          }
        }
      }
    }
  }

  //finding mouse location based on array, vector
  void mousel(FlowField flow, PVector lol) {

    PVector mlp = flow.lookup(lol);
    pushMatrix();
    //println(mlp.x+" "+mlp.y);


    popMatrix();
  }
}

