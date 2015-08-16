
class Boid4 {

  // The usual stuff
  PVector loc;
  PVector vel;
  PVector acc;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  float mass;
  float colr;    //random color of balls
  float ralpha; //random alpha of balls

  Boid4(PVector l, float mf) {
    loc = l.get();
    r=3;

    maxforce = mf;
    acc = new PVector(0, 0);
    vel = new PVector(0, 0);
    mass = random(0.1, 0.3);
    maxspeed = mass;
    colr = random(50, 70);
    ralpha = random(30, 50);
  }
  public void run() {
    update();
    borders();
    render();
  }


  // Implementing Reynolds' flow field following algorithm
  // http://www.red3d.com/cwr/steer/FlowFollow.html
  void follow(FlowField f) {

    // Look ahead
    PVector ahead = vel.get();
    ahead.normalize();
    ahead.mult(32); // Arbitrarily look 32 pixels ahead
    PVector lookup = PVector.add(loc, ahead);




    // What is the vector at that spot in the flow field?
    PVector desired = f.lookup(lookup);
    // Scale it up by maxspeed
    desired.mult(maxspeed);
    // Steering is desired minus velocity
    PVector steer = PVector.sub(desired, vel);
    steer.limit(maxforce);  // Limit to maximum steering force
    acc.add(steer);
  }
  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M

    acc.add(force);
  }

  void separate (ArrayList<Boid4> boids4) {
    float desiredseparation = r*2;
    PVector sum = new PVector();
    int count = 0;
    // For every boid in the system, check if it's too close
    for (Boid4 other : boids4) {
      float d = PVector.dist(loc, other.loc);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        PVector diff = PVector.sub(loc, other.loc);
        diff.normalize();
        diff.div(d);        // Weight by distance
        sum.add(diff);
        count++;            // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      sum.div(count);
      // Our desired vector is the average scaled to maximum speed
      sum.normalize();
      sum.mult(maxspeed);
      // Implement Reynolds: Steering = Desired - Velocity
      PVector steer = PVector.sub(sum, vel);
      steer.limit(maxforce);
      applyForce(steer);
    }
  }


  // Method to update location
  void update() {
    // Update velocity
    vel.add(acc);


    loc.sub(vel); //subtract to reverse direction
    // Reset accelertion to 0 each cycle
    acc.mult(0);
  }

  void render() {

    fill(170, colr, 20, ralpha);
    pushMatrix();
    translate(loc.x, loc.y);
    stroke(0);

    noStroke();
    ellipse(0, 0, mass*5, mass*5);
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (loc.x < -r) loc.x = width+r;
    if (loc.y < -r) loc.y = height+r;
    if (loc.x > width+r) loc.x = -r;
    if (loc.y > height+r) loc.y = -r;
  }
}


