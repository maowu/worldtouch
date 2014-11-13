public class InteractiveContour {
  Contour ct;
  ArrayList<PVector> blobPts;
  PVector centerPt;
  Rectangle bondingBox;
  
  public InteractiveContour(Contour t_contour) {
    this.ct = t_contour;
    bondingBox = this.ct.getBoundingBox();
    blobPts = new ArrayList<PVector>();
    smoothContour();
  }
  
  public void smoothContour() {
    ct.setPolygonApproximationFactor(max( (float)ct.getPolygonApproximationFactor()*0.2 , 0.1));
    blobPts = ct.getPolygonApproximation().getPoints();
  }
 
  
  public void calculatePolyCenter() {
    float x = 0.;
    float y = 0.;
    int pointCount = 0;
    
    for (PVector point : blobPts) {
      x += point.x;
      y += point.y;
      pointCount++;
    }
  
    x = x/(float)pointCount;
    y = y/(float)pointCount;
    
    centerPt = new PVector(x, y);
  }
  
  public PVector getCenterPoint() {
    calculatePolyCenter();
    return centerPt;
  }
  
  public Rectangle getBoundingBox(){
    return bondingBox;
  }
  
  public float area() {
    return ct.area();
  }
  
  public ArrayList<PVector> getPoints() {
    return blobPts;
  }
  
}
