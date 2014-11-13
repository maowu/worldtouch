import SimpleOpenNI.*;
import gab.opencv.*;
import hypermedia.net.*;

import java.awt.Rectangle;
import java.awt.Polygon;

import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;

import org.opencv.core.Mat;
import org.opencv.core.CvType;


// setting for this application
int screen_x = 0;    //  screen location x
int screen_y = 0;    //  screen location y
int sw = 1280;       //  screen width
int sh = 960;        //  screen height
int kw = 640;        //  kinect capture width
int kh = 480;        //  kinect capture height

// Kinector for OpenNI and Something for OpenCV
SimpleOpenNI  kinect;
OpenCV opencv;

int[] bgDepth;
int[] nowbgDepth;
int[] depthMap; 
int[] depthDivMap; 
PGraphics sensor_canvas, warp_canvas;

PImage depthImg, maskImg, rgbImg, touchImg, objImg, subbgImg;

int minDepth =  500;      // the min depth distance for kinect sensing (mm)
int maxDepth = 1500;      // the man depth distance for kinect sensing (mm)
int mintouchDepth =  5;  // the min depth distance bewteen surface
int maxtouchDepth = 200;  // the max depth distance bewteen surface
int threshold = 200;      // opencv threshold value

float wScale = 0;         // calurate the scale from kw->sw
float hScale = 0;         // calurate the scale from kh->sh

int SENSOR_COLOR = 0xFFFFFFFF;
int BG_COLOR = 0xFF000000;
int BLOBBG_COLOR = 0xFFFF0000;
int NOTRANGE_COLOR = color(55, 107, 109);
int OBJ_COLOR = 0xFFFF5555;
int BGSUB_COLOR = 0xFF5555FF;
int NO_COLOR = 0x00000000;
int BORDER_COLOR = color(134, 193, 102);

long check_timer = 0;
int BG_AvgNums = 10;
int bgsavecount = 0;
int nowbgsavecount = 0;
boolean _DEBUG = false;
boolean _BG_SAVE = false;
boolean _NOWBG_SAVE = false;
boolean _DRAW_DEMO = false;
boolean _SHOW_SET = true;

ArrayList<Contour> contours = new ArrayList<Contour>();
ArrayList<InteractiveContour> blobs = new ArrayList<InteractiveContour>();

ArrayList<PVector> client_center = new ArrayList<PVector>();
ArrayList<PVector> local_center = new ArrayList<PVector>();

// sensor area
PVector[] area = new PVector[4];

// for Network
UDP udp;  // define the UDP object
String HOST_IP = "localhost";
int send_port = 12345;
int recieve_port = 54321;
String UDP_Str = "";
float[] tdata;

// for setting
Settings settings = null;
String TIP_MSG = "";


// for vision
int NUM_PTS = 256;
float LERP_RATE = 0.1;
PVector[] pts;

void setup() {
  // loading pre setting
  loadInitialSettings(); 
  
  size(sw, sh, P2D);

  // init kinect setting
  kinect = new SimpleOpenNI(this);
  if (kinect.isInit() == false)
  {
    println("Can't init SimpleOpenNI, maybe the camera is not connected!"); 
    exit();
    return;
  }
  // setting mirror
  kinect.setMirror(false);
  //  enable depthMap generation
  kinect.enableDepth();
  //  enable rgb image get
  kinect.enableRGB();
  kinect.alternativeViewPointDepthToImage();
  
  TIP_MSG = "kinect start ......";

  // setting some PImage init
  depthImg = new PImage(kw, kh);
  touchImg = new PImage(kw, kh);
  maskImg = new PImage(kw, kh);
  rgbImg = new PImage(kw, kh);
  objImg = new PImage(kw, kh);
  subbgImg = new PImage(kw, kh);
  
  // setting depth info init
  bgDepth = new int[kw*kh];
  nowbgDepth = new int[kw*kh];
  depthDivMap = new int[kw*kh];
  for (int i=0; i< kw*kh; i++) {
    bgDepth[i] = 0;
    nowbgDepth[i] = 0;
    depthDivMap[i] = 0;
  }
  
  opencv = new OpenCV(this, kw, kh);
  
  wScale = (float)sw/(float)kw;
  hScale = (float)sh/(float)kh;
  
  // for debug show
  sensor_canvas = createGraphics(kw, kh);
  warp_canvas = createGraphics(kw, kh);
  TIP_MSG = "";
  
  // Network
  udp = new UDP( this, recieve_port );
  udp.listen( true );
  
  // for demo 
  pts = new PVector[NUM_PTS];
  for (int i=0; i<NUM_PTS; i++)
    pts[i] = new PVector(width/2, height/2);
  
  
}


void draw() {
  // update the cam
  kinect.update();

  if (millis() - check_timer > 5) {
    // ----- begin the kinect sensing and processing ------- //
    background(0);
    sensor_canvas.clear();
    warp_canvas.clear();

    depthMap = kinect.depthMap();
    depthImg = kinect.depthImage();
    rgbImg = kinect.rgbImage();
    
    // save new depth info for background
    if (_BG_SAVE) {
      bgsavecount++;
      depthBGSave(bgDepth);
      TIP_MSG = "Don't Move, Background scaning ......" + (BG_AvgNums-bgsavecount);
    }
    if (_NOWBG_SAVE) {
      nowbgsavecount++;
      depthBGSave(nowbgDepth);
      TIP_MSG = "Don't Move, Background scaning ......" + (BG_AvgNums-nowbgsavecount);
    }

    // turn depthMap data into gray image (with limited distance between max and min depth )
    touchImg.loadPixels();
    depthImg.loadPixels();
    objImg.loadPixels();
    int tmp_index = 0;
    PVector tmp_pt = new PVector(0, 0);
    for (int x = 0; x < kw; x++) {
      for (int y=0; y < kh; y++) {
        tmp_index = x+y*kw;
        tmp_pt.set(x, y);
        // caclurate the different with depth
        depthDivMap[tmp_index] =  abs(depthMap[tmp_index]-nowbgDepth[tmp_index]);
        
        if (depthMap[tmp_index] >= minDepth && depthMap[tmp_index] <= maxDepth) {
          if ( abs(nowbgDepth[tmp_index]- depthMap[tmp_index]) > mintouchDepth &&  abs(nowbgDepth[tmp_index] - depthMap[tmp_index]) < maxtouchDepth) {
            if (!isInsidePolygon(tmp_pt, area)) {
              touchImg.pixels[tmp_index] = BG_COLOR;
            }else {
              touchImg.pixels[tmp_index] = SENSOR_COLOR;
            }
          }else {
            touchImg.pixels[tmp_index] = BG_COLOR;
          }
          
          if (abs(nowbgDepth[tmp_index]-depthMap[tmp_index])>10) {
            objImg.pixels[tmp_index] = OBJ_COLOR;
          }else {
            objImg.pixels[tmp_index] = BG_COLOR;
          }
          
        } else {
          touchImg.pixels[tmp_index] = BG_COLOR;
          depthImg.pixels[tmp_index] = NOTRANGE_COLOR;
          objImg.pixels[tmp_index] = BG_COLOR;
          subbgImg.pixels[tmp_index] = BG_COLOR;
        }
        
      }  // -- end for(y) -- //
    } // -- end for(x) -- //
    touchImg.updatePixels();
    depthImg.updatePixels();
    objImg.updatePixels();
    
    if (_BG_SAVE && bgsavecount>BG_AvgNums) {
      println("bg_saved");
      updateBGSub();
      TIP_MSG = "";
      _BG_SAVE = false;
    }
    if (_NOWBG_SAVE && nowbgsavecount>BG_AvgNums) {
      println("nowbg_saved");
      updateBGSub();
      TIP_MSG = "";
      _NOWBG_SAVE = false;
    }

    // load the image from gray image
    opencv.loadImage(touchImg);
    
    
    // simple smooth image 
    opencv.blur(8);
    if(_DEBUG) {
      showThumbImg(opencv.getSnapshot(), kw, 0, kw/2, kh/2, true, BORDER_COLOR, "SOURCE BLUR");
    }
    
    // make it as binary image with threshold
    opencv.threshold(threshold); 
    // use image then dilate and erode it to close holes
    opencv.dilate();
    opencv.erode();
    if(_DEBUG) {
      // show the dilate and erode image
      showThumbImg(opencv.getSnapshot(), kw, kh/2, kw/2, kh/2, true, BORDER_COLOR, "BLUR to Binary");
    }
    
    // find contours
    contours.clear();
    contours = opencv.findContours(false, true);
    filterContours(contours, kw*kh/128.0);
    if(_SHOW_SET) {
      image(depthImg, 0, 0, kw, kh);
    }
    
    // draw interactive area
    if(_SHOW_SET) {
      for (int i=0; i<area.length; i++) {
        pushStyle();
        fill(190, 194, 63);
        stroke(190, 194, 63);
        ellipse(area[i].x, area[i].y, 20, 20);
        line(area[i].x, area[i].y, area[(i+1)%area.length].x, area[(i+1)%area.length].y);
        fill(196, 98, 67);
        textAlign(CENTER);
        text(i, area[i].x, area[i].y+3);
        popStyle();
      }
    }

    if(_DEBUG) {
      showThumbImg(touchImg, kw, kh, kw/2, kh/2, true, BORDER_COLOR, "obj image");
      showThumbImg(objImg, kw, kh/2*3, kw/2, kh/2, true, BORDER_COLOR, "sub bg image");
    }
    
    drawContours(sensor_canvas, blobs, 0, 0, 1.0, 1.0, color(255, 255, 0), 3, true, BLOBBG_COLOR, false);
    if(_SHOW_SET) {
      image(sensor_canvas, 0, 0, kw, kh);
    }
    
    // now starting warp sensor image to project view image
    opencv.loadImage(sensor_canvas);
    opencv.gray();
    if(_DEBUG) {
      showThumbImg(opencv.getSnapshot(), kw/2*3, 0, kw/2, kh/2, true, BORDER_COLOR, "warp gray");
    }
    opencv.threshold(threshold); 
    if(_DEBUG) {
      showThumbImg(opencv.getSnapshot(), kw/2*3, kh/2, kw/2, kh/2, true, BORDER_COLOR, "warp threshold");
    }
    opencv.toPImage(warpPerspective(area, kw, kh), warp_canvas);
    opencv.loadImage(warp_canvas);
    contours.clear();
    contours = opencv.findContours(false, true);
    filterContours(contours, kw*kh/128.0);
    PGraphics pg = createGraphics(kw, kh);
    
    drawContours(pg, blobs, 0, 0, 1.0, 1.0, color(200, 200, 0), 3, true, BLOBBG_COLOR, true);
    if(_DEBUG) {
      showThumbImg(pg.get(), kw/2*3, kh, kw/2, kh/2, true, BORDER_COLOR, "warp contours");
    }else {
      if(_SHOW_SET) {
        showThumbImg(pg.get(), kw, 0, kw, kh, true, BORDER_COLOR, "warp contours");
      }
    }
    
    // draw centerpoint from client;
    CenterPointCollect(blobs);
    sendInteraction(local_center);
    drawClientInteractive(client_center);
  
    if(_DRAW_DEMO) {
      for (int i=0; i<blobs.size(); i++) {
        InteractiveContour c = blobs.get(i);
        ArrayList<PVector> cpts = c.getPoints();
        for (int j=0; j<NUM_PTS; j++) {
          float ang = map(j, 0, NUM_PTS, 0, TWO_PI);
          //float ang = (0.01*frameCount + map(j, 0, NUM_PTS, 0, TWO_PI)) % TWO_PI;
          PVector p1 = new PVector(width/2 + width *0.5 * cos(ang), height/2 + width * 0.5 * sin(ang));
          PVector p2 = cpts.get((int) map(j, 0, NUM_PTS-1, 0, cpts.size()-1));
          pts[j] = PVector.lerp(pts[j], p2, LERP_RATE); //new PVector(lerp(pts[j].x, p2.x, LERP_RATE), lerp(pts[j].y, p2.y, LERP_RATE));      
          strokeWeight(3.0 / contours.size());
          stroke(255, 180);
          line(p1.x, p1.y, pts[j].x*wScale, pts[j].y*hScale);      
        }
      }
    }
 
    
    // ----- end the kinect sensing and processing ------- //
    check_timer = millis();
  }
  showTIPMSG();
}


void drawContours(PGraphics p, ArrayList<InteractiveContour> tblob, int xRef, int yRef, float wcaleIn, float hscaleIn, int colorStrokeIn, int strokeWeightIn, boolean fillIn, int colorFillIn, boolean Mode) {
  p.beginDraw();
  for (InteractiveContour b : tblob) {
    if (fillIn)  
      p.fill(colorFillIn); 
    else 
      p.noFill();
    p.stroke(colorStrokeIn);
    p.strokeWeight(strokeWeightIn);

    //contour.setPolygonApproximationFactor(max( (float)contour.getPolygonApproximationFactor()*0.1 , 0.1));
  
    p.beginShape();
    //for (PVector point : contour.getPoints()) {
    for (PVector point : b.getPoints()) {
      p.vertex(point.x, point.y);
    }
    p.endShape();

    if(Mode) {
      p.stroke(200,0,0);
      p.noFill();
      p.stroke(150, 210, 40);
      p.rect(b.getBoundingBox().x, b.getBoundingBox().y, b.getBoundingBox().width, b.getBoundingBox().height);
      
      PVector tmp_center = b.getCenterPoint();
      p.fill(40, 250, 210);
      float tmp_r = sqrt(b.area())/2;
      p.ellipse(tmp_center.x, tmp_center.y, tmp_r, tmp_r);
      
    }

  }
  p.endDraw();
}

void CenterPointCollect(ArrayList<InteractiveContour> tblob) {
  local_center.clear();
  for (InteractiveContour b : tblob) {
    local_center.add( FixScreenPointSet(b.getCenterPoint()) ); 
  }
}

PVector FixScreenPointSet(PVector pt) {
  return new PVector(pt.x*wScale, pt.y*hScale);
}


void filterContours(ArrayList<Contour> cts, float sizeRef) {
  blobs.clear();  
  int count = 0;
  for (int i=cts.size()-1; i>=0; i--) {
    Contour contour = cts.get(i);
    if (contour.area()<sizeRef) {
      cts.remove(i);
    }else {
      count++;
      InteractiveContour itc = new InteractiveContour(contour);
      blobs.add(itc);
    }
  }
}



boolean isInsidePolygon(PVector pos, PVector[] vertices) {
  int i, j=vertices.length-1;
  int sides = vertices.length;
  boolean oddNodes = false;
  for (i=0; i<sides; i++) {
    float tmpi_y = vertices[i].y;
    float tmpi_x = vertices[i].x;
    float tmpj_y = vertices[j].y;
    float tmpj_x = vertices[j].x;
    if ((tmpi_y < pos.y && tmpj_y >= pos.y || tmpj_y < pos.y && tmpi_y >= pos.y) && (tmpi_x <= pos.x || tmpj_x <= pos.x)) {
      oddNodes^=(tmpi_x + (pos.y-tmpi_y)/(tmpj_y - tmpi_y)*(tmpj_x-tmpi_x)<pos.x);
    }
    j=i;
  }
  return oddNodes;
}

void showTIPMSG() {
  int str_l = TIP_MSG.length();
  if(str_l>0) {
    pushStyle();
    fill(255, 177, 27);
    noStroke();
    textSize(20);
    rect(0, 20, str_l*10+20, 40);
    fill(255, 255, 255);
    text(TIP_MSG, 10, 45);
    popStyle();
  }
}

void showThumbImg(PImage pg, int Xref, int Yref, int Wref, int Href, boolean border, int bordercolor, String title) {
  pushMatrix();
    translate(Xref, Yref);
    image(pg, 0, 0, Wref, Href);
    noFill();
    if(border){  
      stroke(bordercolor);
      rect(0, 0, Wref-1, Href-1);
    }
    
    fill(255);
    text(title, 5, 10);
  popMatrix();
}

void depthBGSave(int[] tmpDepth) {
  int tmp_index = 0;
  int [] tDepth = tmpDepth;
  for (int x = 0; x < kw; x++) {
    for (int y=0; y < kh; y++) {
      tmp_index = x+y*kw;
      int tmpdepth = depthMap[tmp_index];
      tDepth[tmp_index] = (tDepth[tmp_index]+tmpdepth)/2;
    }
  }
}

void updateBGSub() {
  subbgImg.loadPixels();
  int tmp_index = 0;
  for (int x = 0; x < kw; x++) {
    for (int y=0; y < kh; y++) {
      tmp_index = x+y*kw;
      int tmpdepth = depthMap[tmp_index];
      if(abs(nowbgDepth[tmp_index]-bgDepth[tmp_index])>10) {
        subbgImg.pixels[tmp_index] = BGSUB_COLOR;
      }else {
        subbgImg.pixels[tmp_index] = BG_COLOR;
      }
    }
  }
  subbgImg.updatePixels();
}


void keyPressed() {
  if (key == ' ') {
    _BG_SAVE = true;
    bgsavecount = 0;
  }
  if (key == 'b') {
    _NOWBG_SAVE = true;
    nowbgsavecount = 0;
  }
  if (key == 'd') {
    _DEBUG = !_DEBUG;
  }
  if (key == 'p') {
    _SHOW_SET = !_SHOW_SET;
  }
  
  if (key>='0' && key<='9') {
    area[key-'0'].set(mouseX, mouseY);
    println(area[key-'0']);
  }
  
  if (key=='s') {
    settings.save();
    println("saving settings.txt");
  }
  
  if (key=='w') {
    _DRAW_DEMO = !_DRAW_DEMO; 
  }
}


Mat getPerspectiveTransformation(PVector[] inputPoints, int w, int h) {
  Point[] canonicalPoints = new Point[4];
  canonicalPoints[0] = new Point(0, 0);
  canonicalPoints[1] = new Point(w, 0);
  canonicalPoints[2] = new Point(w, h);
  canonicalPoints[3] = new Point(0, h);

  MatOfPoint2f canonicalMarker = new MatOfPoint2f();
  canonicalMarker.fromArray(canonicalPoints);

  Point[] points = new Point[4];
  for (int i = 0; i < 4; i++) {
    points[i] = new Point(inputPoints[i].x, inputPoints[i].y);
  }
  MatOfPoint2f marker = new MatOfPoint2f(points);
  return Imgproc.getPerspectiveTransform(marker, canonicalMarker);
}

Mat warpPerspective(PVector[] inputPoints, int w, int h) {
  Mat transform = getPerspectiveTransformation(inputPoints, w, h);
  Mat unWarpedMarker = new Mat(w, h, CvType.CV_8UC1);    
  Imgproc.warpPerspective(opencv.getColor(), unWarpedMarker, transform, new Size(w, h));
  return unWarpedMarker;
}
