#include <cata_sphereo/lanes.h>
//#undef HOUGH_PROB
#define HOUGH_PROB //what? I have no idea...because pthreads are not even being used in this code
//Globals******************
//struct Sstruct S, T;
//struct goalDirection goal;
/*
 void grayscaleImage(IplImage* colorImg, IplImage* grayImg)
 {
 //cvSplit(colorImg, NULL,NULL,NULL,grayImg); //OPENCV1.0!!!SIMPLE: get the blue plane
 cvSplit(colorImg,grayImg,NULL,NULL,NULL);    //OPENCV2.0! How stupid is this???
 //cvSplit(colorImg, NULL,grayImg,NULL,NULL); //OPENCV3.0??
 }
 void binarizeImage(IplImage* grayImg, IplImage* binImg)
 {
 unsigned char threshold;
 threshold=cvGetTrackbarPos("Brightness T","unwrapped");
 cvThreshold(grayImg,binImg,threshold,255,CV_THRESH_BINARY); //apply static threshold to blue plane
 //cvCanny(binImg,binImg,50,50,3); //perform edge detection on the binary image to get countours
 }
 */
void
getSegmentSequence(IplImage* img_bin, struct line_segments* segments)
{
  int threshold = 21; // TODO: get from a trackbar
  //  threshold = cvGetTrackbarPos("Hough T", "unwrapped");

#ifndef HOUGH_PROB

  lines = cvHoughLines2( img_bin, segment_endpoints_storage, CV_HOUGH_STANDARD, 1, CV_PI/180, threshold, 0, 0 );
#else
  segments->endpoints = cvHoughLines2(img_bin, segment_endpoints_storage,
      CV_HOUGH_PROBABILISTIC, 1, CV_PI / 180, threshold, 50, 30);
  segments->compute_params();
#endif
}
/*
 void drawSegment(IplImage* img, struct segment s, CvScalar color, int thickness, int mode)
 {
 #ifndef HOUGH_PROB
 float* L = (float*)line;
 float rho = L[0];
 float theta = L[1];
 CvPoint pt1, pt2;
 double a = cos(theta), b = sin(theta);
 double x0 = a*rho, y0 = b*rho;
 pt1.x = cvRound(x0 + 1000*(-b));
 pt1.y = cvRound(y0 + 1000*(a));
 pt2.x = cvRound(x0 - 1000*(-b));
 pt2.y = cvRound(y0 - 1000*(a));
 cvLine( img, pt1, pt2, color, 2, 8 );
 //printf("rho:%f,theta:%f\n",rho,theta/CV_PI*180);

 #else
 CvFont font;
 double hScale=.33;
 double vScale=.33;
 int    lineWidth=1;
 cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX, hScale,vScale,0,lineWidth);
 char str[50];


 //	cvLine( img,  s.endpoints[0], s.endpoints[1], color, (int)(s.weight*20+1), 8 );
 cvLine( img,  s.endpoints[0], s.endpoints[1], color, thickness, 8, 0);

 cvCircle(img, s.endpoints[0], 2, color, thickness, 8, 0);
 cvCircle(img, s.endpoints[1], 2, color, thickness, 8, 0);
 //sprintf(str, "t=%f, r=%f", s.params.theta, s.params.rho);
 if(mode==DEBUG_MODE)
 {
 sprintf(str, "%2.2f", s.params.theta);
 cvPutText(img, str, getSegmentMidpoint(s), &font, color);
 }
 #endif

 }
 void drawWeightedSegment(IplImage* img, struct segment s, CvScalar color, int mode)
 {
 int thickness;
 thickness=10/(s.weight+1);
 if(thickness<0)
 thickness=0;
 else if (thickness>255)
 thickness=255;
 drawSegment(img, s, color, thickness, mode);
 }
 void drawSegmentSequence(IplImage* img, struct line_segments* segments, CvScalar color, int thickness, int mode)
 {
 const int minLineCount=200;
 for(int i=0;i<MIN(segments->endpoints->total,minLineCount);i++)
 drawSegment(img, segments->getSegment(i), color, thickness, mode);
 }
 void drawWeightedSegmentSequence(IplImage* img, struct line_segments* segments, CvScalar color, int mode)
 {
 const int minLineCount=200;
 for(int i=0;i<MIN(segments->endpoints->total,minLineCount);i++)
 drawWeightedSegment(img, segments->getSegment(i), color, mode);
 }
 */
CvPoint
getSegmentMidpoint(struct segment s)
{
  CvPoint midpoint;
  midpoint.x = 0.5 * (s.endpoints[0].x + s.endpoints[1].x);
  midpoint.y = 0.5 * (s.endpoints[0].y + s.endpoints[1].y);
  return midpoint;
}

struct lane
getLaneHypothesis(struct line_segments* segments)
{
  struct segment random_segments[2];
  return getLaneHypothesis(segments, random_segments);
}
struct lane
getLaneHypothesis(struct line_segments* segments,
    struct segment* random_segments)
{
  int r1 = 0;
  int r2 = 1;
  //printf("%d\n", segments->endpoints->total);
  if (segments->endpoints->total > 1)
    {
      r1 = rand() % segments->endpoints->total;
      do
        {
          r2 = rand() % segments->endpoints->total;
        }
      while (r2 == r1);
    }
  else
    return lane();

  random_segments[0] = segments->getSegment(r1);
  random_segments[1] = segments->getSegment(r2);

  return lane(random_segments);
}

float
getSegmentTheta(CvPoint* endpoints)
{
  CvPoint p1 = endpoints[0];
  CvPoint p2 = endpoints[1];

  float dx = p1.x - p2.x;
  float dy = p1.y - p2.y;

  if (dx == 0 && dy == 0)
    return -100.0;
  else if (dx == 0 && dy > 0)
    return CV_PI / 2;
  else if (dx == 0 && dy < 0)
    return -CV_PI / 2;
  else if (dx > 0 && dy == 0)
    return 0;
  else if (dx < 0 && dy == 0)
    return 0;
  else if ((dx > 0 && dy > 0) || (dx < 0 && dy < 0)) //both same
    return -atan(dy / dx);
  else if ((dx > 0 && dy < 0) || (dx < 0 && dy > 0)) //both different
    return atan(-dy / dx);
}

void
drawDirectionArrow(IplImage* img, CvPoint p_base, int arrow_length,
    float direction, CvScalar color, int thickness)
{
  CvPoint p_tip, p_head_0, p_head_1;

  int tip_length = (int) (arrow_length * 0.3);
  p_tip = cvPoint(p_base.x - arrow_length * cos(direction),
      p_base.y + arrow_length * sin(direction));

  float head_theta_0, head_theta_1;
  head_theta_0 = direction - 0.4;
  head_theta_1 = direction + 0.4;

  p_head_0 = cvPoint(p_tip.x + tip_length * cos(head_theta_0),
      p_tip.y - tip_length * sin(head_theta_0));
  p_head_1 = cvPoint(p_tip.x + tip_length * cos(head_theta_1),
      p_tip.y - tip_length * sin(head_theta_1));

  cvLine(img, p_base, p_tip, color, thickness, 8, 0);
  cvLine(img, p_tip, p_head_0, color, thickness, 8, 0);
  cvLine(img, p_tip, p_head_1, color, thickness, 8, 0);

  cvCircle(img, p_base, 3, color, 2, 8, 0);
}
/*
 void
 drawGoalArrow(IplImage* colorImg, struct goalDirection goal)
 {
 float direction;
 direction = goal.angle;
 int maxScore = 20;
 int thickness;
 if (goal.score > maxScore)
 thickness = 10;
 else
 thickness = (int) ((float) goal.score / (float) maxScore * 10);

 drawDirectionArrow(colorImg, cvPoint(160, 120), 100, goal.angle + CV_PI,
 COLOR_ORANGE, thickness);
 //	drawDirectionArrow(colorImg, cvPoint(160,120), 100, goal.angle, COLOR_ORANGE, thickness);

 }
 */



float
angleDiff(float t0, float t1)
{
  return fabs(asin(sin(t1 - t0)));
}
float
angleDiffSigned(float t0, float t1)
{
  return (asin(sin(t1 - t0)));
}
float
angleDiffHalf(float t0, float t1)
{
  return fabs(2 * asin(sin(0.5 * (t1 - t0))));
}
float
angleDiffHalfSigned(float t0, float t1)
{
  return 2 * asin(sin(0.5 * (t1 - t0)));
}
float
getAngleBetweenPoints(CvPoint p0, CvPoint p1)
{
  return 0.5 * CV_PI + atan2((float) (p0.x - p1.x), (float) (p0.y - p1.y));
}
CvPoint
getLineIntersection(struct segment* s)
{
  return getLineIntersection(s[0].params, s[1].params);
}
CvPoint
getLineIntersection(struct line_params p0, struct line_params p1)
{
  float A[4];
  float b[2];
  float x[2];
  CvPoint p;
  A[0] = sin(p0.theta);
  A[1] = cos(p0.theta);
  A[2] = sin(p1.theta);
  A[3] = cos(p1.theta);

  b[0] = p0.rho;
  b[1] = p1.rho;

  CvMat Amat = cvMat(2, 2, CV_32FC1, A);
  CvMat xmat = cvMat(2, 1, CV_32FC1, x);
  CvMat bmat = cvMat(2, 1, CV_32FC1, b);
  cvSolve(&Amat, &bmat, &xmat);
  p.x = cvRound(x[0]);
  p.y = cvRound(x[1]);
  return p;
}

float
getDirectionWeight(struct line_params params, float orientation)
{
  //assigns [0 1] weight depending on angular difference between a segment and angle
  return 1 - angleDiff(params.theta, orientation) / (CV_PI / 2);
}

struct segment_metric
getSegment2LaneMetric(struct segment s, struct lane L)
{
  struct line_params Np[2]; //normal lane parameters
  struct line_params Lp[2]; //lane parameters
  CvPoint pp[2];
  CvPoint pint;
  CvPoint mp;
  float d[2];
  float dist;
  int index;
  struct segment distance_segment;
  struct segment_metric metric;
  mp = getSegmentMidpoint(s);

  //obtain parameters of line L
  L.getParams(Lp);

  //obtain parameters of a line normal to lane L
  Np[0].theta = L.t0 + CV_PI / 2;
  Np[1].theta = L.t1 + CV_PI / 2;
  Np[0].rho = getRho(mp.x, mp.y, Np[0].theta);
  Np[1].rho = getRho(mp.x, mp.y, Np[1].theta);

  pp[0] = getLineIntersection(Lp[0], Np[0]);
  pp[1] = getLineIntersection(Lp[1], Np[1]);

  d[0] = getDist(mp, pp[0]);
  d[1] = getDist(mp, pp[1]);

  index = (d[0] < d[1]) ? 0 : 1;

  if (L.isPointOnLane(pp[0]) < 0 && L.isPointOnLane(pp[1]) < 0)
    {
      pint = L.fulcrum;
      index = -1;
    }
  else if (index == L.isPointOnLane(pp[index]))
    pint = pp[index];
  else
    {
      if (index == 1)
        {
          pint = pp[0];
          index = 0;
        }
      else if (index == 0)
        {
          pint = pp[1];
          index = 1;
        }
    }

  if (index > -1)
    {
      metric.dtheta = angleDiff(Lp[index].theta, s.params.theta);
    }
  else
    {
      float dtheta0 = angleDiff(Lp[0].theta, s.params.theta);
      float dtheta1 = angleDiff(Lp[1].theta, s.params.theta);
      metric.dtheta = MIN(dtheta0, dtheta1);
    }
  dist = getDist(mp, pint);
  metric.dist = dist;

  distance_segment.endpoints[0] = pint;
  distance_segment.endpoints[1] = mp;
  distance_segment.params = line_params(mp, pint);
  distance_segment.weight = dist;

  metric.distance_segment = distance_segment;

  return metric;
}
/*
 void drawDistanceSegmentSequence(IplImage* img, struct line_segments* segments, struct lane L)
 {
 struct segment distance_segment;
 struct segment_metric metric;
 CvPoint pint = metric.distance_segment.endpoints[0];
 for(int i=0;i<segments->endpoints->total;i++)
 {
 metric=getSegment2LaneMetric(segments->getSegment(i), L);

 if(L.isPointOnLane(pint)==1)
 {
 drawSegment(img, metric.distance_segment, COLOR_GREEN,5);
 cvCircle(img, pint, 3, COLOR_GREEN, 3);
 }
 else if (L.isPointOnLane(pint)==0)
 {
 drawSegment(img, metric.distance_segment, COLOR_RED,5);
 cvCircle(img, pint, 3, COLOR_RED, 3);
 }
 else
 {
 drawSegment(img, metric.distance_segment, COLOR_YELLOW,5);
 cvCircle(img, pint, 3, COLOR_YELLOW, 3);
 }
 }
 }
 */
void
removeSegmentsOnLane(struct line_segments* segments, lane L, int threshold)
{
  struct segment s;
  int i = 0;
  while (i < segments->endpoints->total)
    {
      s = segments->getSegment(i);
      if (getSegment2LaneMetric(s, L).dist < threshold)
        {
          cvSeqRemove(segments->endpoints, i);
          cvSeqRemove(segments->weights, i);
          cvSeqRemove(segments->params, i);
        }
      else
        i++;
    }
}
/*
 int getLaneSimilarityWeight(struct lane L, struct lanes Lanes)
 { //provides a measure of similarity of a lane against two lanes
 return 0;
 }
 int getLaneSimilarity(struct lane L0, struct lane L1)
 { //provides a measure of similarity between two lanes
 return 0;
 }
 int scoreLaneHypothesis(struct line_segments* segments, struct lane L)
 {
 for(int i=0; i<segments->endpoints->total; i++)
 {
 //getDirectionWeight(segments->getSegment(i));
 }
 return 0;
 }
 */

struct lane
findLaneRANSAC(IplImage* img, struct line_segments* segments, int mode,
    int numOfIterations)
{
  struct lane hypothesis_lane;
  struct lane best_lane_so_far;
  struct segment hypothesis_segments[2];
  struct segment test_segment;
  struct segment_metric metric;
  struct lanes lanes_output;
  int best_score_so_far = 0;

#ifdef DEBUG_MODE
  if(mode==DEBUG_MODE)
  drawWeightedSegmentSequence(img, segments, COLOR_DARKBLUE);
#endif
  for (int i = 0; i < numOfIterations; i++)
    {
      hypothesis_lane = getLaneHypothesis(segments, hypothesis_segments);

#ifdef DEBUG_MODE
      printf("=========================\n%d/%d RANSAC iteration\n=========================\n", i, numOfIterations);
      printf("Opening angle: %f\n", RAD2DEG*hypothesis_lane.getInternalAngle());
#endif
      if ((RAD2DEG * hypothesis_lane.getInternalAngle()) > 110)
        {
#ifdef DEBUG_MODE
          laneImg = cvCloneImage(img);
          drawLane(laneImg, hypothesis_lane);
          //	drawLane(laneImg, best_lane_so_far, COLOR_RED);
          drawSegment(laneImg, hypothesis_segments[0], COLOR_GREEN, 2, DEBUG_MODE);
          drawSegment(laneImg, hypothesis_segments[1], COLOR_GREEN, 2, DEBUG_MODE);
          cvShowImage("unwrapped", laneImg);
#endif
          int score = 0;
          for (int j = 0; j < segments->endpoints->total; j++)
            {
              test_segment = segments->getSegment(j);
              metric = getSegment2LaneMetric(test_segment, hypothesis_lane); //computes distance of segment to Lane
              if (metric.dist < 40 && (RAD2DEG) * metric.dtheta < 20.0)
                score++;
              //score=score+10/(1+test_segment.weight);

#ifdef DEBUG_MODE
              //score-=metric.dist+7*180/CV_PI*metric.dtheta;
              printf("%d/%d scoring iteration: ", j, segments->endpoints->total);
              segmentImg = cvCloneImage(laneImg);
              drawSegment(segmentImg, test_segment, COLOR_BLUE, 4, DEBUG_MODE);
              drawSegment(segmentImg, metric.distance_segment, COLOR_YELLOW);
              printf("W: %f dist: %f, theta: %f score: %d\n ", test_segment.weight, metric.dist, metric.dtheta, score);
              cvShowImage("unwrapped", segmentImg);
              cvWaitKey(10000);
              cvReleaseImage(&segmentImg);
#endif
            }//!for(j)
          if (score > best_lane_so_far.score)
            {
              best_lane_so_far = hypothesis_lane;
              best_lane_so_far.score = score;
            }
#ifdef DEBUG_MODE
          printf(
              "*********************************\nSCORE: %d\n*********************************\n\n",
              score);
          cvReleaseImage(&laneImg);
#endif
        }//!if
    }//!for(i)
#ifdef DEBUG_MODE
  printf(
      "\n\n++++++++++++++++++++++++++++++++\nBest Score: %d \n++++++++++++++++++++++++++++++++\n\n",
      best_lane_so_far.score);
#endif

  return best_lane_so_far;
}//!RANSAC
struct lanes
findLanesRANSAC(IplImage *img, struct line_segments* segments, int mode,
    int numOfIterations, float scoreThreshold)
{
  IplImage* imgCopy = NULL;
#ifdef DEBUG_MODE
  imgCopy = cvCloneImage(img);
  drawWeightedSegmentSequence(imgCopy, segments, COLOR_DARKBLUE);
  cvShowImage("unwrapped", imgCopy);
#endif
  struct lanes Lanes;

  for (int i = 0; i < Lanes.num_of_lanes; i++)
    {
      Lanes.Lane[i] = findLaneRANSAC(imgCopy, segments, mode, numOfIterations);

      if (Lanes.Lane[i].score > scoreThreshold)
        {
          Lanes.Lane[i].isVisible = true;
          removeSegmentsOnLane(segments, Lanes.Lane[i], 50);
        }
      else
        Lanes.Lane[i].isVisible = false;
    }

  return Lanes;
}
float
getRho(int x, int y, float theta)
{
  return y * cos(theta) + x * sin(theta);
}
float
getDist(CvPoint* p)
{
  return getDist(p[0], p[1]);
}
float
getDist(CvPoint p0, CvPoint p1)
{
  float dx, dy;
  dx = p0.x - p1.x;
  dy = p0.y - p1.y;
  return (float) sqrt((float) (dx * dx + dy * dy));
}
/*
 struct goalDirection getGoalDirectionFromSegmentSequence(struct line_segments* segments)
 {
 struct goalDirection goal;
 float thetaSum=0;
 float thetaMean=0;
 float var, std;
 float M;
 float oldM;
 float S;
 float theta;
 if(segments->endpoints->total>1)
 {
 M=segments->getSegment(0).params.theta;
 oldM=M;
 thetaSum = segments->getSegment(0).params.theta;
 S=0;
 var=0;
 for(int i=1; i<segments->endpoints->total; i++)
 {
 //thetaSum+=segments->getSegment(i).params.theta;
 theta = segments->getSegment(i).params.theta;
 M = oldM + (theta-oldM)/(i+1);
 S = S + (theta-oldM)*(theta-M);
 //M = oldM + angleDiff(theta,oldM)/(i+1);
 //S = S + angleDiff(theta,oldM)*angleDiff(theta,M);
 var = S/(i);
 oldM=M;
 thetaSum+=theta;
 }
 printf("N=%d, thetaAvg=%f, thetaAvg2=%f, var=%f\n", segments->endpoints->total, RAD2DEG*M, RAD2DEG*(thetaSum/segments->endpoints->total), RAD2DEG*sqrt(fabs(var)) );
 goal.angle = M;
 goal.variance = var;
 goal.score = segments->endpoints->total;
 }
 if(segments->endpoints->total>1)
 {
 float Z;
 for(int i=0; i<segments->endpoints->total; i++)
 {
 theta = segments->getSegment(i).params.theta;
 //Z = fabs(angleDiff(M,theta))/sqrt(fabs(var));
 Z = (RAD2DEG*M-RAD2DEG*theta)/(RAD2DEG*sqrt(fabs(var)));
 printf("i=%d, theta=%f, M=%f, Z=%f\n", i, RAD2DEG*theta, RAD2DEG*M, Z);
 }
 }

 return goal;
 }
 struct goalDirection getGoalDirectionFromSegmentSequenceRANSAC(struct line_segments* segments)
 {
 int r=0;
 //printf("%d\n", segments->endpoints->total);]
 struct goalDirection goal;
 float hypothesis_theta, theta, theta_score;
 float error_tolerance = DEG2RAD*3;
 int numOfTrials=50;
 int best_r_so_far;
 float best_theta_score_so_far=65535;

 int score;
 int best_score_so_far = 0;

 if(segments->endpoints->total>0)
 {
 //for(int trial=0; trial<numOfTrials; trial++)
 for(int trial=0; trial<segments->endpoints->total; trial++)
 {
 r = trial;
 //r=rand()%segments->endpoints->total;
 hypothesis_theta = segments->getSegment(r).params.theta;
 theta_score=0; score=0;
 //printf("=======================================\n");
 //printf("trial=%d, seg=%d, hyp_angle=%f\n", trial, r, hypothesis_theta);
 for(int i=0; i<segments->endpoints->total; i++)
 {
 theta = segments->getSegment(i).params.theta;
 theta_score+=fabs(angleDiff(theta, hypothesis_theta));
 //	if( fabs(angleDiff(theta, hypothesis_theta))<error_tolerance )
 //		score++;
 //printf("segment=%d, theta_score=%f\n", i, theta_score);
 //	printf("segment=%d, score=%d\n", i, score);
 }

 if(theta_score<best_theta_score_so_far)
 {
 best_theta_score_so_far = theta_score;
 best_r_so_far=r;
 }

 //				if(score>best_score_so_far)
 //				{
 //					best_score_so_far = score;
 //					best_r_so_far=r;
 //				}


 //printf("********************************************\n");
 //printf("best_score:=%f, angle=%f\n", best_theta_score_so_far, segments->getSegment(best_r_so_far).params.theta);
 //printf("********************************************\n");
 //printf("********************************************\n");
 //printf("best_score:=%d, angle=%f\n", best_score_so_far, segments->getSegment(best_r_so_far).params.theta);
 //printf("********************************************\n");

 }
 //printf("+++++++++++++++++++++++++++++++++++++++++++++++\n");
 //printf("BEST_SCORE=%d\n", best_score_so_far);
 goal.angle = segments->getSegment(best_r_so_far).params.theta;
 goal.score = segments->endpoints->total;
 goal.variance = 0;
 }
 return goal;
 }
 */
