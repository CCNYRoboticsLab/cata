#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <stdio.h>
#include "time.h"

#define RELEASE_MODE 0
//#define DEBUG_MODE

#define LANES_COLOR CV_RGB(0,0,255)
#define DEG2RAD CV_PI/180
#define RAD2DEG 180/CV_PI

#define COLOR_RED CV_RGB(255, 0, 0)
#define COLOR_BLUE CV_RGB(0, 0, 255)
#define COLOR_DARKBLUE CV_RGB(0,0,100)
#define COLOR_GREEN CV_RGB(0,255,0)
#define COLOR_MAGENTA CV_RGB(255,0,255)
#define COLOR_YELLOW CV_RGB(255,255,0)
#define COLOR_ORANGE CV_RGB(255,128,0)
#define COLOR_BLACK CV_RGB(0,0,0)
#define COLOR_GRAY  CV_RGB(128,128,128)
#define COLOR_WHITE CV_RGB(255,255,255)

/*
 struct goalDirection
 {
 float angle;
 int score;
 float variance;

 goalDirection()
 {
 angle=0;
 score=0;
 variance=0;
 }
 };
 struct Sstruct
 {
 int a;
 int b;
 int len; //length of string str
 char s[20];
 int H; //H of D
 int W; //W of D
 int D[10][10];
 };
 */

static CvMemStorage* segment_endpoints_storage = cvCreateMemStorage(0);
static CvMemStorage* segment_params_storage = cvCreateMemStorage(0);
static CvMemStorage* segment_weights_storage = cvCreateMemStorage(0);

//void grayscaleImage(IplImage* colorImg, IplImage* grayImg);
//void binarizeImage(IplImage* grayImg, IplImage* binImg);
void
getSegmentSequence(IplImage *img_bin, struct line_segments* segments);
//void drawSegment(IplImage* img, struct segment s, CvScalar color, int thickness=2, int mode=RELEASE_MODE);
//void drawWeightedSegment(IplImage* img, struct segment s, CvScalar color, int mode=RELEASE_MODE);
//void drawSegmentSequence(IplImage* img, struct line_segments* segments, CvScalar color, int thickness=2, int mode=RELEASE_MODE);
//void drawWeightedSegmentSequence(IplImage* img, struct line_segments* segments, CvScalar color, int mode=RELEASE_MODE);
struct lane
getLaneHypothesis(struct line_segments* segments);
struct lane
getLaneHypothesis(struct line_segments* segments,
    struct segment* random_segments);
float
getSegmentTheta(CvPoint* endpoints);
//void enterSegmentSequence(struct line_segments* segments, int N);
CvPoint
getSegmentMidpoint(struct segment s);
float
angleDiff(float t0, float t1);
float
angleDiffSigned(float t0, float t1);
float
angleDiffHalf(float t0, float t1);
float
angleDiffHalfSigned(float t0, float t1);
float
getAngleBetweenPoints(CvPoint p0, CvPoint p1);
CvPoint
getLineIntersection(struct segment* s);
CvPoint
getLineIntersection(struct line_params p0, struct line_params p1);
float
getDirectionWeight(struct line_params params, float orientation);
struct segment_metric
getSegment2LaneMetric(struct segment s, struct lane L);
//void drawDistanceSegmentSequence(IplImage* img, struct line_segments* segments, struct lane L);
float
getRho(int x, int y, float theta);
float
getDist(CvPoint* p);
float
getDist(CvPoint p0, CvPoint p1);
void
drawDirectionArrow(IplImage* img, CvPoint p_base, int arrow_length,
    float direction, CvScalar color = COLOR_ORANGE, int thickness = 3);
//void drawGoalArrow(IplImage* colorImg, struct goalDirection goal);

struct line_params
{
    float theta;
    float rho;
    line_params(CvPoint p0, CvPoint p1)
    {
      CvPoint pts[2];
      pts[0] = p0;
      pts[1] = p1;
      compute_params(pts);
    }
    line_params(CvPoint* pts)
    {
      compute_params(pts);
    }
    void
    compute_params(CvPoint *pts)
    {
      theta = getSegmentTheta(pts);
      rho = getRho(pts[0].x, pts[0].y, theta);
    }
    line_params()
    {
      theta = rho = 0.0;
    }
};
struct segment
{
    CvPoint endpoints[2];
    struct line_params params;
    float weight;
    segment()
    {
    }
};
struct segment_metric
{
    float dist; //distance of midpoint to Lane
    float dtheta; //angle error
    struct segment distance_segment;
};

struct lane
{
    CvPoint fulcrum; //intersection
    float t0, t1; //angles
    bool isVisible;
    int score;
    lane()
    {
      isVisible = false;
      score = 0;
    }
    lane(struct segment *s, int argScore = 0)
    { //constructs lane from two segments
      isVisible = true;
      CvPoint m[2];
      m[0] = getSegmentMidpoint(s[0]);
      m[1] = getSegmentMidpoint(s[1]);
      fulcrum = getLineIntersection(s);
      t0 = getAngleBetweenPoints(m[0], fulcrum);
      t1 = getAngleBetweenPoints(m[1], fulcrum);
      //printf("angle=%f\n", angleDiffHalf(t0,t1)*180.0/CV_PI);
      score = argScore;
    }
    lane(float arg_t0, float arg_t1, CvPoint arg_fulcrum, int argScore = 0)
    {
      t0 = arg_t0;
      t1 = arg_t1;
      fulcrum = arg_fulcrum;
      score = argScore;
      isVisible = false;
    }
    void
    getParams(struct line_params* params)
    {
      params[0].theta = t0;
      params[0].rho = getRho(fulcrum.x, fulcrum.y, t0);
      params[1].theta = t1;
      params[1].rho = getRho(fulcrum.x, fulcrum.y, t1);
    }
    float
    getInternalAngle()
    {
      return angleDiffHalf(t0, t1);
    }
    int
    isPointOnLane(CvPoint p)
    {
      float dt;
      dt = getAngleBetweenPoints(p, fulcrum);
      if (angleDiffHalf(t0, dt) < 0.1)
        return 0;
      else if (angleDiffHalf(t1, dt) < 0.1)
        return 1;
      else
        return -1;
    }
};

struct lanes
{
    static const int num_of_lanes = 1;
    lane Lane[num_of_lanes];
    float direction;
    lanes()
    {
      // TODO: Automize computation
      direction = CV_PI / 2; // TODO: use proper direction (I think this time lanes are vertical (not horizontal as 0)
      if (num_of_lanes == 1)
        Lane[0] = lane(direction, direction + CV_PI, cvPoint(160, 120)); // TODO: Assign proper point arg values (I'm picking midpoints 150px from center on each side)
      else if (num_of_lanes == 2)
        {
          Lane[0] = lane(direction, direction + CV_PI, cvPoint(80, 120)); // TODO: Assign proper arg_fulcrum points (I'm picking midpoints 150px from center on each side)
          Lane[1] = lane(direction, direction + CV_PI, cvPoint(260, 120));
        }
    }
    lanes(struct lane* argLane)
    {
      for (int i = 0; i < num_of_lanes; i++)
        Lane[i] = argLane[i];
    }
    lanes(struct lane argLane0, struct lane argLane1)
    {
      if (num_of_lanes == 2)
        {
          Lane[0] = argLane0;
          Lane[1] = argLane1;
        }
      else if (num_of_lanes == 1)
        Lane[0] = argLane0;

    }
};
struct line_segments
{
    CvSeq* endpoints; //segment container
    CvSeq* params; //segment line equation params (rho, theta)
    CvSeq* weights; //segment weights
    void
    compute_params()
    {
      struct line_params p;
      CvPoint* pts;
      for (int i = 0; i < endpoints->total; i++)
        {
          pts = (CvPoint*) cvGetSeqElem(endpoints, i);
          p = line_params(pts);
          cvSeqPush(params, &p);
        }
    }
    void
    compute_weights(struct lanes Lanes)
    { //compute weight of each segment relative to desired orientation (much simpler)
      float w;
      for (int i = 0; i < endpoints->total; i++)
        {
          struct segment_metric metric_candidate[Lanes.num_of_lanes];
          struct segment_metric theMetric;

          for (int j = 0; j < Lanes.num_of_lanes; j++)
            metric_candidate[j] = getSegment2LaneMetric(getSegment(i),
                Lanes.Lane[j]);

          if (Lanes.num_of_lanes == 2)
            theMetric
                = (metric_candidate[0].dist < metric_candidate[1].dist) ? metric_candidate[0]
                    : metric_candidate[1];
          else
            theMetric = metric_candidate[0];

          theMetric.dist = 1;
          theMetric.dtheta = 0.1;
          w = (theMetric.dist + 10 * RAD2DEG * theMetric.dtheta);
          cvSeqPush(weights, &w);
        }
    }
    void
    init()
    {
      params = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq),
          sizeof(line_params), segment_params_storage);
      weights = cvCreateSeq(CV_SEQ_ELTYPE_GENERIC, sizeof(CvSeq),
          sizeof(float), segment_weights_storage);
    }
    struct segment
    getSegment(int index)
    {
      struct segment s;
      s.endpoints[0] = *((CvPoint*) cvGetSeqElem(endpoints, index));
      s.endpoints[1] = *((CvPoint*) cvGetSeqElem(endpoints, index) + 1);
      s.params = *((struct line_params*) cvGetSeqElem(params, index));
      if (index < weights->total)
        s.weight = *((float*) cvGetSeqElem(weights, index));
      else
        s.weight = -1000.0;
      return s;
    }
};

void
removeSegmentsOnLane(struct line_segments* segments, lane L, int threshold);
struct lane
findLaneRANSAC(IplImage* img, struct line_segments* segments,
    int mode = RELEASE_MODE, int numOfIterations = 30);
struct lanes
    findLanesRANSAC(IplImage *img, struct line_segments* segments,
        int mode = RELEASE_MODE, int numOfIterations = 30,
        float scoreThreshold = 3);
