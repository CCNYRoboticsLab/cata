/*
 * common.h
 *
 *  Created on: Jan 11, 2011
 *      Author: carlos
 */

#ifndef COMMON_H_
#define COMMON_H_

const int FORWARD = 1;
const int BACKWARD = -1;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <iostream>
#include <vector>
//#include <ctype.h>
//#include <stdio.h>

void
printMatrix(cv::Mat &theMatrix);
void
markCorners(cv::Mat &imageMat, std::vector<cv::Point2f> pointsOnImage);
void
drawDirectionArrow(cv::Mat &img, cv::Point2f &p_base, cv::Point2f &p_end,
    cv::Scalar color, int thickness, int lineType = 8);
void
drawOFVectors(cv::Mat &imageMat, std::vector<cv::Point2f> &pointsBegin,
    std::vector<cv::Point2f> &pointsEnd, std::vector<uchar> &status);
void
addCornersToVector(std::vector<cv::Point2f> &target, const std::vector<
    cv::Point2f> &additional);
int
gloabalHeadingFromOF(const cv::Mat &oldImg, const cv::Mat &curImg,
    cv::Mat &dst, std::vector<cv::Mat> &masks, bool color);

void
printMatrix(cv::Mat &theMatrix)
{
  std::cout << "[" << std::endl;
  for (int i = 0; i < theMatrix.rows; i++)
    {
      for (int j = 0; j < theMatrix.cols; j++)
        std::cout << theMatrix.at<float> (i, j) << ", ";
      std::cout << std::endl;
    }
  std::cout << "]" << std::endl;
}

/** @brief computes the global heading direction (backward/forward) using sparse optical flow vectors from the image 4 quadrants
 @param dst The canvas image where to draw the computed optical flow
 @return >0: FORWARD, <0: BACKWARD
 **/
int
gloabalHeadingFromOF(const cv::Mat &oldImg, const cv::Mat &curImg,
    cv::Mat &dst, std::vector<cv::Mat> &masks, bool color)
{
  // TEST images:
  cv::namedWindow("Old Img", CV_WINDOW_AUTOSIZE);
  cv::imshow("Old Img", oldImg);
  cv::namedWindow("New Img", CV_WINDOW_AUTOSIZE);
  cv::imshow("New Img", curImg);
  //---------------------------------------------------
  // Feature tracking parameters:
  std::vector<cv::Point2f> cornersOld; // To store features being tracked first
  std::vector<cv::Point2f> cornersCurr; // To store features being tracked on sequential frame
  std::vector<cv::Point2f> cornersTemp; // To store features extracted from each quadrant
  // Other parameters used in tracking
  int maxCorners = 10;
  double qualityLevel = 0.1; // indicates the minimal acceptable lower eigenvalue for a point to be included as a corner.
  // The actual minimal eigenvalue used for the cutoff is the product of the quality_level and the largest lower
  // eigenvalue observed in the image. Hence, the quality_level should not exceed 1 (a typical value might be 0.10 or 0.01)
  double minDistance = 1.0; //guarantees that no two returned points are within the indicated number of pixels.
  int blockSize = 3;
  bool useHarrisDetector = true;
  double k = 0.04;
  //---------------------------------------------------

  //---------------------------------------------------
  // Fixed arguments for calcOpticalFlowPyrLK
  std::vector<uchar> status;
  std::vector<float> err;
  cv::Size winSize = cv::Size(3, 3);
  int maxLevel = 2;

  //---------------------------------------------------
  // Search in each quadrant:
  //-------------------------------------------
  int concensusNE = 0;
  int concensusNW = 0;
  int concensusSW = 0;
  int concensusSE = 0;
  uint cornersNE = 0;
  uint cornersNW = 0;
  uint cornersSW = 0;
  uint cornersSE = 0;
  uint cIdx = 0;

  int counterValid = 0;
  float xValues = 0.0;
  float yValues = 0.0;

  cv::Mat oldImgBW;
  if (color)
    cv::cvtColor(oldImg, oldImgBW, CV_BGR2GRAY); // convert frame to grayscale
  else
    oldImgBW = oldImg;

  if (masks.size() == 4)
    {
      // First quadrant NE (North East = upper right)
      cv::goodFeaturesToTrack(oldImgBW, cornersTemp, maxCorners, qualityLevel,
          minDistance, masks[0], blockSize, useHarrisDetector, k);
      cornersNE = cornersTemp.size();
      addCornersToVector(cornersOld, cornersTemp);

      // Second quadrant NW (North West = upper left)
      cv::goodFeaturesToTrack(oldImgBW, cornersTemp, maxCorners, qualityLevel,
          minDistance, masks[1], blockSize, useHarrisDetector, k);
      addCornersToVector(cornersOld, cornersTemp);
      cornersNW = cornersOld.size();

      // Third quadrant SW (South West = lower left)
      cv::goodFeaturesToTrack(oldImgBW, cornersTemp, maxCorners, qualityLevel,
          minDistance, masks[2], blockSize, useHarrisDetector, k);
      addCornersToVector(cornersOld, cornersTemp);
      cornersSW = cornersOld.size();

      // Fourth quadrant SE (South East = lower right)
      cv::goodFeaturesToTrack(oldImgBW, cornersTemp, maxCorners, qualityLevel,
          minDistance, masks[3], blockSize, useHarrisDetector, k);
      addCornersToVector(cornersOld, cornersTemp);
      cornersSE = cornersOld.size();
    }
  else // Not using masks
    {
      cv::goodFeaturesToTrack(oldImgBW, cornersTemp, maxCorners, qualityLevel,
          minDistance, cv::Mat(), blockSize, useHarrisDetector, k);
      addCornersToVector(cornersOld, cornersTemp);
    }
  // This termination criteria tells the algorithm to stop when it has either done
  // # iterations or when epsilon is better than 0.3
  // You can play with these parameters for speed vs. accuracy but these values work pretty well in many situations.
  cv::TermCriteria optical_flow_termination_criteria = cv::TermCriteria(
      CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, .3);
  // Default is:
  //  cv::TermCriteria criteria=cv::TermCriteria( TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  // Find matches for all corners (features being tracked)
  cv::calcOpticalFlowPyrLK(oldImg, curImg, cornersOld, cornersCurr, status,
      err, winSize, maxLevel, optical_flow_termination_criteria);
  //  cv::calcOpticalFlowPyrLK(oldImg, curImg, cornersOld, cornersCurr, status,
  //      err, winSize, maxLevel);

  drawOFVectors(dst, cornersOld, cornersCurr, status);

  if (masks.size() == 4)
    {
      // First quadrant NE (North East = upper right)
      // compute average velocity magnitude (x-axis suffices as +x: FOE, -x: FOC/going backward)
      for (cIdx = 0; cIdx < cornersNE; cIdx++)
        {
          if (status[cIdx] > 0) // if match is valid
            {
              counterValid++;
              float xDiff = cornersCurr[cIdx].x - cornersOld[cIdx].x; // subtract head (new xpos) - tail (old xpos)
              if (xDiff > 0.0)
                xValues += 1.0; // add positive weight
              else
                xValues -= 1.0; // subtract positive weight
              yValues += cornersCurr[cIdx].y - cornersOld[cIdx].y; // subtract head (new ypos) - tail (old ypos)
            }
        }
      if (counterValid > 0)
        {
          float xMotionAvgNE = xValues / (float) counterValid;
          float yMotionAvgNE = yValues / (float) counterValid;

          std::cout << "xMotionAvgNE" << xMotionAvgNE << std::endl;

          if (xMotionAvgNE > 0.0)
            {
              concensusNE = FORWARD;
#ifdef DEBUG
              std::cout << "NE: " << "moving FORWARD" << std::endl;
#endif
            }
          else

            {
              concensusNE = BACKWARD;
#ifdef DEBUG
              std::cout << "NE: " << "moving BACKWARD" << std::endl;
#endif
            }
        }

      // Second quadrant NW (North West = upper left)
      // compute average velocity magnitude (x-axis suffices as +x: FOC, -x: FOE/going forward)
      counterValid = 0;
      xValues = 0.0;
      yValues = 0.0;
      for (; cIdx < cornersNW; cIdx++)
        {
          if (status[cIdx] > 0) // if match is valid
            {
              counterValid++;
              float xDiff = cornersCurr[cIdx].x - cornersOld[cIdx].x; // subtract head (new xpos) - tail (old xpos)
              if (xDiff > 0.0)
                xValues += 1.0; // add positive weight
              else
                xValues -= 1.0; // subtract positive weight
              yValues += cornersCurr[cIdx].y - cornersOld[cIdx].y; // subtract head (new ypos) - tail (old ypos)
            }
        }
      if (counterValid > 0)
        {
          float xMotionAvgNW = xValues / (float) counterValid;
          float yMotionAvgNW = yValues / (float) counterValid;

          std::cout << "xMotionAvgNW" << xMotionAvgNW << std::endl;

          if (xMotionAvgNW < 0.0)
            {
              concensusNW = FORWARD;
#ifdef DEBUG
              std::cout << "NW: " << "moving FORWARD" << std::endl;
#endif
            }
          else
            {
              concensusNW = BACKWARD;
#ifdef DEBUG
              std::cout << "NW: " << "moving BACKWARD" << std::endl;
#endif
            }
        }

      // Third quadrant SW (South West = lower left)
      // compute average velocity magnitude (x-axis suffices as +x: FOC, -x: FOE/going forward)
      counterValid = 0;
      xValues = 0.0;
      yValues = 0.0;
      for (; cIdx < cornersSW; cIdx++)
        {
          if (status[cIdx] > 0) // if match is valid
            {
              counterValid++;
              float xDiff = cornersCurr[cIdx].x - cornersOld[cIdx].x; // subtract head (new xpos) - tail (old xpos)
              if (xDiff > 0.0)
                xValues += 1.0; // add positive weight
              else
                xValues -= 1.0; // subtract positive weight
              yValues += cornersCurr[cIdx].y - cornersOld[cIdx].y; // subtract head (new ypos) - tail (old ypos)
            }
        }

      if (counterValid > 0)
        {
          float xMotionAvgSW = xValues / (float) counterValid;
          float yMotionAvgSW = yValues / (float) counterValid;

          std::cout << "xMotionAvgSW" << xMotionAvgSW << std::endl;

          //      if (xMotionAvgSW < 0 || yMotionAvgSW > 0)
          if (xMotionAvgSW < 0.0)
            {
              concensusSW = FORWARD;
#ifdef DEBUG
              std::cout << "SW: " << "moving FORWARD" << std::endl;
#endif
            }
          else
            {
              concensusSW = BACKWARD;
#ifdef DEBUG
              std::cout << "SW: " << "moving BACKWARD" << std::endl;
#endif
            }
        }

      // Fourth quadrant SE (South East = lower right)
      // compute average velocity magnitude (x-axis suffices as +x: FOE, -x: FOC/going backward)
      counterValid = 0;
      xValues = 0.0;
      yValues = 0.0;
      for (; cIdx < cornersSE; cIdx++)
        {
          if (status[cIdx] > 0) // if match is valid
            {
              counterValid++;
              float xDiff = cornersCurr[cIdx].x - cornersOld[cIdx].x; // subtract head (new xpos) - tail (old xpos)
              if (xDiff > 0.0)
                xValues += 1.0; // add positive weight
              else
                xValues -= 1.0; // subtract positive weight
              yValues += cornersCurr[cIdx].y - cornersOld[cIdx].y; // subtract head (new ypos) - tail (old ypos)
            }
        }

      if (counterValid > 0)
        {
          float xMotionAvgSE = xValues / (float) counterValid;
          float yMotionAvgSE = yValues / (float) counterValid;

          std::cout << "xMotionAvgSE" << xMotionAvgSE << std::endl;

          //      if (xMotionAvgSE > 0 || yMotionAvgSE > 0)
          if (xMotionAvgSE > 0.0)
            {
              concensusSE = FORWARD;
#ifdef DEBUG
              std::cout << "SE: " << "moving FORWARD" << std::endl;
#endif
            }
          else
            {
              concensusSE = BACKWARD;
#ifdef DEBUG
              std::cout << "SE: " << "moving BACKWARD" << std::endl;
#endif
            }
        }

      int globalConcensus = concensusNE + concensusNW + concensusSW
          + concensusSE;
#ifdef DEBUG
      std::cout << "Global Concensus value: " << globalConcensus << std::endl;
#endif
      if (globalConcensus > 0)
        {
          std::cout << "GLOBAL: " << "moving FORWARD" << std::endl;
          globalConcensus = FORWARD;
        }
      if (globalConcensus < 0)
        {
          std::cout << "GLOBAL:" << "moving BACKWARD" << std::endl;
          globalConcensus = BACKWARD;
        }
      return globalConcensus;
    }
  else
    {
      return 0;
    }

}

//void drawDirectionArrow(cv::Mat &img, cv::Point p_base, int arrow_length, float direction, cv::Scalar color, int thickness)
void
drawDirectionArrow(cv::Mat &img, cv::Point2f &p_base, cv::Point2f &p_tip,
    cv::Scalar color, int thickness, int lineType)
{
  cv::Point2f p_diffs = p_tip - p_base;

  int arrow_length = sqrt(pow(p_diffs.x, 2) + pow(p_diffs.y, 2));

  float dir_angle = atan2(p_diffs.y, p_diffs.x);
  cv::Point2f p_head_0, p_head_1;

  int tip_length = (int) (arrow_length * 0.3);

  float head_theta_0, head_theta_1;
  head_theta_0 = dir_angle - 0.4;
  head_theta_1 = dir_angle + 0.4;

  p_head_0 = cv::Point2f(p_tip.x - tip_length * cos(head_theta_0), p_tip.y
      - tip_length * sin(head_theta_0));
  p_head_1 = cv::Point2f(p_tip.x - tip_length * cos(head_theta_1), p_tip.y
      - tip_length * sin(head_theta_1));

  cv::line(img, p_base, p_tip, color, thickness, lineType, 0);
  cv::line(img, p_tip, p_head_0, color, thickness, lineType, 0);
  cv::line(img, p_tip, p_head_1, color, thickness, lineType, 0);

  cv::circle(img, p_base, 3, color, 2, 8, 0);
}

void
markCorners(cv::Mat &imageMat, std::vector<cv::Point2f> pointsOnImage)
{
  int nThickness = 1;

  std::vector<cv::Point2f>::const_iterator ci;

  //   if (pointsOnImage.empty() == false)
  for (ci = pointsOnImage.begin(); ci != pointsOnImage.end(); ci++)
    {
      //Each pressed point is painted as a small circle on the window
      cv::circle(imageMat, cv::Point(ci->x, ci->y), 3, CV_RGB(0, 255, 0),
          nThickness, 8, 0);
    }
}

void
drawOFVectors(cv::Mat &imageMat, std::vector<cv::Point2f> &pointsBegin,
    std::vector<cv::Point2f> &pointsEnd, std::vector<uchar> &status)
{
  int nThickness = 1;
  std::vector<cv::Point2f>::const_iterator ciBegin;
  std::vector<cv::Point2f>::const_iterator ciEnd;
  std::vector<uchar>::const_iterator ciStatus;
  //  cv::namedWindow("FrameTest", CV_WINDOW_AUTOSIZE );

  if (pointsBegin.size() == pointsEnd.size())
    {
      ciBegin = pointsBegin.begin();
      ciEnd = pointsEnd.begin();
      int count = 0;

      for (; ciBegin != pointsBegin.end(); ciBegin++, ciEnd++, count++)
        {
          cv::Point2f pointA(ciBegin->x, ciBegin->y);
          cv::Point2f pointB(ciEnd->x, ciEnd->y);
#ifdef DEBUG
          //             std::cout << "Status: " << count << std::endl;
          //          printf("Status %u\n", status[count]);
          //          std::cout << "(" << pointA.x << ", " << pointA.y << ")" << std::endl;
          //          std::cout << "(" << pointB.x << ", " << pointB.y << ")" << std::endl
          //          << std::endl;
#endif
          if (status[count] > 0) // Only draw valid flow vectors
            drawDirectionArrow(imageMat, pointA, pointB, CV_RGB(0, 255, 0),
                nThickness, 8);
          //             cv::imshow("FrameTest", frameMat);
          //             cv::waitKey(0);
        }
    }
}

void
addCornersToVector(std::vector<cv::Point2f> &target, const std::vector<
    cv::Point2f> &additional)
{
  for (uint i = 0; i < additional.size(); i++)
    target.push_back(additional[i]);
}

#endif /* COMMON_H_ */
