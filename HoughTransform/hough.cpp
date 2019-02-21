// Hough Lines
//
// YongHye Kwon

// Kwangwoon University (2014 ~ ) , Major: Robotics
// Feb 13, 2017
// modified date: Feb 16, 2017

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const double PI = 3.14159265;
const double DEG2RAD = PI / 180.;

typedef struct HoughLineInfo {
 vector<unsigned int> iAngle;
 vector<double> dRho;
}HoughLineInfo;

int main()
{
 Mat matSrc = imread("./testImage/cameraman1.png", CV_LOAD_IMAGE_GRAYSCALE);
 if (matSrc.empty() == true)return -1;

 Mat matDraw;
 cvtColor(matSrc, matDraw, CV_GRAY2BGR);

 // 1.  Edge detection, e.g. using the Canny edge detector.

 Mat matEdge;
 GaussianBlur(matSrc, matSrc, Size(9, 9), 1.0);
 Canny(matSrc, matEdge, 100, 150);




 //2. Mapping of edge points to the Hough space and storage in an accumulator

 const unsigned int kAngleSize = 180;
 const unsigned int kDistMax = sqrt((matEdge.cols*matEdge.cols) + (matEdge.rows*matEdge.rows)) + 1;
 const unsigned int kDistanceSize = kDistMax * 2;
 const unsigned int kThresHoldForLine = 100;

 unsigned int* pNvote = new unsigned int[kAngleSize*kDistanceSize];
 memset(pNvote, 0, (kAngleSize*kDistanceSize) * sizeof(unsigned int));

 double lukCos[180];
 double lukSin[180];
 unsigned int lukAngleIdx[180];

 for (int i = 0; i < kAngleSize; i++)
 {
  double angle = (double)i*DEG2RAD;

  lukCos[i] = cos(angle);
  lukSin[i] = sin(angle);
  lukAngleIdx[i] = i*kDistanceSize;
 }

 unsigned char* ucMatEdgeData = matEdge.data;
 for (int y = 0; y < matEdge.rows; y++)
 {
  for (int x = 0; x < matEdge.cols; x++)
  {
   if (*ucMatEdgeData++ == 0)continue;

   for (unsigned int j = 0; j < kAngleSize; j++)
   {
    double rho = lukCos[j] * x + lukSin[j] * y;
    rho += (double)kDistMax;

    pNvote[lukAngleIdx[j] + (int)(rho + 0.5)]++;
   }
  }
 }

 //3. Interpretation of the accumulator to yield lines of infinite length. 
 //The interpretation is done by thresholding and possibly other constraints.

 Point ptA;
 Point ptB;

 HoughLineInfo lineVec;

 vector<Vec2f> linesVec;

 for(int i= 0;i < 180; i++)
  for (int j = 0; j < kDistanceSize; j++)
  {
   int nVote = pNvote[lukAngleIdx[i] + j];
   if (nVote >= kThresHoldForLine)
   {
    bool isTrueLine = true;

    for (int dAngle = -1; dAngle <= 1 && isTrueLine; dAngle++)
    {
     if (i + dAngle < 0)continue;
     if (i + dAngle >= kAngleSize)break;

     for (int dRho = -1; dRho <= 1 && isTrueLine; dRho++)
     {
      if (j + dRho < 0)continue;
      if (j + dRho >= kDistanceSize)break;

      if (pNvote[lukAngleIdx[i + dAngle] + (j + dRho)] > nVote)isTrueLine = false;
     }
    }

    if (isTrueLine == false) continue;

    lineVec.iAngle.push_back(i);
    lineVec.dRho.push_back(j - (int)kDistMax);
   }
  }

 int nLineVecSize = lineVec.dRho.size();
 for (int i = 0; i < nLineVecSize; i++)
 {
  int angle = lineVec.iAngle[i];

  double cX = lukCos[angle];
  double cY = lukSin[angle];

  double rho = lineVec.dRho[i];

  double x0 = cX*rho;
  double y0 = cY*rho;

  ptA.x = cvRound(x0 + 1000. * (-cY));
  ptA.y = cvRound(y0 + 1000. * (cX));
  ptB.x = cvRound(x0 - 1000. * (-cY));
  ptB.y = cvRound(y0 - 1000. * (cX));

  line(matDraw, ptA, ptB, Scalar(0, 255, 0), 1);
 }

 delete[] pNvote;


 imshow("src", matSrc); 
 imshow("edge", matEdge); 
 imshow("drawLine", matDraw); 

 waitKey(0);

 return 0;
}
[출처] (3차 수정) opencv 내부 함수 houghline 보다 더 빠른 houghline 코드입니다. (OpenCV KOREA 대한민국 최고의 컴퓨터비젼 커뮤니티) |작성자 로봇미엔
