#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    Mat img = imread("images/cans/sevenup.jpg", CV_LOAD_IMAGE_COLOR);
    if(img.empty())
       return -1;



    /*
    Mat gray;

    int scale = 1;
    int delta = 0;
    int ddepth = CV_16S;

    cvtColor(img,gray,CV_BGR2GRAY);
    Mat grad;

    /// Contours horizontaux et verticaux avec filtres créés à la main
    /*Mat contoursHor,contoursVer;

    Mat filtreHor = (Mat_<double>(3,3) << 1,0,-1,2,0,-2,1,0,-1);
    Mat filtreVer = (Mat_<double>(3,3) << 1,2,1,0,0,0,-1,-2,-1);
    filter2D(gray,contoursHor,gray.depth(),filtreHor, Point(-1,-1));
    filter2D(gray,contoursVer,gray.depth(),filtreVer,Point(-1,-1));
    pow(contoursHor,2,contoursHor);
    pow(contoursVer,2,contoursVer);

    namedWindow("Contours horizontaux");
    imshow("Contours horizontaux",contoursHor);
    namedWindow("Contours verticaux");
    imshow("Contours verticaux",contoursVer);

    ///Utilisation du filtre de Sobel fait par OpenCV
    Mat grad_x, grad_y;
      Mat abs_grad_x, abs_grad_y;

      /// Gradient X
      //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
      Sobel( gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
      convertScaleAbs( grad_x, abs_grad_x );

      /// Gradient Y
      //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
      Sobel( gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
      convertScaleAbs( grad_y, abs_grad_y );

      /// Total Gradient (approximate)
      addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );


    Mat cdst;
    cvtColor(grad, cdst, CV_GRAY2BGR);
    namedWindow("Sobel");
    imshow("Sobel",cdst);
    /// Détection de lignes

  vector<Vec2f> lines;
  HoughLines(grad, lines, 1, CV_PI/180, 100, 0, 0 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }

  vector<Vec4i> lines;
  HoughLinesP(grad, lines, 1, CV_PI/180, 50, 450, 10 );
  cout << CV_PI << endl;
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
*/

 Mat dst, cdst;
 Canny(img, dst, 50, 200, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);

 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
 #else
    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 150, 30 );
    cout << lines.size() << endl;
    if (!lines.empty()) {
        Vec4i leftSide=lines[0];
        Vec4i rightSide=leftSide;
        for( size_t i = 0; i < lines.size(); i++ )
        {
            Vec4i l = lines[i];
            if (abs(l[0] - l[2]) < 30) {
                if ((l[0]<=leftSide[0]) && (l[2]<=leftSide[2]))  {
                    leftSide=l;
                }
                if ((l[0]>=rightSide[0]) && (l[2]>=rightSide[2])) {
                    rightSide=l;
                }
                line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
            }
        }
        line( cdst, Point(leftSide[0], leftSide[1]), Point(leftSide[2], leftSide[3]), Scalar(0,255,0), 3, CV_AA);
        line( cdst, Point(rightSide[0], rightSide[1]), Point(rightSide[2], rightSide[3]), Scalar(0,255,0), 3, CV_AA);
    }

 #endif

    //namedWindow("Detected lines",CV_WINDOW_AUTOSIZE);
    //imshow("Detected lines",cdst);

    imshow("source", img);
    imshow("detected lines", cdst);

    waitKey(0);
    return 0;
}



