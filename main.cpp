#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
using namespace cv;
using namespace std;


Mat src; Mat src_gray;
int thresh = 100;
int max_thresh = 255;
RNG rng(12345);
Point h_g,b_d;
Mat dst;

// Image
void thresh_callback(int, void*);



void thresh_callback(int, void*)
{

  Mat threshold_output;
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;

  /// Detect edges using Threshold
  threshold( src_gray, threshold_output, thresh, 255, THRESH_BINARY );
  /// Find contours
  findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


  /// Find the rotated rectangles and ellipses for each contour
  vector<RotatedRect> minRect( contours.size() );
  vector<RotatedRect> minEllipse( contours.size() );


  for( int i = 0; i < contours.size(); i++ )
     { minRect[i] = minAreaRect( Mat(contours[i]) );
       if( contours[i].size() > 5 )
         { minEllipse[i] = fitEllipse( Mat(contours[i]) ); }
     }

  /// Draw contours + rotated rects + ellipses
  Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
  for( int i = 0; i< contours.size(); i++ )
     {
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       // contour
       drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
       // ellipse
        ellipse( drawing, minEllipse[i], color, 2, 8 );
       // rotated rectangle
       Point2f rect_points[4]; minRect[i].points( rect_points );
       /*
       for( int j = 0; j < 4; j++ )
          line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
          */
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );
}


int main(int argc, char *argv[])
{

    Mat img = imread("images/cans/sevenup.jpg", CV_LOAD_IMAGE_COLOR);

    if(img.empty())
       return -1;



 Mat cdst;
 Canny(img, dst, 50, 200, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);


    vector<Vec4i> lines;
    HoughLinesP(dst, lines, 1, CV_PI/180, 50, 150, 30 );
    cout << lines.size() << endl;

    Vec4i leftSide,rightSide;
    if (!lines.empty()) {
        leftSide=lines[0];
        rightSide=leftSide;
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



    int dist = rightSide[0]-leftSide[0];
    int _hauteur = max(rightSide[3]-rightSide[1],leftSide[3]-leftSide[1]);
    int _marge_basse = _hauteur/4;
    Point h_g(leftSide[0],max(leftSide[1]-dist,0));
    Point b_d(rightSide[0],rightSide[3]+_marge_basse);
    cout << h_g << endl;
    cout << b_d << endl;
    Mat threshold_output;
      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;

      /// Detect edges using Threshold
      threshold( dst, threshold_output, thresh, 255, THRESH_BINARY );
      /// Find contours
      findContours( Mat(threshold_output, Rect(h_g,b_d)), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );


      /// Find the rotated rectangles and ellipses for each contour
      vector<RotatedRect> minRect( contours.size() );
      //vector<RotatedRect> minEllipse( contours.size() );
      RotatedRect minEllipse;
    for (int i=0; i<contours.size() ;i++) {
        if (contours[i].size() > 5) {
            cout << "coucou" << endl;
            minEllipse = fitEllipse(Mat(contours[i]));
            break;
        }
    }

    /*
    Mat matrice_Reduite(dst,Rect(h_g,b_d));
    vector<Point2f> points;

    matrice_Reduite.copyTo(points);
    RotatedRect mon_ellipse = fitEllipse(Mat(points));
    */
    ellipse( img,minEllipse, Scalar(0,255,0), 3, CV_AA );
    rectangle(img,Rect(h_g,b_d),Scalar(255,100,0),3,CV_AA);
    //namedWindow("Detected lines",CV_WINDOW_AUTOSIZE);
    //imshow("Detected lines",cdst);

    imshow("source", img);
    imshow("detected lines", cdst);





  /// Load source image and convert it to gray
  src = imread("images/cans/sevenup.jpg", 1 );

  /// Convert image to gray and blur it
  cvtColor( src, src_gray, CV_BGR2GRAY );
  blur( src_gray, src_gray, Size(3,3) );

  /// Create Window
  char* source_window = "Source";
  namedWindow( source_window, CV_WINDOW_AUTOSIZE );
  imshow( source_window, src );

  createTrackbar( " Threshold:", "Source", &thresh, max_thresh, thresh_callback);
  thresh_callback( 0, 0);

  waitKey(0);

    return 0;
}



