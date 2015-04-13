#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cmath>
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

double length(Vec4i line) {
    return sqrt(pow(line[3]-line[1],2)+pow(line[2]-line[0],2));
}

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
    {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 5 )
        {
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
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

vector<Point2i> findCanLimits(Mat &contoursImg) {

    /// Délimitations de la canette
    int top=0;
    int bottom = contoursImg.rows-1;
    int left=0;
    int right=contoursImg.cols-1;
    while (countNonZero(contoursImg.row(top)) < 10) {
        top++;
    }
    while (countNonZero(contoursImg.row(bottom)) < 10) {
        bottom--;
    }
    while (countNonZero(contoursImg.col(left)) < 10) {
        left++;
    }
    while (countNonZero(contoursImg.col(right)) < 10) {
        right--;
    }

    /// Points qui définissent le rectangle
    Point2i h_g(left,top);
    Point2i b_d(right,bottom);

    cout << h_g << endl;
    cout << b_d << endl;

    vector<Point2i> limits;
    limits.push_back(h_g);
    limits.push_back(b_d);
    return limits;
}

int main(int argc, char *argv[])
{

    Mat img = imread("images/cans/perrier.jpg", CV_LOAD_IMAGE_COLOR);
    if(img.empty())
       return -1;

    Mat cdst;
    Canny(img, dst, 50, 200, 3);
    GaussianBlur(dst,dst,Size(3,3),0);
    cvtColor(dst, cdst, CV_GRAY2BGR);


    vector<Point2i> limits;
    cout << "lol" << endl;
    limits = findCanLimits(dst);
    Rect limitsRect(limits[0],limits[1]);
    rectangle(cdst,limitsRect,Scalar(255,0,0));

    /// Méthode utilisant la transformée de Hough
    /// Pas très efficace pour trouver les lignes verticales des bords
//    vector<Vec4i> lines;
//    HoughLinesP(dst, lines, 1, CV_PI/180, 30, img.rows/3, 20 );
//    cout << "Nombre de lignes détectées : " << lines.size() << endl;
//
//    Vec4i leftSide,rightSide;
//    if (!lines.empty()) {
//        leftSide=lines[0];
//        rightSide=leftSide;
//        for( size_t i = 0; i < lines.size(); i++ )
//        {
//            Vec4i l = lines[i];
//            /// On vérifie que la ligne est bien verticale
//            if (abs(l[0] - l[2]) < 10) {
//                /// La limite gauche ou droite est celle qui est la plus à gauche (droite)
//                /// ET la plus grande possible
//                if ((l[0]<=leftSide[0]) && (l[2]<=leftSide[2])) {
//                    /// Pour "orienter" la ligne de haut en bas
//                    if (l[1]<l[3]) {
//                        leftSide=l;
//                    }
//                    else {
//                        leftSide[0]=l[2];
//                        leftSide[1]=l[3];
//                        leftSide[2]=l[0];
//                        leftSide[3]=l[1];
//                    }
//                }
//                if ((l[0]>=rightSide[0]) && (l[2]>=rightSide[2])) {
//                    /// Pour "orienter" la ligne de haut en bas
//                    if (l[1]<l[3]) {
//                        rightSide=l;
//                    }
//                    else {
//                        rightSide[0]=l[2];
//                        rightSide[1]=l[3];
//                        rightSide[2]=l[0];
//                        rightSide[3]=l[1];
//                    }
//                }
//                /// Toutes les lignes sont dessinées en rouge
//                line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
//            }
//        }
//        /// Les deux lignes sélectionnées sont dessinées en vert
//        line( cdst, Point(leftSide[0], leftSide[1]), Point(leftSide[2], leftSide[3]), Scalar(0,255,0), 3, CV_AA);
//        line( cdst, Point(rightSide[0], rightSide[1]), Point(rightSide[2], rightSide[3]), Scalar(0,255,0), 3, CV_AA);
//
//
//
//
//        int dist = abs(rightSide[0]-leftSide[0]);
//        int _hauteur = max(rightSide[3]-rightSide[1],leftSide[3]-leftSide[1]);
//        int _marge_basse = _hauteur/5;
//        Point h_g(leftSide[0],max(leftSide[1]-dist,0));
//        Point b_d(max(rightSide[0],rightSide[2]),min(rightSide[1]+_marge_basse,img.rows/3));
//        cout << h_g << endl;
//        cout << b_d << endl;
        Mat threshold_output;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;

        /// Avec image des contours Canny

        /// Detect edges using Threshold
        threshold( dst, threshold_output, thresh, 255, THRESH_BINARY );
        /// Find contours
        Rect searchEllipseArea(limitsRect.x,limitsRect.y,limitsRect.width,limitsRect.height/3);
        Mat area(img, searchEllipseArea);
        findContours( Mat(threshold_output, searchEllipseArea), contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
        Mat drawing = Mat::zeros( dst.size(), CV_8UC3 );
        for( int i = 0; i< contours.size(); i++ )
        {
            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 1, 8, hierarchy, 0, Point() );
        }
        namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
        imshow( "Contours", drawing );
        /// Find the ellipses for each contour
        //vector<RotatedRect> minEllipse( contours.size() );
        RotatedRect minEllipse;
        for (int i=0; i<contours.size() ;i++) {
            if (contours[i].size() > 250) {
                cout << "coucou" << endl;
                minEllipse = fitEllipse(Mat(contours[i]));
                ellipse( area,minEllipse, Scalar(0,255,0), 3, CV_AA );
            }
        }

        //ellipse( area,minEllipse, Scalar(0,255,0), 3, CV_AA );
        rectangle(img,Rect(h_g,b_d),Scalar(255,100,0),3,CV_AA);


    namedWindow("Detected rectangle",CV_WINDOW_AUTOSIZE);
    imshow("Detected rectangle",cdst);

    imshow("source", img);

    ///Calcul d'histogramme
    /// Separate the image in 3 places ( B, G and R )
      vector<Mat> bgr_planes;
      split( Mat(img, limitsRect), bgr_planes );

      /// Establish the number of bins
      int histSize = 256;

      /// Set the ranges ( for B,G,R) )
      float range[] = { 0, 256 } ;
      const float* histRange = { range };

      bool uniform = true; bool accumulate = false;

      Mat b_hist, g_hist, r_hist;

      /// Compute the histograms:
      calcHist( &bgr_planes[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
      calcHist( &bgr_planes[1], 1, 0, Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
      calcHist( &bgr_planes[2], 1, 0, Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

      // Draw the histograms for B, G and R
      int hist_w = 512; int hist_h = 400;
      int bin_w = cvRound( (double) hist_w/histSize );

      Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );

      /// Normalize the result to [ 0, histImage.rows ]
      normalize(b_hist, b_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
      normalize(g_hist, g_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );
      normalize(r_hist, r_hist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

      /// Draw for each channel
      for( int i = 1; i < histSize; i++ )
      {
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                           Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                           Scalar( 255, 0, 0), 2, 8, 0  );
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                           Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                           Scalar( 0, 255, 0), 2, 8, 0  );
          line( histImage, Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                           Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                           Scalar( 0, 0, 255), 2, 8, 0  );
      }

      /// Display
      namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
      imshow("calcHist Demo", histImage );

    waitKey(0);

    return 0;
}



