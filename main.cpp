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
double INFINI=1000.0;

// Image
void thresh_callback(int, void*);

double length(Vec4i line) {
    return sqrt(pow(line[3]-line[1],2)+pow(line[2]-line[0],2));
}

/// Renvoie les points haut gauche et bas droite qui définissent le rectangle qui contient la canette
/// Le rectangle n'est pas orienté, il est vertical.
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

///Associe des points au contour de la canette.
vector<Point2i> fitCanShape(Mat &can, int nbPoints) {
    vector<Point2i> points;
    //Bas gauche -> haut gauche
    for (int i=nbPoints-1; i>=0; i--) {
        uchar *row = can.ptr<uchar>(i*can.rows/nbPoints);
        int col=0;
        int val = (int) row[col];
        while ((val==0) && (col < can.cols)) {
            col++;
            val = (int) row[col];
        }
        if (col!=can.cols) {
            points.push_back(Point2i(col,i*can.rows/nbPoints));
        }
        else {
            points.push_back(Point2i(can.cols,i*can.rows/nbPoints));
        }
    }

    //Haut droite -> bas droite
    for (int i=0; i<nbPoints; i++) {
        uchar *row = can.ptr<uchar>(i*can.rows/nbPoints);
        int col=can.cols;
        int val = (int) row[col];
        while ((val==0) && (col >= 0)) {
            col--;
            val = (int) row[col];
        }
        if (col!=0) {
            points.push_back(Point2i(col,i*can.rows/nbPoints));
        }
        else {
            points.push_back(Point2i(can.cols,i*can.rows/nbPoints));
        }
    }

    return points;
}

///Donne les paramètres de la droite qui correspond le mieux aux points donnés
///(c'est surtout la pente suivant x qui est significative)
Vec4f slope(vector<Point2i> shape, int position, int nbPoints) {
    vector<Point2i> pointsForSlope;
    Vec4f segment;
    for (int i=position-nbPoints/2; i <= position+nbPoints/2; i++) {
        pointsForSlope.push_back(shape[i]);
    }
    fitLine(pointsForSlope,segment,CV_DIST_L12,0,0.01,0.01);
    return segment;
}

///Retourne l'index dans le vecteur shape du point correspondant au
///coin supérieur haut de la canette.
int topLeftOfCan(vector<Point2i> &shape, int resolution) {
    int position=resolution/2;

    ///Calcul du coeff directeur du segment gauche de la canette
    Vec4f segment=slope(shape,position, 6);
    //Vec4i currentSegment;
    while (abs(segment[0]) < 0.2) {
        position++;
        segment=slope(shape, position, 6);
        cout << "segment : " << "vx=" << segment[0] << " vy=" << segment[1] << endl;
    }

    while (abs(segment[0]) > 0.2) {
        position++;
        segment=slope(shape,position, 3);
        cout << "segment : " << "vx=" << segment[0] << "vy=" << segment[1] << endl;
    }
    return ++position;
}

///Recherche l'ellipse du haut de la canette
vector<Point2i> topEllipseOfCan(vector<Point2i> &shape, int resolution) {
    int position = topLeftOfCan(shape, resolution);

    int topY=shape[++position].y;
    cout << "topY : " << topY << endl;
    //Index du haut de la canette trouvé
    vector<Point2i> halfEllipse;
    for (int currentY=topY; ((currentY <= topY) && (position < shape.size())); position++) {
        currentY=shape[position].y;
        halfEllipse.push_back(shape[position]);
    }
    cout << "Nombre de points de l'ellipse : " << halfEllipse.size() << endl;
    if (position < shape.size()) {
        return halfEllipse;
    }
    return vector<Point2i>();
}

//Haut gauche -> haut droite
//    for (int i=0; i<nbPoints; i++) {
//        Mat col=can.col(i*can.cols/nbPoints);
//        int row=0;
//        int val=(int) col.at<uchar>(row);
//        while ((val==0) && (row < can.rows)) {
//            row++;
//            val = (int) col.at<uchar>(row);
//        }
//        if (row != can.rows) {
//            points.push_back(Point2i(i*can.cols/nbPoints,row));
//        }
//        else {
//            points.push_back(Point2i(i*can.cols/nbPoints,0));
//        }
//    }

int main(int argc, char *argv[])
{
    cout.precision(4);

    Mat img = imread("images/cans/sevenup.jpg", CV_LOAD_IMAGE_COLOR);

    if(img.empty())
       return -1;

    /// Image couleur pour pouvoir tracer des formes en couleur
    /// sur l'image des contours qui est en noir et blanc
    Mat cdst;
    /// Tracé des contours
    Canny(img, dst, 50, 200, 3);
    /// Floutage, utilisé pour rendre les bords plus continus
    GaussianBlur(dst,dst,Size(3,3),0);

    threshold( dst, dst, thresh, 255, THRESH_BINARY );
    cvtColor(dst, cdst, CV_GRAY2BGR);

    /// On trace le rectangle qui contient la canette
    vector<Point2i> limits;
    limits = findCanLimits(dst);
    /// Le rectangle est défini par 2 points contenus dans limits.
    Rect limitsRect(limits[0],limits[1]);
    rectangle(cdst,limitsRect,Scalar(255,0,0));

    /// Can : image contenant le rectangle qui englobe la canette
    Mat can(dst, limitsRect);
    Mat colorCan;
    cvtColor(can,colorCan,CV_GRAY2BGR);

    /// On cherche la forme qui s'adaptera le mieux au contour extérieur de la canette
    vector<Point2i> shape;

    /// Nombre de points par coté à fitter sur le contour
    int resolution = 60;
    shape = fitCanShape(can,resolution);

    /// On trouve le coté gauche du "couvercle de la canette"
    int topLeft = topLeftOfCan(shape, resolution);
    cout << topLeft << endl;
    circle(colorCan,Point(shape[topLeft].x,shape[topLeft].y), 5, Scalar(200,100,50));

    /// On récupère la demi-ellipse du haut
    vector<Point2i> halfEllipse = topEllipseOfCan(shape, resolution);

    vector<Point2i> topEllipse = halfEllipse;
    int centerY = halfEllipse[halfEllipse.size()-1].y;
    for (int sizeHalf = halfEllipse.size(), i=sizeHalf-1 ; i>=0; i--) {
        int dist = centerY-halfEllipse[i].y;
        topEllipse.push_back(Point2i(halfEllipse[i].x, centerY+dist));
    }
    for (int i=0;i<topEllipse.size()-1;i++) {
        line(colorCan,topEllipse[i],topEllipse[i+1],Scalar(150,30,59), 3, CV_AA);
    }


    RotatedRect fittedEllipse = fitEllipse(topEllipse);
    Rect boundingRect = fittedEllipse.boundingRect();

    rectangle(img,
                Rect(
                    Point(boundingRect.x+limitsRect.x,boundingRect.y+limitsRect.y),
                    Point(boundingRect.x+boundingRect.width+limitsRect.x,boundingRect.y+boundingRect.height+limitsRect.y)),
                Scalar(0,255,200));
    namedWindow("topEllipse",CV_WINDOW_AUTOSIZE);
    imshow("topEllipse",img);

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
      //imshow("calcHist Demo", histImage );

    waitKey(0);

    return 0;
}



