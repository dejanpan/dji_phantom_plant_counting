#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>

#include <boost/filesystem.hpp>

#define in2cm 2.54

using namespace cv;
using namespace std;
using namespace boost::filesystem;

enum DeviceMode { JAI, NIKON, SCANNER };

struct Scanner
{
  static double scaling()
  {
    // ratio width should be the same as ratio heigth
    return 1. / (ratio_width() * ratio_height());
  }

  static double ratio_width()
  {
    return width_dpi / in2cm;
  }

  static double ratio_height()
  {
    return height_dpi / in2cm;
  }

private:
  static const double width_dpi = 600.;
  static const double height_dpi = 600.;
};

struct Jai
{
  //for calculation of height_mm and width_mm use:
  //http://www.vision-doctor.co.uk/optical-calculations/calculation-object-size.html
  //used lens: http://www.fujifilmusa.com/products/optical_devices/machine-vision/1-3-high/tf8da-8b/
  //see http://www.stemmer-imaging.de/de/produkte/jai-ad-130-ge/

  static double scaling()
  {
    //ratio width_px/width_mm should be the same as heigth_px/height_mm
    return 1. / (ratio_width() * ratio_height()) * 0.01;
  }

  static double ratio_width()
  {
    return width_px / width_mm;
  }

  static double ratio_height()
  {
    return height_px / height_mm;
  }

private:
  static const double width_px = 1296;
  static const double height_px = 966;
  static const double width_mm = 462.00;
  static const double height_mm = 334.89;
};

struct Nikon
{
  //for calculation of height_mm and width_mm use:
  //http://www.vision-doctor.co.uk/optical-calculations/calculation-object-size.html
  //used lens: http://www.fujifilmusa.com/products/optical_devices/machine-vision/1-3-high/tf8da-8b/
  //see http://www.stemmer-imaging.de/de/produkte/jai-ad-130-ge/

  static double scaling()
  {
    //ratio width_px/width_mm should be the same as heigth_px/height_mm
    return 1. / (ratio_width() * ratio_height()) * 0.01;
  }

  static double ratio_width()
  {
    return width_px / width_mm;
  }

  static double ratio_height()
  {
    return height_px / height_mm;
  }

private:
  // day30
  //static const double width_px = 3295;
  //static const double height_px = 2270;
  // day26
  //static const double width_px = 3219;
  //static const double height_px = 2256;
  // day23
  static const double width_px = 3203;
  static const double height_px = 2252;
  static const double width_mm = 220.00;
  static const double height_mm = 160.00;
};

struct AreaConverter
{
  double pixel_scale_;
  double length_scale_;
  double width_scale_;

  AreaConverter(DeviceMode dev)
  {
    switch(dev)
    {
      case JAI:
      {
        pixel_scale_ = Jai::scaling();
        length_scale_ = Jai::ratio_height();
        width_scale_ = Jai::ratio_width();
        break;
      }
      case NIKON:
      {
        pixel_scale_ = Nikon::scaling();
        length_scale_ = Nikon::ratio_height();
        width_scale_ = Nikon::ratio_width();
        break;
      }
      case SCANNER: {
        pixel_scale_ = Scanner::scaling();
        length_scale_ = Scanner::ratio_height();
        width_scale_ = Scanner::ratio_width();
        break;
      }
    }
  }

  inline double px2cm2(double pixels) { return pixels * pixel_scale_; }
  inline double px2length(double pixels) { return pixels / length_scale_; }
  inline double px2width(double pixels) { return pixels / width_scale_; }
};


struct by_y
{
  inline bool operator()(Point2f const &a, Point2f const &b) { return a.y < b.y; }
};

struct by_x
{
  inline bool operator()(Point2f const &a, Point2f const &b) { return a.x < b.x; }
};

struct contours_and_centers
{
  vector<Point>  contour;
  int side;
  Point2f center;
  double length;
  double width;
};

/// this sorts first by the side (0-left, 1-right) and then by the center.y
/// aka column-major based sorting
struct by_side_and_y 
{
  bool operator()(contours_and_centers const &a, contours_and_centers const &b) 
  {
    if (a.side != b.side)
      return a.side < b.side;    
    return a.center.y < b.center.y;
  }
};

void printHelp()
{
  cerr <<" Usage: leaf_segmentation_node image plant_nr [--jai|--nikon|--scanner]" << endl;
}

void rotate(cv::Mat& src, double angle, cv::Mat& dst)
{
  int len = std::max(src.cols, src.rows);
  cv::Point2f pt(len/2., len/2.);
  cv::Mat r = cv::getRotationMatrix2D(pt, angle, 1.0);

  cv::warpAffine(src, dst, r, cv::Size(len, len));
}

void perspectiveTransformation(cv::Mat& src, cv::Mat& dst)
{
  /* day: 30
  // Corners of original image
  vector<Point2f> srcCorners;
  srcCorners.push_back(Point2f(746, 567));
  srcCorners.push_back(Point2f(3996, 464));
  srcCorners.push_back(Point2f(831, 2894));
  srcCorners.push_back(Point2f(4086, 2793));

  // Corners of the destination image
  vector<Point2f> dstCorners;
  dstCorners.push_back(Point2f(746, 464));
  dstCorners.push_back(Point2f(4167, 464));
  dstCorners.push_back(Point2f(746, 2798));
  dstCorners.push_back(Point2f(4167, 2798));*/

  /* day: 26
  // Corners of original image
  vector<Point2f> srcCorners;
  srcCorners.push_back(Point2f(800, 577));
  srcCorners.push_back(Point2f(4005, 484));
  srcCorners.push_back(Point2f(884, 2902));
  srcCorners.push_back(Point2f(4083, 2810));

  // Corners of the destination image
  vector<Point2f> dstCorners;
  dstCorners.push_back(Point2f(800, 484));
  dstCorners.push_back(Point2f(4083, 484));
  dstCorners.push_back(Point2f(800, 2902));
  dstCorners.push_back(Point2f(4083, 2902));*/

  // day: 23
  // Corners of original image
  vector<Point2f> srcCorners;
  srcCorners.push_back(Point2f(814, 533));
  srcCorners.push_back(Point2f(4076, 437));
  srcCorners.push_back(Point2f(884, 2913));
  srcCorners.push_back(Point2f(4140, 2838));

  // Corners of the destination image
  vector<Point2f> dstCorners;
  dstCorners.push_back(Point2f(814, 437));
  dstCorners.push_back(Point2f(4140, 437));
  dstCorners.push_back(Point2f(814, 2913));
  dstCorners.push_back(Point2f(4140, 2913));

  // Get transformation matrix
  Mat transmtx = getPerspectiveTransform(srcCorners, dstCorners);

  // Apply perspective transformation
  warpPerspective(src, dst, transmtx, dst.size());
}

int main( int argc, char** argv )
{
  if( argc != 4 ) { printHelp(); return -1; }

  DeviceMode mode;
  if (std::string(argv[3]) == "--jai") {          mode = JAI; }
  else if (std::string(argv[3]) == "--nikon")   { mode = NIKON; }
  else if (std::string(argv[3]) == "--scanner") { mode = SCANNER; }
  else { printHelp(); return -1; }

  Mat imgOriginal = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
  int plant_nr = atoi(argv[2]);

  if(!imgOriginal.data)                              // Check for invalid input
  {
    cerr <<  "Could not open or find the image" << endl ;
    return -1;
  }

  // create file iterator to open the next image if needed
  path imagePath(argv[1]);
  typedef vector<path> vec;             // store paths,
  vec filenames;                        // so we can sort them later
  vec::const_iterator dirIter;

  // copy all file names to a vector
  copy(directory_iterator(imagePath.parent_path()), directory_iterator(), back_inserter(filenames));

  // sort, since directory iteration
  sort(filenames.begin(), filenames.end());

  // find the position of the actual file
  for (dirIter = filenames.begin(); dirIter != filenames.end(); ++dirIter)
  {
    if (*dirIter == imagePath.string())
    {
      // leave loop if position of actual file is reached
      break;
    }
  }
  // leave program if image do not exist
  if (dirIter >= filenames.end())
  {
    cerr <<  "Could not open or find the image" << endl ;
    return -1;
  }

  namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
  namedWindow("Thresholded Image", WINDOW_NORMAL); //create a window called "Threholded Image"
  namedWindow("Original", WINDOW_NORMAL); //create a window called "Original"
  namedWindow("Contours", WINDOW_NORMAL);
  namedWindow("Original with Contour", WINDOW_NORMAL); //create a window called "Original with Contour"

  moveWindow("Control", 0, 0);
  moveWindow("Thresholded Image", 400, 0);
  moveWindow("Contours", 400, 400);
  moveWindow("Original", 800, 0);
  moveWindow("Original with Contour", 800, 400);

  // values for green color
  int iLowH, iHighH;
  int iLowS, iHighS;
  int iLowV, iHighV;
  int min_area_px, max_area_px;

  int applyOpening = 1;
  int applyClosing = 1;
  int fontSize = 45;
  RNG rng(12345);

  // day: 30
  //Rect cropRect(746, 464, 4167 - 746, 2798 - 464);
  // day: 26
  //Rect cropRect(800, 484, 4083 - 800, 2902 - 464);
  // day: 23
  Rect cropRect(814, 437, 4140 - 814, 2913 - 437);

  AreaConverter area_conv(mode);

  // set default values:
  switch (mode)
  {
  case JAI:
    iLowH = 0; iHighH = 110;
    iLowS = 0; iHighS = 255;
    iLowV = 40; iHighV = 120;

    min_area_px = 30000;
    max_area_px = 500000;
    break;

  case NIKON:
    iLowH = 7; iHighH = 100;
    iLowS = 0; iHighS = 255;
    iLowV = 0; iHighV = 150;

    min_area_px = 20000;
    max_area_px = 2000000;

    // get plant number from filename
    plant_nr = atoi(dirIter->stem().string().substr(0, 2).c_str());

    // perspective transformation to remove perspective distortion
    perspectiveTransformation(imgOriginal, imgOriginal);

    // crop image
    imgOriginal(cropRect).copyTo(imgOriginal);

    imwrite("/tmp/dummy.png", imgOriginal);
    break;

  case SCANNER:
    iLowH = 10; iHighH = 70;
    iLowS =  0; iHighS = 255;
    iLowV =  0; iHighV = 190;

    min_area_px = 1000;
    max_area_px = 10000;
    break;    
  }


  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 110); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 110);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);

  cvCreateTrackbar("Apply opening", "Control", &applyOpening, 1); //Value (0 or 1)
  cvCreateTrackbar("Apply closing", "Control", &applyClosing, 1); //Value (0 or 1)

  cvCreateTrackbar("Minimum area in px", "Control", &min_area_px, 200000);
  cvCreateTrackbar("Maximum area in px", "Control", &max_area_px, 10000000);

  cvCreateTrackbar("Font size", "Control", &fontSize, 100);

  vector<contours_and_centers> leafs;
  Mat drawing, imgOriginalContour;

  bool done = false;
  //  while (true)
  while (!done)
  {
    // segmentation
    Mat imgHSV;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    Mat imgThresholded;
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    if(applyOpening)
    {
      //morphological opening (remove small objects from the foreground)
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    }
    if(applyClosing)
    {
      //morphological closing (fill small holes in the foreground)
      dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
      erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    }
    imshow("Thresholded Image", imgThresholded); //show the thresholded image
    imshow("Original", imgOriginal); //show the original image

    // contours
    vector<vector<Point> > contours, contours_filtered;
    vector<Vec4i> hierarchy;
    findContours( imgThresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    ///  Get the moments:
    vector<Moments> mu(contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    { 
      mu[i] = moments( contours[i], false ); 
    }

    ///  Get the mass centers:
    /// Note: this assumes that the leaves are placed on the paper in two columns (column-major)
    vector<Point2f> mc, mc_left, mc_right;
    leafs.clear();
    for( int i = 0; i < contours.size(); i++ )
    {
      int area_px = contourArea(contours[i]);
      if( (area_px >= min_area_px) && (area_px <= max_area_px) )
      {
        contours_and_centers leaf;

        leaf.contour = contours[i];
        leaf.center = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        leaf.side = int(leaf.center.x >= .5 * imgThresholded.cols);
        leafs.push_back(leaf);
      }
    }

    std::sort(leafs.begin(), leafs.end(), by_side_and_y());
      
    for( int i = 0; i < leafs.size(); i++ )
    { 
      contours_filtered.push_back(leafs[i].contour);
    }
    
    /// drawing
    cerr << endl;
    drawing = Mat::zeros( imgThresholded.size(), CV_8UC3 );
    imgOriginal.copyTo(imgOriginalContour);

    for ( int i = 0; i < leafs.size(); i++ )
    {
      int area_px = contourArea(leafs[i].contour);
      float leafLength_px, leafWidth_px;
      RotatedRect minRect = minAreaRect(leafs[i].contour);

      if (minRect.size.height < minRect.size.width)
      {
        leafLength_px = minRect.size.width;
        leafWidth_px = minRect.size.height;
      }
      else
      {
        leafLength_px = minRect.size.height;
        leafWidth_px = minRect.size.width;
      }

      leafs[i].length = area_conv.px2length((double)leafLength_px);
      leafs[i].width = area_conv.px2width((double)leafWidth_px);
      cerr << " * Contour[" << i << "] - Area Pixel: " << area_px << " - Area cm2: " 
           << area_conv.px2cm2(area_px) << " - ContourLength: " << arcLength(leafs[i].contour, true)
           << " - LeafLength: " << leafs[i].length << " - LeafWidth: " << leafs[i].width << endl;
      Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
      drawContours( drawing, contours_filtered, i, color, 8, 8, hierarchy, 0, Point() );
      drawContours( imgOriginalContour, contours_filtered, i, color, 8, 8, hierarchy, 0, Point() );
      std::ostringstream str;
      str << "id " << i+1 << " cm2 " << area_conv.px2cm2(area_px);
      putText( drawing, str.str(), leafs[i].center, FONT_HERSHEY_COMPLEX_SMALL, fontSize*0.1, cvScalar(200,200,250), 4, CV_AA);
    }

    imshow("Contours", drawing );
    imshow("Original with Contour", imgOriginalContour); //show the original image with Contour
      
    int inputKey = cv::waitKey(1000) & 0xff;

    // save image if 'ESC' or 's' was pressed
    //if (inputKey == 27 || inputKey == 's')
    if (true)
    {
      // write to file
      char outputfilename_yaml[1024];
      char outputfilename_contours[1024];
      if (NIKON != mode)
      {
        sprintf(outputfilename_yaml, "/media/pad1pal/data/data/phenotyping/dji_plant_counting/20160629/contours/ground_truth_%06d.yaml", plant_nr);
        sprintf(outputfilename_contours, "/media/pad1pal/data/data/phenotyping/dji_plant_counting/20160629/contours/ground_truth_%06d.png", plant_nr);
      }
      else
      {
        sprintf(outputfilename_yaml, "/tmp/ground_truth_%s.yaml", dirIter->stem().string().c_str());
        sprintf(outputfilename_contours, "/tmp/ground_truth_%s.png", dirIter->stem().string().c_str());
      }
      cv::imwrite(outputfilename_contours, imgOriginalContour);


      std::ofstream gt(outputfilename_yaml);
      YAML::Emitter emitter;
      double total_area = 0.0;

      emitter << YAML::BeginMap;
      emitter << YAML::Key << "plant" << YAML::Value << plant_nr;
      emitter << YAML::Key << "gps_location" << YAML::Value << "loc";
      emitter << YAML::Key << "leafs" << YAML::Value << leafs.size();
      emitter << YAML::Key << "leaf" << YAML::Value << YAML::BeginSeq;
      for (int i = 0; i < leafs.size(); i++)
      {
        double area = area_conv.px2cm2(contourArea(leafs[i].contour));

        emitter <<  YAML::BeginMap;
        emitter << YAML::Key << "leaf_id" << YAML::Value << i+1;
        emitter << YAML::Key << "leaf_area" << YAML::Value << area;
        emitter << YAML::Key << "leaf_incline" << YAML::Value << "incline";
        emitter << YAML::Key << "leaf_width" << YAML::Value << leafs[i].width;
        emitter << YAML::Key << "leaf_length" << YAML::Value << leafs[i].length;
        emitter <<  YAML::EndMap;
        total_area += area;
      }
      emitter << YAML::EndSeq;
      emitter << YAML::Key << "plant_area" << YAML::Value << total_area;
      emitter << YAML::EndMap;

      gt << emitter.c_str();

      // switch to next picture
      inputKey = 'n';

      // leave program if 'ESC' was pressed
      if (inputKey == 27)
      {
        cerr << "exiting" << endl;
        break;
      }
    }

    // load next image if 'n' was pressed (allowed only in NIKON mode)
    if (NIKON == mode &&
        inputKey == 'n')
    {
      // switch to next image if available
      if (++dirIter >= filenames.end()) {
        cerr <<  "Could not open or find the image" << endl;
        return -1;
      }

      // load new image
      imgOriginal = imread(dirIter->string(), CV_LOAD_IMAGE_COLOR);   // Read the file
      if (!imgOriginal.data)                                // Check for invalid input
      {
        cerr <<  "Could not open or find the next image" << endl;
        return -1;
      }
      else
      {
        cerr <<  "image: " << dirIter->string() << endl;
      }

      // get plant number of the new image
      plant_nr = atoi(dirIter->stem().string().substr(0, 2).c_str());

      // perspective transformation to remove perspective distortion
      perspectiveTransformation(imgOriginal, imgOriginal);

      // crop image
      imgOriginal(cropRect).copyTo(imgOriginal);
    }
    done = true;
  } 

  return 0; 
}
