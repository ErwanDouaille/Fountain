#ifndef ONEDOLLARRECOGNIZEROBSERVER_H
#define ONEDOLLARRECOGNIZEROBSERVER_H

#pragma once


#include "LgObserver.h"

#define MIN_LENGTH_OF_PATH 16
#define MAX_LENGTH_OF_PATH 128


#define DISTANCE_FOR_RECOGNIZE 50
#define MIN_NB_BEFORE_RECOG 10

#define DISTANCE_FOR_ADD 5

using namespace lg;

typedef vector<Point2D>  Path2D;
typedef Path2D::iterator Path2DIterator;

class MyRectangle
{
public:
    double x, y, width, height;
    MyRectangle(double x, double y, double width, double height)
    {
        this->x = x;
        this->y = y;
        this->width = width;
        this->height = height;
    }
};


class RecognitionResult
{
public:
    string name;
    double score;
    RecognitionResult(string name, double score)
    {
        this->name = name;
        this->score = score;
    }
};

class GestureTemplate
{
public:
    string name;
    Path2D points;

    GestureTemplate(string name, Path2D points)
    {
        this->name   = name;
        this->points = points;
    }
};

typedef vector<GestureTemplate> GestureTemplates;

class OneDollarRecognizerObserver : public Observer
{
private:
    map<string,Path2D> _paths;
	map<string,int> _numPointsInGestures;
    GestureTemplates _templates;
    bool _run;

    double halfDiagonal;
    double angleRange;
    double anglePrecision;
    double goldenRatio;
    int numPointsInGesture;
    int squareSize;
    bool shouldIgnoreRotation;

public:
    OneDollarRecognizerObserver(string);
    ~OneDollarRecognizerObserver(void);

    Node* clone(string cloneName) const;

    bool start();
    bool stop();
    bool observe(map<string, Group3D *> groups3D, map<string, Group2D *> groups2D, map<string, Group1D *> groups1D, map<string, GroupSwitch *> groupsSwitch);

    set<string> need() const;

    void loadTemplates();
    int addTemplate(string name, Path2D points);

    RecognitionResult recognize(Path2D points);
    Path2D normalizePath(Path2D points);
    Path2D resample(Path2D points);
    double distanceAtBestAngle(Path2D points, GestureTemplate aTemplate);
    bool   getRotationInvariance() { return shouldIgnoreRotation; }
    Path2D rotateToZero(Path2D points);
    Path2D scaleToSquare(Path2D points);
    Path2D translateToOrigin(Path2D points);
    double pathLength(Path2D points);
    double getDistance(Point2D p1, Point2D p2);
    double distanceAtAngle(Path2D points, GestureTemplate aTemplate, double rotation);
    Point2D centroid(Path2D points);
    Path2D rotateBy(Path2D points, double rotation);
    double pathDistance(Path2D pts1, Path2D pts2);
    MyRectangle boundingBox(Path2D points);
};

struct SampleGestures
{
    Path2D getGestureCircle()
    {
        Path2D path;
        path.push_back(Point2D(127,141));path.push_back(Point2D(124,140));path.push_back(Point2D(120,139));path.push_back(Point2D(118,139));
        path.push_back(Point2D(116,139));path.push_back(Point2D(111,140));path.push_back(Point2D(109,141));path.push_back(Point2D(104,144));
        path.push_back(Point2D(100,147));path.push_back(Point2D(96,152));path.push_back(Point2D(93,157));path.push_back(Point2D(90,163));
        path.push_back(Point2D(87,169));path.push_back(Point2D(85,175));path.push_back(Point2D(83,181));path.push_back(Point2D(82,190));
        path.push_back(Point2D(82,195));path.push_back(Point2D(83,200));path.push_back(Point2D(84,205));path.push_back(Point2D(88,213));
        path.push_back(Point2D(91,216));path.push_back(Point2D(96,219));path.push_back(Point2D(103,222));path.push_back(Point2D(108,224));
        path.push_back(Point2D(111,224));path.push_back(Point2D(120,224));path.push_back(Point2D(133,223));path.push_back(Point2D(142,222));
        path.push_back(Point2D(152,218));path.push_back(Point2D(160,214));path.push_back(Point2D(167,210));path.push_back(Point2D(173,204));
        path.push_back(Point2D(178,198));path.push_back(Point2D(179,196));path.push_back(Point2D(182,188));path.push_back(Point2D(182,177));
        path.push_back(Point2D(178,167));path.push_back(Point2D(170,150));path.push_back(Point2D(163,138));path.push_back(Point2D(152,130));
        path.push_back(Point2D(143,129));path.push_back(Point2D(140,131));path.push_back(Point2D(129,136));path.push_back(Point2D(126,139));

        return path;
    }

    Path2D getGestureCircleInv()
    {
        Path2D path;
        path.push_back(Point2D(126,139));path.push_back(Point2D(129,136));path.push_back(Point2D(140,131));path.push_back(Point2D(143,129));
        path.push_back(Point2D(152,130));path.push_back(Point2D(163,138));path.push_back(Point2D(170,150));path.push_back(Point2D(178,167));
        path.push_back(Point2D(182,177));path.push_back(Point2D(182,188));path.push_back(Point2D(179,196));path.push_back(Point2D(178,198));
        path.push_back(Point2D(173,204));path.push_back(Point2D(167,210));path.push_back(Point2D(160,214));path.push_back(Point2D(152,218));
        path.push_back(Point2D(142,222));path.push_back(Point2D(133,223));path.push_back(Point2D(120,224));path.push_back(Point2D(111,224));
        path.push_back(Point2D(108,224));path.push_back(Point2D(103,222));path.push_back(Point2D(96,219));path.push_back(Point2D(91,216));
        path.push_back(Point2D(88,213));path.push_back(Point2D(84,205));path.push_back(Point2D(83,200));path.push_back(Point2D(82,195));
        path.push_back(Point2D(82,190));path.push_back(Point2D(83,181));path.push_back(Point2D(85,175));path.push_back(Point2D(87,169));
        path.push_back(Point2D(90,163));path.push_back(Point2D(93,157));path.push_back(Point2D(96,152));path.push_back(Point2D(100,147));
        path.push_back(Point2D(104,144));path.push_back(Point2D(109,141));path.push_back(Point2D(111,140));path.push_back(Point2D(116,139));
        path.push_back(Point2D(118,139));path.push_back(Point2D(120,139));path.push_back(Point2D(124,140));path.push_back(Point2D(127,141));

        return path;
    }

    Path2D getGestureRemoue()
    {
        Path2D path;
        path.push_back(Point2D(100,100));
		path.push_back(Point2D(100,200));
		path.push_back(Point2D(100,100));
		path.push_back(Point2D(100,200));
        return path;
    }

    Path2D getGestureRemoueInv()
    {
        Path2D path;
		path.push_back(Point2D(100,200));
        path.push_back(Point2D(100,100));
		path.push_back(Point2D(100,200));
		path.push_back(Point2D(100,100));
        return path;
    }


};

#endif // ONEDOLLARRECOGNIZEROBSERVER_H
