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

    Path2D getGestureX()
        {
            Path2D path;
            path.push_back(Point2D(87,142));path.push_back(Point2D(89,145));path.push_back(Point2D(91,148));path.push_back(Point2D(93,151));
            path.push_back(Point2D(96,155));path.push_back(Point2D(98,157));path.push_back(Point2D(100,160));path.push_back(Point2D(102,162));
            path.push_back(Point2D(106,167));path.push_back(Point2D(108,169));path.push_back(Point2D(110,171));path.push_back(Point2D(115,177));
            path.push_back(Point2D(119,183));path.push_back(Point2D(123,189));path.push_back(Point2D(127,193));path.push_back(Point2D(129,196));
            path.push_back(Point2D(133,200));path.push_back(Point2D(137,206));path.push_back(Point2D(140,209));path.push_back(Point2D(143,212));
            path.push_back(Point2D(146,215));path.push_back(Point2D(151,220));path.push_back(Point2D(153,222));path.push_back(Point2D(155,223));
            path.push_back(Point2D(157,225));path.push_back(Point2D(158,223));path.push_back(Point2D(157,218));path.push_back(Point2D(155,211));
            path.push_back(Point2D(154,208));path.push_back(Point2D(152,200));path.push_back(Point2D(150,189));path.push_back(Point2D(148,179));
            path.push_back(Point2D(147,170));path.push_back(Point2D(147,158));path.push_back(Point2D(147,148));path.push_back(Point2D(147,141));
            path.push_back(Point2D(147,136));path.push_back(Point2D(144,135));path.push_back(Point2D(142,137));path.push_back(Point2D(140,139));
            path.push_back(Point2D(135,145));path.push_back(Point2D(131,152));path.push_back(Point2D(124,163));path.push_back(Point2D(116,177));
            path.push_back(Point2D(108,191));path.push_back(Point2D(100,206));path.push_back(Point2D(94,217));path.push_back(Point2D(91,222));
            path.push_back(Point2D(89,225));path.push_back(Point2D(87,226));path.push_back(Point2D(87,224));

            return path;
        }

    Path2D getGestureXInv()
        {
            Path2D path;
            path.push_back(Point2D(87,224));path.push_back(Point2D(87,226));path.push_back(Point2D(89,225));path.push_back(Point2D(91,222));
            path.push_back(Point2D(94,217));path.push_back(Point2D(100,206));path.push_back(Point2D(108,191));path.push_back(Point2D(116,177));
            path.push_back(Point2D(124,163));path.push_back(Point2D(131,152));path.push_back(Point2D(135,145));path.push_back(Point2D(140,139));
            path.push_back(Point2D(142,137));path.push_back(Point2D(144,135));path.push_back(Point2D(147,136));path.push_back(Point2D(147,141));
            path.push_back(Point2D(147,148));path.push_back(Point2D(147,158));path.push_back(Point2D(147,170));path.push_back(Point2D(148,179));
            path.push_back(Point2D(150,189));path.push_back(Point2D(152,200));path.push_back(Point2D(154,208));path.push_back(Point2D(155,211));
            path.push_back(Point2D(157,218));path.push_back(Point2D(158,223));path.push_back(Point2D(157,225));path.push_back(Point2D(155,223));
            path.push_back(Point2D(153,222));path.push_back(Point2D(151,220));path.push_back(Point2D(146,215));path.push_back(Point2D(143,212));
            path.push_back(Point2D(140,209));path.push_back(Point2D(137,206));path.push_back(Point2D(133,200));path.push_back(Point2D(129,196));
            path.push_back(Point2D(127,193));path.push_back(Point2D(123,189));path.push_back(Point2D(119,183));path.push_back(Point2D(115,177));
            path.push_back(Point2D(110,171));path.push_back(Point2D(108,169));path.push_back(Point2D(106,167));path.push_back(Point2D(102,162));
            path.push_back(Point2D(100,160));path.push_back(Point2D(98,157));path.push_back(Point2D(96,155));path.push_back(Point2D(93,151));
            path.push_back(Point2D(91,148));path.push_back(Point2D(89,145));path.push_back(Point2D(87,142));

            return path;
        }

    Path2D getGestureStroke()
        {
            Path2D path;
            path.push_back(Point2D(80,140));path.push_back(Point2D(82,140));path.push_back(Point2D(84,141));path.push_back(Point2D(86,142));
            path.push_back(Point2D(88,142));path.push_back(Point2D(90,143));path.push_back(Point2D(92,144));path.push_back(Point2D(94,144));
            path.push_back(Point2D(96,145));path.push_back(Point2D(98,146));path.push_back(Point2D(100,146));path.push_back(Point2D(102,147));
            path.push_back(Point2D(104,148));path.push_back(Point2D(106,148));path.push_back(Point2D(108,149));path.push_back(Point2D(110,150));
            path.push_back(Point2D(112,150));path.push_back(Point2D(114,151));path.push_back(Point2D(116,152));path.push_back(Point2D(118,152));
            path.push_back(Point2D(120,153));path.push_back(Point2D(122,154));path.push_back(Point2D(124,154));path.push_back(Point2D(126,155));
            path.push_back(Point2D(128,156));path.push_back(Point2D(130,156));path.push_back(Point2D(132,157));path.push_back(Point2D(134,158));
            path.push_back(Point2D(136,158));path.push_back(Point2D(138,159));path.push_back(Point2D(140,160));path.push_back(Point2D(142,160));
            path.push_back(Point2D(144,161));path.push_back(Point2D(146,162));path.push_back(Point2D(148,162));path.push_back(Point2D(150,163));
            path.push_back(Point2D(152,164));path.push_back(Point2D(154,164));path.push_back(Point2D(156,165));path.push_back(Point2D(158,166));
            path.push_back(Point2D(160,166));path.push_back(Point2D(162,167));path.push_back(Point2D(164,168));path.push_back(Point2D(166,168));
            path.push_back(Point2D(168,169));path.push_back(Point2D(170,170));path.push_back(Point2D(172,170));path.push_back(Point2D(174,171));
            path.push_back(Point2D(176,172));path.push_back(Point2D(178,172));path.push_back(Point2D(180,173));

            return path;
        }

    Path2D getGesturePerfectStroke()
        {
            Path2D path;
            path.push_back(Point2D(80,140));path.push_back(Point2D(82,140));path.push_back(Point2D(84,140));path.push_back(Point2D(86,140));
            path.push_back(Point2D(88,140));path.push_back(Point2D(90,140));path.push_back(Point2D(92,140));path.push_back(Point2D(94,140));
            path.push_back(Point2D(96,140));path.push_back(Point2D(98,140));path.push_back(Point2D(100,140));path.push_back(Point2D(102,140));
            path.push_back(Point2D(104,140));path.push_back(Point2D(106,140));path.push_back(Point2D(108,140));path.push_back(Point2D(110,140));
            path.push_back(Point2D(112,140));path.push_back(Point2D(114,140));path.push_back(Point2D(116,140));path.push_back(Point2D(118,140));
            path.push_back(Point2D(120,140));path.push_back(Point2D(122,140));path.push_back(Point2D(124,140));path.push_back(Point2D(126,140));
            path.push_back(Point2D(128,140));path.push_back(Point2D(130,140));path.push_back(Point2D(132,140));path.push_back(Point2D(134,140));
            path.push_back(Point2D(136,140));path.push_back(Point2D(138,140));path.push_back(Point2D(140,140));path.push_back(Point2D(142,140));
            path.push_back(Point2D(144,140));path.push_back(Point2D(146,140));path.push_back(Point2D(148,140));path.push_back(Point2D(150,140));
            path.push_back(Point2D(152,140));path.push_back(Point2D(154,140));path.push_back(Point2D(156,140));path.push_back(Point2D(158,140));
            path.push_back(Point2D(160,140));path.push_back(Point2D(162,140));path.push_back(Point2D(164,140));path.push_back(Point2D(166,140));
            path.push_back(Point2D(168,140));path.push_back(Point2D(170,140));path.push_back(Point2D(172,140));path.push_back(Point2D(174,140));
            path.push_back(Point2D(176,140));path.push_back(Point2D(178,140));path.push_back(Point2D(180,140));

            return path;
        }

};

#endif // ONEDOLLARRECOGNIZEROBSERVER_H
