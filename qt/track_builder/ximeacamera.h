#ifndef XIMEACAMERA_H
#define XIMEACAMERA_H

#include <QImage>
#include <string>
#include <m3api/xiApi.h> // Linux, OSX
#include <cv.hpp>

class XimeaCamera
{
public:
    XimeaCamera(std::string serialNumber);
    bool open();
    void close();
    bool getImage(cv::Mat& image);

private:
    void applySettings();

    std::string serialNumber;
    HANDLE xiH;
    XI_IMG ximage;

};

#endif // XIMEACAMERA_H
