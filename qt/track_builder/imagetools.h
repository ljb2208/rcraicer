#ifndef IMAGETOOLS_H
#define IMAGETOOLS_H

#include <QImage>
#include <cv.hpp>

class ImageTools
{
public:
    ImageTools();

    QImage getQImage(cv::Mat image);
    QImage applyRedFilter(cv::Mat image);
};

#endif // IMAGETOOLS_H
