#include "imagetools.h"
#include <vector>

ImageTools::ImageTools()
{

}


QImage ImageTools::getQImage(cv::Mat image)
{
    QImage qImage;

    if (image.type() == CV_8UC1)
    {
        qImage = QImage(image.data, image.cols, image.rows, static_cast<int>(image.step), QImage::Format_Grayscale8);
    }
    else
    {
        qImage = QImage(image.data, image.cols, image.rows, static_cast<int>(image.step), QImage::Format_RGB888);
    }

    return qImage;
}

QImage ImageTools::applyRedFilter(cv::Mat image)
{
    QImage qImage;

    cv::Mat channels[3];
    cv::split(image, channels);

    cv::Mat filterImage;
    filterImage.create(image.rows, image.cols, CV_8UC1);

    cv::Mat newFilterImage;
    newFilterImage.create(image.rows, image.cols, CV_8UC3);

    for (int cols=0; cols < image.cols; cols++)
    {
        for (int rows=0; rows < image.rows; rows++)
        {
            int r = (int)channels[2].at<uchar>(rows, cols);
            int g = (int)channels[1].at<uchar>(rows, cols);
            int b = (int)channels[0].at<uchar>(rows, cols);

            int newVal = r*2 -g -b;

            if (newVal < 155)
            {
                newVal = 0;
            }

            if (newVal > 255)
            {
                newVal = 255;
            }

            channels[0].at<uchar>(rows, cols) = newVal;
            channels[1].at<uchar>(rows, cols) = newVal;
            channels[2].at<uchar>(rows, cols) = newVal;
        }
    }

//    channels[1]=cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
//    channels[2]=cv::Mat::zeros(image.rows, image.cols, CV_8UC1);

    cv::merge(channels, 3, newFilterImage);

//    return getQImage(filterImage);
    return getQImage(newFilterImage);
}
