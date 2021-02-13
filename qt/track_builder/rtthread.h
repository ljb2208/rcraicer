#ifndef RTTHREAD_H
#define RTTHREAD_H

#include <QImage>
#include <QThread>
#include <QMutex>
#include <QWaitCondition>
#include <cv.hpp>
#include "ximeacamera.h"

class RTThread : public QThread
{
    Q_OBJECT
public:
    explicit RTThread(QObject *parent = nullptr);
    ~RTThread();

    void stop();

signals:
    void imagesReceived(const cv::Mat &leftImage, const cv::Mat &rightImage);

protected:
    void run() override;

    XimeaCamera* leftCamera;
    XimeaCamera* rightCamera;

    QMutex mutex;
    QWaitCondition condition;

    cv::Mat leftImage;
    cv::Mat rightImage;

    bool running;

};

#endif // RTTHREAD_H
