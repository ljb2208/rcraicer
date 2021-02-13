#include "rtthread.h"

RTThread::RTThread(QObject *parent) : QThread(parent)
{
    running = false;
    leftCamera = new XimeaCamera("31703351");
    rightCamera = new XimeaCamera("32703551");
}


RTThread::~RTThread()
{
    delete leftCamera;
    delete rightCamera;
}

void RTThread::stop()
{
    running = false;
}


void RTThread::run()
{
    leftCamera->open();
    rightCamera->open();

    running = true;

    while(running) {
        mutex.lock();

        leftCamera->getImage(leftImage);
        rightCamera->getImage(rightImage);

        mutex.unlock();

        emit imagesReceived(leftImage, rightImage);
    }

    leftCamera->close();
    rightCamera->close();
}
