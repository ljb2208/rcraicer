#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QtDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);    

    connect(&leftImageViewer, &ImageViewer::mousePress, this, &MainWindow::onLeftImageClicked);
    connect(&rightImageViewer, &ImageViewer::mousePress, this, &MainWindow::onRightImageClicked);

    ui->leftView->setScene(&leftImageViewer);
    ui->leftView->scene()->addItem(&leftPixMap);

    ui->filterView->setScene(&filterImageViewer);
    ui->filterView->scene()->addItem(&filterPixMap);

    ui->rightView->setScene(&rightImageViewer);
    ui->rightView->scene()->addItem(&rightPixMap);



    qRegisterMetaType< cv::Mat >("cv::Mat");
    connect(&camThread, &RTThread::imagesReceived, this, &MainWindow::onImagesReceived);
    connected = false;

    captureMode = false;
}

MainWindow::~MainWindow()
{    
    if (camThread.isRunning())
    {
        camThread.stop();
        camThread.wait();
    }


    delete ui;
}


void MainWindow::onLeftImageClicked(int x, int y)
{

}

void MainWindow::onRightImageClicked(int x, int y)
{

}

void MainWindow::onImagesReceived(cv::Mat leftImage, cv::Mat rightImage)
{
    if (!captureMode)
    {
        imageMutex.lock();
        currentLeftImage = leftImage.clone();
        currentRightImage = rightImage.clone();
        imageMutex.unlock();

        leftPixMap.setPixmap(QPixmap::fromImage(imageTools.getQImage(leftImage).rgbSwapped()));
        ui->leftView->fitInView(&leftPixMap, Qt::KeepAspectRatio);

        filterImage.load("qtest.png");
    //    qDebug("Fitler image: %i/%i", filterImage.width(), filterImage.height());
        rightPixMap.setPixmap(QPixmap::fromImage(filterImage));
        ui->rightView->fitInView(&rightPixMap, Qt::KeepAspectRatio);

//        rightPixMap.setPixmap(QPixmap::fromImage(imageTools.getQImage(rightImage).rgbSwapped()));
//        ui->rightView->fitInView(&rightPixMap, Qt::KeepAspectRatio);
    }
}

void MainWindow::on_connectCamerasBtn_clicked()
{
    if (!connected)
    {
        ui->connectCamerasBtn->setText("Stop");
        camThread.start();
        connected = true;
    }
    else
    {
        ui->connectCamerasBtn->setText("Start");
        camThread.stop();
        connected = false;
    }
}

void MainWindow::on_captureBtn_clicked()
{
    if (!captureMode)
    {
        captureMode = true;
        ui->captureBtn->setText("Stream");

//        cv::Mat savedImage = cv::imread("test.png");
//        filterPixMap.setPixmap((QPixmap::fromImage(imageTools.getQImage(savedImage))));
        ImageTools tool;

        imageMutex.lock();
        filterImage = tool.applyRedFilter(currentLeftImage);
        imageMutex.unlock();
        filterImage.save("qtest.png");
        qDebug("Fitler image: %i/%i", filterImage.width(), filterImage.height());

        filterPixMap.setPixmap(QPixmap::fromImage(filterImage));

        ui->filterView->fitInView(&filterPixMap, Qt::KeepAspectRatio);
        ui->filterView->setSceneRect(filterImage.rect());
//        ui->filterView->show();

    }
    else
    {
        captureMode = false;
        ui->captureBtn->setText("Capture");
    }

}
