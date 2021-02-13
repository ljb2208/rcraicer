#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsPixmapItem>
#include "ximeacamera.h"
#include "rtthread.h"
#include "imageviewer.h"
#include "imagetools.h"
#include <QMutex>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onImagesReceived(cv::Mat leftImage, cv::Mat rightImage);
    void onLeftImageClicked(int x, int y);
    void onRightImageClicked(int x, int y);

    void on_connectCamerasBtn_clicked();

    void on_captureBtn_clicked();

private:
    Ui::MainWindow *ui;

    XimeaCamera* leftCamera;
    XimeaCamera* rightCamera;

    ImageViewer leftImageViewer;
    ImageViewer rightImageViewer;
    QGraphicsScene filterImageViewer;

    QGraphicsPixmapItem leftPixMap;
    QGraphicsPixmapItem rightPixMap;
    QGraphicsPixmapItem filterPixMap;

    QMutex imageMutex;

    cv::Mat currentLeftImage;
    cv::Mat currentRightImage;

    QImage filterImage;

    RTThread camThread;
    bool connected;
    bool captureMode;
    ImageTools imageTools;

};
#endif // MAINWINDOW_H
