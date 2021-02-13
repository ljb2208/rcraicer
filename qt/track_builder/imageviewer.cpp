#include "imageviewer.h"

ImageViewer::ImageViewer(QGraphicsScene *parent) : QGraphicsScene(parent)
{

}

void ImageViewer::mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent)
{
    emit mousePress(mouseEvent->scenePos().x(), mouseEvent->scenePos().y());
}
