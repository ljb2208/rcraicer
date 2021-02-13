#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QWidget>
#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

class ImageViewer : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit ImageViewer(QGraphicsScene *parent = nullptr);
    virtual void mousePressEvent(QGraphicsSceneMouseEvent * mouseEvent);

signals:
    void mousePress(const int x, const int y);

};

#endif // IMAGEVIEWER_H
