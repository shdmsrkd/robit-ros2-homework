#include "../include/camera_qt_ui_pkg/main_window.hpp"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);
    
    ui->label->setFixedSize(640, 480);
    
    ui->label->setAlignment(Qt::AlignCenter);
    
    qnode = new QNode();
    
    QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
    QObject::connect(qnode, SIGNAL(imageReceived(QImage)), this, SLOT(updateCameraImage(QImage)));
    
    qDebug() << "MainWindow initialized with label size:" << ui->label->size();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    QMainWindow::closeEvent(event);
}

void MainWindow::updateCameraImage(const QImage &image)
{
    if (image.isNull()) {
        qDebug() << "Received null image";
        return;
    }
    
    qDebug() << "Original image size:" << image.width() << "x" << image.height();
    
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaled = pixmap.scaled(ui->label->size(), 
                                 Qt::KeepAspectRatio,  
                                 Qt::SmoothTransformation);  
    
    qDebug() << "Scaled image size:" << scaled.width() << "x" << scaled.height
    
    ui->label->setPixmap(scaled);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete qnode;
}
