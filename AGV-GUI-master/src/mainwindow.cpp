#include "includes/mainwindow.h"
#include "includes/buttons.h"
#include "includes/labels.h"
#include "ui_mainwindow.h"
#include <QTime>
#include <QTimer>
#include <QDateTime>
#include <QString>
#include <sstream>
#include <QtMultimedia/QMediaPlayer>

//initializing
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow) {
        qRegisterMetaType<std::string>();
        qRegisterMetaType<int>();

        //init mainwindow
        ui->setupUi(this);
        initAllLabels();
        slot_initAllBtns(); // initialise all buttons
        slot_initAllParams();

        //set talker node and thread
        m_talkerNode = new TalkerNode();
        m_talkerNode->initNode();
        connect(this, SIGNAL(sendPath(std::string)), m_talkerNode, SLOT(slot_sendPathfile(std::string)));
        connect(this, SIGNAL(sendJoyCommand(int)), m_talkerNode, SLOT(slot_sendJoyCommand(int)));
        m_talkerThread = new QThread();
        connect(m_talkerNode, SIGNAL(finished()), m_talkerThread, SLOT(quit()));
        m_talkerNode->moveToThread(m_talkerThread);

        //set listener node and thread
        m_listenerNode = new ListenerNode();
        connect(this, SIGNAL(init()), m_listenerNode, SLOT(slot_startListening()));
        m_listenerThread = new QThread();
        connect(m_listenerNode, SIGNAL(finished()), m_listenerThread, SLOT(quit()));
        m_listenerNode->moveToThread(m_listenerThread);

        // connect ROS topic signals to qml
        connect(m_listenerNode, SIGNAL(speedOdom(double)), this, SLOT(slot_displaySpeed(double)));
        connect(m_listenerNode, SIGNAL(steeringAngle(double)), this, SLOT(slot_displayAngle(double)));
        connect(m_listenerNode, SIGNAL(brakeState(int)), this, SLOT(slot_displayBrake(int)));
        connect(m_listenerNode, SIGNAL(acceleration(double)), this, SLOT(slot_displayDAC(double)));
        connect(m_listenerNode, SIGNAL(vehicleMode(int)), this, SLOT(slot_displayMode(int)));


        //memory clear
        connect(this, SIGNAL(canQuit()), qApp, SLOT(quit()));

        //dash board
        QQuickView *qmlview = new QQuickView();
        QWidget *qmlcontainer = QWidget::createWindowContainer(qmlview, this);
        qmlcontainer->setMinimumSize(570, 550);
        qmlcontainer->setMaximumSize(570, 550);
        qmlcontainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        ui->gridLayout_dashbd->addWidget(qmlcontainer);
        ui->gridLayout_dashbd->setAlignment(qmlcontainer, Qt::AlignTop);
        qmlview->setSource(QUrl("qrc:/qml/Dial.qml"));
        qmlview->engine()->rootContext()->setContextProperty("qspeed", &qspeed);
        qmlview->engine()->rootContext()->setContextProperty("qangle", &qangle);
        qmlview->engine()->rootContext()->setContextProperty("qbrake", &qbrake);
        qmlview->engine()->rootContext()->setContextProperty("qacceleration", &qacceleration);
        qmlview->engine()->rootContext()->setContextProperty("qmode", &qmode);

        //start threads
        m_talkerThread->start();
        m_listenerThread->start();
        //m_markerThread->start();
        this->setWindowState(Qt::WindowMaximized);
        //m_vizlibWidget->rviz_display();
        sleep(1);
        emit init();
    }

    /* Interaction Functions */
    MainWindow::~MainWindow(){
        delete ui;
    }

    void MainWindow::slot_initAllParams() {
        qspeed.setSpeed(0); //velocity = 0
        qangle.setAngle(0); //steering angle = 0
        qbrake.setBrake(0); //no brake
        qpath.setPath(false); //no path yet
        qmode.setMode(0); //parking/brake mode
        qmusic.setPlaying(false); //music not playing
        qobstacle.setObstacle(100.0); //obstacle far
        qacceleration.setAccelerationLevel(0); //no motion
    }

    void MainWindow::slot_displaySpeed(double speed) {
        qspeed.setSpeed(speed);
    }

    void MainWindow::slot_displayAngle(double angle) {
        qangle.setAngle(angle);
    }

    void MainWindow::slot_displayBrake(int brake_state) {
        qbrake.setBrake(brake_state);
    }

    void MainWindow::slot_displayDAC(double acceleration) { // 0 <= acceleration <= 5
        if(acceleration <= 0.2) {
            qacceleration.setAccelerationLevel(0);
        }
        else if (acceleration <= 1.0) {
            qacceleration.setAccelerationLevel(1);
        }
        else if (acceleration <= 2.0) {
            qacceleration.setAccelerationLevel(2);
        }
        else if (acceleration <= 3.0) {
            qacceleration.setAccelerationLevel(3);
        }
        else if (acceleration <= 4.0) {
            qacceleration.setAccelerationLevel(4);
        }
        else if (acceleration <= 5.0) {
            qacceleration.setAccelerationLevel(5);
        }
        else {
            qacceleration.setAccelerationLevel(0);
        }
    }

    void MainWindow::slot_displayMode(int vehicle_mode) {
        qmode.setMode(vehicle_mode);
    }

    void MainWindow::slot_displayObstacle(double closest_dist) {
        qobstacle.setObstacle(closest_dist);
        if(closest_dist <= 15.0) {
            ui->label_obstacle->show();
        }
        else {
            ui->label_obstacle->hide();
        }
    }
