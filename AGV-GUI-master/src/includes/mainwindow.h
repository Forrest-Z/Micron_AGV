#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <iomanip>
#include <QtQml/QQmlEngine>
#include <QtQuick/QQuickView>
#include <QQmlContext>
#include "TalkerNode.h"
#include "ListenerNode.h"
#include "includes/Settings.h"
//#include "VizlibWidget.h"
//#include "MarkerNode.h"
#include <QtMultimedia/QMediaPlayer>
#include <QTime>
#include <QTimer>

Q_DECLARE_METATYPE(std::string)
Q_DECLARE_METATYPE(int)

namespace Ui {
class MainWindow;
}

class QMode : public QObject {
    Q_OBJECT
    Q_PROPERTY(int m_current_mode READ getMode WRITE setMode NOTIFY modeChanged)
signals:
    void modeChanged();
private:
    int m_current_mode = 0; // for vehicle mode, 0 for brake mode, 1 for autonomous mode 2 for manual mode
    int m_last_mode = 0;
    int m_destination = 0; // for destination, 3 for 10A, 4 for 10N, 5 for 10X

public:
    int getMode() const {
        return m_current_mode;
    }
    void setMode(int new_mode){
        if(m_current_mode != new_mode && new_mode != -1){
            if(new_mode == 1) { // auto mode from brake mode
                if(m_current_mode == 0) {
                    m_last_mode = m_current_mode;
                    m_current_mode = 1;
                }
            }
            else {
                m_last_mode = m_current_mode;
                m_current_mode = new_mode;
            }
            emit modeChanged();
        }
    }
    void setDest(int new_dest) {
        if(m_destination != new_dest){
            m_destination = new_dest;
            emit modeChanged();
        }
    }
    int getDest() {
        return m_destination;
    }
};

class Icons {
private:
    const QPixmap *pix_start_0 = new QPixmap(":/icons/new/start_0.png");
    const QPixmap *pix_stop_0 = new QPixmap(":/icons/new/stop_0.png");
    const QPixmap *pix_fab10a_0 = new QPixmap(":/icons/new/fab10a_0.png");
    const QPixmap *pix_fab10n_0 = new QPixmap(":/icons/new/fab10n_0.png");
    const QPixmap *pix_fab10x_0 = new QPixmap(":/icons/new/fab10x_0.png");

    const QPixmap *pix_start_1 = new QPixmap(":/icons/new/start_1.png");
    const QPixmap *pix_stop_1 = new QPixmap(":/icons/new/stop_1.png");
    const QPixmap *pix_fab10a_1 = new QPixmap(":/icons/new/fab10a_1.png");
    const QPixmap *pix_fab10n_1 = new QPixmap(":/icons/new/fab10n_1.png");
    const QPixmap *pix_fab10x_1 = new QPixmap(":/icons/new/fab10x_1.png");

public:
    QPixmap getIcon(int name, bool on) {
        if(!on){
            switch (name) {
                case 1 :
                    return *pix_start_0;
                case 2 :
                    return *pix_stop_0;
                case 3 :
                    return *pix_fab10a_0;
                case 4 :
                    return *pix_fab10n_0;
                case 5 :
                    return *pix_fab10x_0;
                default :
                    qDebug() << "Invalid button name";
            }
        }
        else {
            switch (name) {
                case 1 :
                    return *pix_start_1;
                case 2 :
                    return *pix_stop_1;
                case 3 :
                    return *pix_fab10a_1;
                case 4 :
                    return *pix_fab10n_1;
                case 5 :
                    return *pix_fab10x_1;
                default :
                    qDebug() << "Invalid button name";
            }
        }
    }
    QSize getSize(int name) {
        if(name == 1 || name == 2) { //return the size for start and stop buttons
            return QSize(190, 60);
        }
        else {
            return QSize(190, 60); //return the size for destination buttons
        }
    }
};

// for qml display speed
class QSpeed : public QObject {
    Q_OBJECT
    Q_PROPERTY(double m_speed READ speed WRITE setSpeed NOTIFY speedChanged)
signals:
    void speedChanged();
private:
    double m_speed = 0.0;
    // compare if two doubles are roughly same
    bool doubleEql(double a, double b, double epsilon = 0.01){
        return (fabs(a - b) <= epsilon);
    }
public:
    double speed() const {
        return m_speed;
    }
    void setSpeed(double v){
        if(!doubleEql(m_speed, v)){
            m_speed = v;
            emit speedChanged();
        }
    }
};

// for qml display steering angle
class QAngle : public QObject {
    Q_OBJECT
    Q_PROPERTY(double m_angle READ angle WRITE setAngle NOTIFY angleChanged)
signals:
    void angleChanged();
private:
    double m_angle = 0.0;
    // compare if two doubles are roughly same
    bool doubleEql(double a, double b, double epsilon = 0.01){
        return (fabs(a - b) <= epsilon);
    }
public:
    double angle() const {
        return m_angle;
    }
    void setAngle(double a){
        if(!doubleEql(m_angle, a)){
            m_angle = a;
            emit angleChanged();
        }
    }
};

// for qml display steering angle
class QBrake : public QObject {
    Q_OBJECT
    Q_PROPERTY(int m_brake READ brake_state WRITE setBrake NOTIFY brakeChanged)
signals:
    void brakeChanged();
private:
    int m_brake = 0;
public:
    int brake_state() const {
        return m_brake;
    }
    void setBrake(int a){
        if(m_brake != a){
            m_brake = a;
            emit brakeChanged();
        }
    }
};

// for displaying obstacle info
class QObstacle : public QObject {
    Q_OBJECT
    Q_PROPERTY(double m_obstacle_dist READ getObstacle WRITE setObstacle NOTIFY obstacleChanged)
signals:
    void obstacleChanged();
private:
    double m_obstacle_dist = 99999.9;
public:
    bool doubleEql(double a, double b, double epsilon = 0.01){
        return (fabs(a - b) <= epsilon);
    }
    double getObstacle() const {
        return m_obstacle_dist;
    }
    void setObstacle(double a){
        if(!doubleEql(m_obstacle_dist, a)){
            m_obstacle_dist = a;
            emit obstacleChanged();
        }
    }
};

// for qml display steering angle
class QPath : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool m_path_exist READ path_exist NOTIFY path_existChanged)
signals:
    void path_existChanged();
private:
    bool m_path_exist = false;

public:
    bool path_exist() const {
        return m_path_exist;
    }
    void setPath(bool a){
        if(m_path_exist != a){
            m_path_exist = a;
            emit path_existChanged();
        }
    }
};

class QMusic : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool m_Playing READ isPlaying NOTIFY musicChanged)
signals:
    void musicChanged();
private:
    bool m_playing = false;
    QMediaPlayer *player = new QMediaPlayer;

public:
    bool isPlaying() const {
        return m_playing;
    }
    void setPlaying(bool a){
        if(m_playing != a){
            m_playing = a;
            emit musicChanged();
        }
    }
    void startPlaying() {
        if(isPlaying()) {
            player->play();
            setPlaying(true);
            qDebug() << "music resumed";
        }
        else {
            player->setMedia(QUrl::fromLocalFile("home/agv/UI/AGV-UI/src/multimedia/something-just-like-this.mp3")); //absolute file path
            player->setVolume(100);
            player->play();
            setPlaying(true);
            qDebug() << "music starting";
        }
    }
    void pausePlaying() {
        if(isPlaying()) {
            player->pause();
            qDebug() << "music paused";
        }
        else {
            qDebug() << "music is playing";
        }
    }
};

class QCountDown : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool m_timer_exist READ isTiming NOTIFY timerChanged)
signals:
    void timerChanged();
private:
    const int remainingTime = 3; // 3 seconds
    int interval = 1; // 1 second
    bool m_timer_exist = false;
    QTimer *timer = new QTimer();
    QTime *qtime = new QTime(0, 0, remainingTime); // 3 seconds

public:
    QTimer* getTimer() {
        return timer;
    }
    bool isTiming() const {
        return m_timer_exist;
    }
    void setTiming(bool a){
        if(m_timer_exist != a){
            m_timer_exist = a;
            emit timerChanged();
        }
    }
    int getRemainingTime(){
        return qtime->second();
    }
    int setRemainingTime(int t){
        if(qtime->second() != t){
            qtime->setHMS(0, 0, t);
            emit timerChanged();
        }
        return qtime->second();
    }
    void startCountDown() {
        qtime->setHMS(0, 0, remainingTime);
        timer->start(interval*1000);
        qDebug() << "timer started";
        setTiming(true);
    }
};

// for qml display dac acceleration
class QAcceleration : public QObject {
    Q_OBJECT
    Q_PROPERTY(int m_acceleration_level READ getAccelerationLevel WRITE setAccelerationLevel NOTIFY accelerationChanged)
signals:
    void accelerationChanged();
private:
    int m_acceleration_level = 0;
public:
    int getAccelerationLevel() const {
        return m_acceleration_level;
    }
    void setAccelerationLevel(int a){
        if(m_acceleration_level != a){
            m_acceleration_level = a;
            emit accelerationChanged();
        }
    }
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void sendPath(std::string);  //path_filename_topic
    void canQuit();
    void init();
    void sendJoyCommand(int);
    void sendVehicleMode(int);
    void sendPathExist(bool);

public slots:
    void slot_initAllParams();
    void initAllLabels();
    void labelsStartCountDown();
    void labelsStartRunning(int);
    void labelsMissionPause();

    // slots for button states in buttons.h
    void slot_initAllBtns();
    void slot_disableAllDestBtns();
    void slot_enableAllDestBtns();
    void resetAllDestButtons();
    void slot_readyToStart();
    void setF10AButtonOn(bool);
    void setF10NButtonOn(bool);
    void setF10XButtonOn(bool);
    void setStartButtonOn(bool);
    void setStopButtonOn(bool);

    // some other slots
    void slot_sendPathName(int);
    void slot_displayRemainingTime();

    // slots for ROS states
    void slot_displaySpeed(double);
    void slot_displayAngle(double);
    void slot_displayBrake(int);
    void slot_displayObstacle(double);
    void slot_displayDAC(double);
    void slot_displayMode(int);

private Q_SLOTS:
    void on_m_button_Start_clicked();
    void on_m_button_Stop_clicked();
    void on_m_button_10X_clicked();
    void on_m_button_10N_clicked();
    void on_m_button_10A_clicked();

private:
    Ui::MainWindow *ui;
    TalkerNode *m_talkerNode;
    ListenerNode *m_listenerNode;
    QThread *m_talkerThread;
    QThread *m_listenerThread;
    QThread *m_markerThread;
    //QLabel *currentTimeLabel;
    //VizlibWidget *m_vizlibWidget;
    //MarkerNode *m_markerNode;

    Setting setting;
    QMode qmode;
    QSpeed qspeed;
    QAngle qangle;
    QBrake qbrake;
    QPath qpath;
    QMusic qmusic;
    QCountDown qcountDown;
    QObstacle qobstacle;
    QAcceleration qacceleration;
    Icons *icons = new Icons;
};

#endif // MAINWINDOW_H
