#include "mainwindow.h"
#include "ui_mainwindow.h"

void MainWindow::setStartButtonOn(bool on) {
    ui->m_button_Start->setChecked(on);
    ui->m_button_Start->setIcon(icons->getIcon(1, on));
    ui->m_button_Start->setIconSize(icons->getSize(1));
}
void MainWindow::setStopButtonOn(bool on) {
    ui->m_button_Stop->setChecked(on);
    ui->m_button_Stop->setIcon(icons->getIcon(2, on));
    ui->m_button_Stop->setIconSize(icons->getSize(2));
}
void MainWindow::setF10AButtonOn(bool on) {
    ui->m_button_10A->setChecked(on);
    ui->m_button_10A->setIcon(icons->getIcon(3, on));
    ui->m_button_10A->setIconSize(icons->getSize(3));
}
void MainWindow::setF10NButtonOn(bool on) {
    ui->m_button_10N->setChecked(on);
    ui->m_button_10N->setIcon(icons->getIcon(4, on));
    ui->m_button_10N->setIconSize(icons->getSize(4));
}
void MainWindow::setF10XButtonOn(bool on) {
    ui->m_button_10X->setChecked(on);
    ui->m_button_10X->setIcon(icons->getIcon(5, on));
    ui->m_button_10X->setIconSize(icons->getSize(5));
}
void MainWindow::resetAllDestButtons() {
    setF10AButtonOn(false);
    setF10NButtonOn(false);
    setF10XButtonOn(false);
}

/* enable all buttons but the start button */
void MainWindow::slot_initAllBtns() {
    // control the on/off of the animation
    ui->m_button_Start->setCheckable(true);
    ui->m_button_Stop->setCheckable(true); // ???
    ui->m_button_10A->setCheckable(true);
    ui->m_button_10N->setCheckable(true);
    ui->m_button_10X->setCheckable(true);
    // controll the up/down of the button
    ui->m_button_Start->setChecked(false);
    ui->m_button_Stop->setChecked(false); // ???
    ui->m_button_10A->setChecked(false);
    ui->m_button_10N->setChecked(false);
    ui->m_button_10X->setChecked(false);
    // enable/disable the function of a button
    ui->m_button_Start->setDisabled(true);
    setStartButtonOn(false);
    setStopButtonOn(false);
    setF10AButtonOn(false);
    setF10XButtonOn(false);
    setF10NButtonOn(false);

    qDebug() << "all buttons initialised";
}

/* enable all destination buttons */
void MainWindow::slot_enableAllDestBtns() {
    ui->m_button_10A->setEnabled(true);
    ui->m_button_10N->setEnabled(true);
    ui->m_button_10X->setEnabled(true);

    qDebug() << "all buttons enabled";
}

/* disable all buttons but the stop button */
void MainWindow::slot_disableAllDestBtns(){
    ui->m_button_10A->setDisabled(true);
    ui->m_button_10N->setDisabled(true);
    ui->m_button_10X->setDisabled(true);

    qDebug() << "all buttons disabled";
}

void MainWindow::on_m_button_10A_clicked() { // button_id 3
    resetAllDestButtons();
    bool available = setting.button10AIsAvailable();
    if(available) {
        setF10AButtonOn(true);
        slot_sendPathName(3);
    }
    qpath.setPath(available);
    slot_readyToStart();
}
void MainWindow::on_m_button_10N_clicked() { // button_id 4
    resetAllDestButtons();
    bool available = setting.button10XIsAvailable();
    if(available) {
        setF10NButtonOn(true);
        slot_sendPathName(4);
    }
    qpath.setPath(available);
    slot_readyToStart();
}
void MainWindow::on_m_button_10X_clicked() { // button_id 5
    resetAllDestButtons();
    bool available = setting.button10XIsAvailable();
    if(available) {
        setF10XButtonOn(true);
        slot_sendPathName(5);
    }
    qpath.setPath(available);
    slot_readyToStart();
}

void MainWindow::slot_sendPathName(int button_id) {
    //std::string path = qpath.getPath(setting.getBuggyID(), button_id);
    std::string path = setting.getPathPackage().getPath(button_id);
    emit sendPath(path);
    const std::string& debug = path;
    qDebug() << debug.data() << debug.c_str();
    qmode.setDest(button_id);
    qDebug() << button_id << "******************";
}

// when a destination has been checked
void MainWindow::slot_readyToStart() {
    //bool allowStartWhileMoving = true;

    qDebug() << "checking if we are ready to start";
    double velocity = qspeed.speed();
    bool path_exist = qpath.path_exist();
    int vehicle_mode = qmode.getMode(); // 0 for brake/parking mode, 1 for autonomous mode
    if(path_exist){
        if(vehicle_mode == 0){ //vehicle at parking/brake mode
            if(velocity <= 0.2 || setting.allowChangingDestWhileMoving()){ // all 3 criteria are met
                ui->m_button_Start->setEnabled(true);
                ui->m_button_Start->setChecked(false);
                qDebug() << "You are all set, let's move out";
            }
            else {
                qDebug() << "vehicle still moving";
            }
        }
        else {
            qDebug() << "vehicle not in parking mode";
        }
    }
    else {
        qDebug() << qpath.path_exist();
        qDebug() << "Please select a path first";
    }
    qDebug() << "checking done";
}

void MainWindow::on_m_button_Start_clicked() {
    // button & label animations
    slot_disableAllDestBtns();
    ui->m_button_Start->setDisabled(true);
    setStartButtonOn(true);
    setStopButtonOn(false);
    labelsStartCountDown();

    // start count down
    qcountDown.startCountDown();
    connect(qcountDown.getTimer(), SIGNAL(timeout()), this, SLOT(slot_displayRemainingTime()) );
    // start the audio player
    qmusic.startPlaying();

    qDebug() << qbrake.brake_state();
    qDebug() << "Start being pressed";
}

void MainWindow::slot_displayRemainingTime() {
    int t = qcountDown.getRemainingTime();
    if(t >= 0) {    // During countdown
        ui->label_countDown->show();
        ui->label_countDown->setNum(t);
        qcountDown.setRemainingTime(t-1);
        qDebug() << "######### timer: " << t;
    }
    else if (t == -1) {     // Disconnect once countdown finished
        // send start cmd
        emit sendJoyCommand(1);
        qDebug() << "Joy Cmd Sent **************";
        qmode.setMode(1); // set mode to autonomous mode
        ui->label_countDown->hide();
        disconnect(qcountDown.getTimer(), SIGNAL(timeout()), this, SLOT(slot_displayRemainingTime()) );
        // connect obstacle signals to qt
        connect(m_listenerNode, SIGNAL(obstacleDist(double)), this, SLOT(slot_displayObstacle(double)));
        // end count down & recover other labels
        int destination = qmode.getDest();
        labelsStartRunning(destination);
    }
}

void MainWindow::on_m_button_Stop_clicked() {
    emit sendJoyCommand(0); // send stop cmd to ROS
    qmode.setMode(0); // set mode to brake mode
    setStartButtonOn(false);
    setStopButtonOn(true);
    slot_readyToStart();

    // Stop the countdown
    disconnect(qcountDown.getTimer(), SIGNAL(timeout()), this, SLOT(slot_displayRemainingTime()) );
    // connect obstacle signals to qt
    disconnect(m_listenerNode, SIGNAL(obstacleDist(double)), this, SLOT(slot_displayObstacle(double)));
    labelsMissionPause();
    ui->label_obstacle->hide();

    // Stop the mp3 player
    qmusic.pausePlaying();

    //bool allowChangingDest = true; // allow changing destination while moving
    if(setting.allowChangingDest()) slot_enableAllDestBtns();

    qDebug() << qbrake.brake_state();
    qDebug() << "Stop being pressed";
}
