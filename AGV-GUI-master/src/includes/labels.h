#include "mainwindow.h"
#include "ui_mainwindow.h"

void MainWindow::initAllLabels() {
    ui->label_selectDest->show();
    ui->label_countDown->hide();
    ui->label_starting->hide();
    ui->label_heading->hide();
    ui->label_obstacle->hide();
}

void MainWindow::labelsStartCountDown() {
    ui->label_selectDest->hide();
    ui->label_starting->setText("<html><head/><body><p align='center'>"
                                "<span style='font-size:24pt; font-weight:600;'>Starting in</span></p></body></html>");
    ui->label_starting->show();
    ui->label_heading->hide();
}

void MainWindow::labelsStartRunning(int btn_name) {
    ui->label_selectDest->hide();
    ui->label_countDown->hide();
    ui->label_starting->hide();
    ui->label_obstacle->hide();
    QString qs;
    switch (btn_name) {
        case 3 : //10A
            qs = "<html><head/><body><p align='center'><span style=' font-size:24pt; font-weight:600;'>"
                 "Heading to </span><span style=' font-size:36pt; font-weight:600; color:#3465a4;'>F10A</span></p></body></html>";
            break;
        case 4 : //10N
            qs = "<html><head/><body><p align='center'><span style=' font-size:24pt; font-weight:600;'>"
                 "Heading to </span><span style=' font-size:36pt; font-weight:600; color:#3465a4;'>F10N</span></p></body></html>";
            break;
        case 5 : //10X
            qs = "<html><head/><body><p align='center'><span style=' font-size:24pt; font-weight:600;'>"
                 "Heading to </span><span style=' font-size:36pt; font-weight:600; color:#3465a4;'>F10X</span></p></body></html>";
            break;
    }
    ui->label_heading->setText(qs);
    ui->label_heading->show();
}

void MainWindow::labelsMissionPause() {
    ui->label_selectDest->show();
    ui->label_countDown->hide();
    ui->label_starting->hide();
    ui->label_heading->hide();
    ui->label_obstacle->hide();
}
