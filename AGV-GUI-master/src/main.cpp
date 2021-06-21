#include "includes/mainwindow.h"
#include <QApplication>
#include "ros/ros.h"
#include <string>
#include <stdlib.h>
int main(int argc, char *argv[]){
    //system("gnome-terminal -x bash -c 'source ~/my_ws/devel/setup.bash;roslaunch my_package file.launch'");
    ros::init(argc, argv, "UI"/*, ros::init_options::AnonymousName*/);
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    return a.exec();
}
