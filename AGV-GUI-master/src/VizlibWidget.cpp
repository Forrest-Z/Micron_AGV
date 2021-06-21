#include "includes/VizlibWidget.h"
#include <QLayout>
#include <iostream>
#include <QDebug>
VizlibWidget::VizlibWidget(QWidget *parent) :
    QWidget(parent){
}

void VizlibWidget::rviz_display(){
    render_panel_ = new rviz::RenderPanel;
    QGridLayout *layout = new QGridLayout(this);
    layout->addWidget(render_panel_,0 ,0);
    this->setLayout(layout);

    manager_ = new rviz::VisualizationManager( render_panel_ );
    render_panel_->initialize( manager_->getSceneManager(), manager_ );
    manager_->initialize();
    manager_->startUpdate();
    //manager_->removeAllDisplays();

    rviz::Display *map = manager_->createDisplay( "rviz/Map", "adjustable map", true );
    map->subProp( "Topic" )->setValue( "/map" );
    // sleep(1);

    /* Source of error */
    rviz::Display *path = manager_->createDisplay( "rviz/Path", "fixed path", true );
//    path->subProp( "Topic" )->setValue( "/current_path" ); // where the error came from
    // sleep(1);

    rviz::Display *marker = manager_->createDisplay( "rviz/Marker", "marker", true );
    marker->setTopic("/vis_marker", "visualization_msgs::Marker");
    // sleep(1);

    rviz::Display *amcl = manager_->createDisplay( "rviz/PoseArray", "marker", true );
    amcl->subProp( "Topic" )->setValue( "/particlecloud" );
    // sleep(1);

    manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue("500");
    // sleep(1);

    manager_->getViewManager()->getCurrent()->lookAt(100,100,100);
    manager_->getViewManager()->setCurrentViewControllerType("rviz/TopDownOrtho");
    // sleep(1);

//    rviz::Display *robot = manager_->createDisplay( "rviz/RobotModel", "adjustable robot", true );
//    robot->subProp( "Robot Description" )->setValue( "robot_description" );

//    rviz::Display *laser = manager_->createDisplay( "rviz/LaserScan", "adjustable scan", true );
//    laser->subProp( "Topic" )->setValue( "/scan" );
//    laser->subProp( "Size (m)" )->setValue( "0.1" );

}

// not working
void VizlibWidget::rviz_updateView(float x, float y){
    manager_->getViewManager()->getCurrent()->lookAt(x, y, 100);

}

VizlibWidget::~VizlibWidget(){
    delete manager_;
}
