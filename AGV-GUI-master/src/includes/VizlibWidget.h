#ifndef VIZLIBTEST_H
#define VIZLIBTEST_H

#include <QWidget>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/view_controller.h>

class VizlibWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VizlibWidget(QWidget *parent = nullptr);
    ~VizlibWidget();
public slots:
    void rviz_display();
    void rviz_updateView(float, float);
private:
    rviz::VisualizationManager *manager_;
    rviz::RenderPanel *render_panel_;

};

#endif // VIZLIBTEST_H
