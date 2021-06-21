# AGV-UI

This is a GUI for Micron Fab 10 autonomous buggy project.

Code Authors: 
> Congyi LIU, [github page](https://github.com/CYLau001), E-mail: e0012640@u.nus.edu

> Shuo SUN, [github page](https://github.com/SS47816), E-mail: e0134075@u.nus.edu

---
## Prerequisites
1. Qt Creator
2. ROS Kinetic + Rivz
3. g++ 5.4.0

---
## Installation

### Install Qt Creator
1. `sudo chmod 777 qt-opensource-linux-x64-VERSION.run`
2. `./qt-opensource-linux-x64-*.run`
3. ID: kelvinkang@micron.com Password: Micron123
4. select Qt5.13.0

### Add Qt Creator shortcut
1. `cd ~/.local/share/applications`
2. `gedit DigiaQt-qtcreator-enterprise.desktop`
3. change `Exec=/home` to "`Exec= bash -i -c /home`

### Set up the headers
1. `cd /opt/ros/kinetic/include/rviz/ogre_helpers`
2. `sudo chmod 777 qt_ogre_render_window.h`
3. `gedit qt_ogre_render_window.h`
4. line 36 & 37: change to `#include <OGRE/OgreColourValue.h>` and `#include <OGRE/OgreRenderTargetListener.h>`
5. `cd ..`
6. `sudo chmod 777 render_panel.h`
7. `gedit render_panel.h`
8. line 36: change to `#include <OGRE/OgreSceneManager.h>`

---
