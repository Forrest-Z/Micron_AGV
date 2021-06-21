#include <QString>
#include <QDebug>

class Setting {

private:
    const int buggy_id = 2; // buggy no. 1/2/3 or 0 for development
    const bool debug = false;
    const bool allow_changing_dest = true; // allow changing destination when in autonomous mode
    const bool allow_changing_dest_while_moving = true; // allow changing destination when vehicle in motion
    const bool dest_button_10A = true;
    const bool dest_button_10N = true;
    const bool dest_button_10X = true;

    class Path_package {
    private:
        std::string path_to_10A;
        std::string path_to_10N;
        std::string path_to_10X;


    public:
        Path_package(std::string path_1, std::string path_2, std::string path_3) {
            path_to_10A = path_1;
            path_to_10N = path_2;
            path_to_10X = path_3;
        }
        std::string getPath(int button_id) {
            switch (button_id) {
            case 3 :
                qDebug() << "10A";
                return path_to_10A;
            case 4 :
                qDebug() << "10N";
                return path_to_10N;
            case 5 :
                qDebug() << "10X";
                return path_to_10X;
            default:
                qDebug() << "Not a valid button_id";
            }
        }
    };

    Path_package *buggy0_paths = new Path_package("gg.path", "10xtotunnel.path", "tunnelloop.path");
    Path_package *buggy1_paths = new Path_package("dryrun.path", "shunda.path", "dryrun2.path");
    Path_package *buggy2_paths = new Path_package("homerun.path", "tunnelto10n.path", "homerun2.path");
    Path_package *buggy3_paths = new Path_package("wetrun.path", "10xtotunnel.path", "wetrun2.path");


public:
    int getBuggyID() const {
        return buggy_id;
    }
    bool debugging() const {
        return debug;
    }
    bool allowChangingDest() const {
        return allow_changing_dest;
    }
    bool allowChangingDestWhileMoving() const {
        return allow_changing_dest_while_moving;
    }
    bool button10AIsAvailable() const {
        return dest_button_10A;
    }
    bool button10NIsAvailable() const {
        return dest_button_10N;
    }
    bool button10XIsAvailable() const {
        return dest_button_10X;
    }
    Path_package getPathPackage() const {
        switch (buggy_id) {
        case 0 :
            qDebug() << "buggy_0";
            return *buggy0_paths;
        case 1 :
            qDebug() << "buggy_1";
            return *buggy1_paths;
        case 2 :
            qDebug() << "buggy_2";
            return *buggy2_paths;
        case 3 :
            qDebug() << "buggy_3";
            return *buggy3_paths;
        }
    }
};
