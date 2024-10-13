#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QApplication>
#include <QMainWindow>
#include <QValueAxis>
#include <QRandomGenerator>
#include <rclcpp/rclcpp.hpp>
#include <waypoint_pkg/utilis.hpp>
#include <yaml-cpp/yaml.h>


QT_CHARTS_USE_NAMESPACE

class OdometryPlot : public rclcpp::Node{
public:

    OdometryPlot() : Node("odometry_plot"){
        waypoint_file_paths = this->declare_parameter<std::vector<std::string>>("waypoint_file_paths", {""});
        if (waypoint_file_paths.size() == 0) {
            RCLCPP_WARN(get_logger(), "No file to read");
            return;
        }

        if (!read_files()) {
            RCLCPP_ERROR(get_logger(), "Error when reading files.");
            return;
        }
        
    }

    std::vector<std::string> waypoint_file_paths;
    std::map<std::string, std::vector<Waypoint::Waypoint>> odometry_datas;
    std::vector<std::string> data_names;
    double x_max, x_min, y_max, y_min;

private:

    bool read_files() {

        for (auto &&waypoint_file_path : waypoint_file_paths) {
            auto slash_position = waypoint_file_path.find_last_of('/');
            std::string file_name = waypoint_file_path.substr(slash_position + 1);
            file_name = file_name.substr(0, file_name.size() - 5);
            data_names.push_back(file_name);

            try {
                auto root = YAML::LoadAllFromFile(waypoint_file_path);
                std::vector<Waypoint::Waypoint> waypoints_vector = {};

                for (auto &&doc : root) {
                    for (YAML::const_iterator it = doc.begin(); it != doc.end(); ++it) {
                        Waypoint::Waypoint waypoint;
                        const YAML::Node& data = it->second;

                        waypoint.pos_x = data["position_x"].as<double>();
                        waypoint.pos_y = data["position_y"].as<double>();
                        waypoint.pos_z = 0.2;
                        waypoint.quat_x = data["quaternion_x"].as<double>();
                        waypoint.quat_y = data["quaternion_y"].as<double>();
                        waypoint.quat_z = data["quaternion_z"].as<double>();
                        waypoint.quat_w = data["quaternion_w"].as<double>();
                        waypoint.roll = data["roll"].as<double>();
                        waypoint.pitch = data["pitch"].as<double>();
                        waypoint.yaw = data["yaw"].as<double>();
                        waypoint.longitude = data["longitude"].as<double>();
                        waypoint.latitude = data["latitude"].as<double>();
                        waypoint.mode = static_cast<Waypoint::WaypointMode>(data["mode"].as<int>());

                        waypoints_vector.push_back(waypoint);

                        if (x_max <= waypoint.pos_x) {
                            x_max = waypoint.pos_x;
                        }
                        if (y_max <= waypoint.pos_y) {
                            y_max = waypoint.pos_y;
                        }
                        if (y_min >= waypoint.pos_y) {
                            y_min = waypoint.pos_y;
                        }
                        if (x_min >= waypoint.pos_x) {
                            x_min = waypoint.pos_x;
                        }

                    }
                }

                odometry_datas[file_name] = waypoints_vector;
            } catch (const YAML::Exception& e) {
                std::stringstream ss;
                ss << "YAML Exception: " << e.what();
                RCLCPP_ERROR(get_logger(), ss.str().c_str());
                return false;
            }
            
        }

        return true;
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OdometryPlot> node = std::make_shared<OdometryPlot>();

    QApplication a(argc, argv);
    QMainWindow window;

    QChart *odometry_chart = new QChart();
    odometry_chart->setTitle("Odometry Comparison");
    
    for (auto &&data_name : node->data_names) {
        QLineSeries *odometry_serials = new QLineSeries();
        for (auto &&waypoint : node->odometry_datas[data_name]) {
            odometry_serials->append(waypoint.pos_x, waypoint.pos_y);
        }
        QPen pen(QColor(QRandomGenerator::global()->bounded(0, 256), QRandomGenerator::global()->bounded(0, 256), QRandomGenerator::global()->bounded(0, 256)));
        pen.setWidth(2);
        odometry_serials->setPen(pen);
        odometry_serials->setName(data_name.c_str());
        odometry_chart->addSeries(odometry_serials);
    }

    odometry_chart->createDefaultAxes();

    // Customize axis labels
    odometry_chart->axes(Qt::Horizontal).first()->setTitleText("X Position");
    odometry_chart->axes(Qt::Vertical).first()->setTitleText("Y Position");

    qreal rangeX = node->x_max - node->x_min;
    qreal rangeY = node->y_max - node->y_min;
    qreal largestRange = std::max(rangeX, rangeY);

    // Set both axes to have the same range to maintain the 1:1 aspect ratio
    odometry_chart->axes(Qt::Horizontal).first()->setRange(node->x_min - largestRange / 20, node->x_min + largestRange + largestRange / 20);
    odometry_chart->axes(Qt::Vertical).first()->setRange(node->y_min - largestRange / 20, node->y_min + largestRange + largestRange / 20);

    // Create a odometry_chart view
    QChartView *chartView = new QChartView(odometry_chart);
    chartView->setRenderHint(QPainter::Antialiasing);

    // Set the odometry_chart view as the central widget
    window.setCentralWidget(chartView);
    window.resize(1000, 1000);
    window.show();

    return a.exec();
}