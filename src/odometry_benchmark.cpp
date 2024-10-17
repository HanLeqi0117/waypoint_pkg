#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtGui/QPainter>
#include <QtGui/QLinearGradient>
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
        mode = this->declare_parameter<std::string>("mode", "comparision");
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
    std::string mode;
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
                        waypoint.covariance = data["covariance"].as<std::array<double, 36>>();
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

// 共分散に基づいて色を計算する関数
QColor covarianceToColor(const std::array<double, 36>& covariance) {
    // x の共分散は covariance[0], y の共分散は covariance[7] に対応
    double x_covariance = covariance[0];
    double y_covariance = covariance[7];

    // x と y の共分散の平均値を用いて色を決定
    double avg_covariance = (x_covariance + y_covariance) / 2.0;

    // 共分散の平均値を 0 ~ 100 の範囲に制限
    avg_covariance = std::min(100.0, avg_covariance);

    // 0 ~ 100 を 0.0 ~ 1.0 にスケーリング
    double normalized_covariance = avg_covariance / 100.0;

    // 共分散が大きいほど赤くするための計算 (R成分)
    int R = static_cast<int>(normalized_covariance * 255.0);  // 0.0 ~ 1.0 を 0 ~ 255 にマッピング
    int G = 0;
    int B = 255 - R;  // 青から赤へのグラデーション

    // 0 のとき青、100 のとき赤、その中間のグラデーションを返す
    return QColor(R, G, B);
}

class CovarianceChartView : public QChartView {
public:
    CovarianceChartView(QChart *chart, QWidget *parent = nullptr)
        : QChartView(chart, parent) {}

protected:
    void paintEvent(QPaintEvent *event) override {
        // 既存のチャートを描画
        QChartView::paintEvent(event);

        // 右上にグラデーションバーを描画
        QPainter painter(this->viewport());

        // グラデーションの矩形範囲を設定 (右上に表示)
        QRect gradientRect(width() - 50, 30, 30, 120);

        // 青から赤のグラデーション
        QLinearGradient gradient(gradientRect.topLeft(), gradientRect.bottomLeft());
        gradient.setColorAt(1.0, QColor(0, 0, 255));  // 0 の色: 青
        gradient.setColorAt(0.0, QColor(255, 0, 0));  // 100 の色: 赤

        // グラデーション描画設定
        painter.setBrush(QBrush(gradient));
        painter.setPen(Qt::NoPen);
        painter.drawRect(gradientRect);

        // 0 と 100 のラベル表示
        painter.setPen(QPen(Qt::black));
        painter.drawText(gradientRect.left() - 30, gradientRect.bottom(), "    0");
        painter.drawText(gradientRect.left() - 30, gradientRect.top(), "100");
    }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<OdometryPlot> node = std::make_shared<OdometryPlot>();
    QApplication app(argc, argv);

    if (node->mode == "comparision") {
        QMainWindow comparision_window;

        QChart *odometry_chart = new QChart();
        odometry_chart->setTitle("Odometry Comparison");
        
        for (auto &&data_name : node->data_names) {
            QLineSeries *odometry_serials = new QLineSeries();
            for (auto &&waypoint : node->odometry_datas[data_name]) {
                odometry_serials->append(waypoint.pos_x, waypoint.pos_y);
            }
            QPen pen(QColor(QRandomGenerator::global()->bounded(0, 256), QRandomGenerator::global()->bounded(0, 256), QRandomGenerator::global()->bounded(0, 256)));
            pen.setWidth(3);
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
        comparision_window.setCentralWidget(chartView);
        comparision_window.resize(1000, 1000);
        comparision_window.show();
        return app.exec();

    } else if (node->mode == "covariance") {
        for (auto &&data_name : node->data_names) {
            QMainWindow *covariance_window = new QMainWindow();
            QtCharts::QChart* chart = new QtCharts::QChart();

            for (auto &&waypoint : node->odometry_datas[data_name]) {
                QtCharts::QLineSeries* lineSeries = new QtCharts::QLineSeries();
                QtCharts::QScatterSeries* scatterSeries = new QtCharts::QScatterSeries();
                QColor pointColor = covarianceToColor(waypoint.covariance);

                lineSeries->append(waypoint.pos_x, waypoint.pos_y);
                scatterSeries->append(waypoint.pos_x, waypoint.pos_y);
                scatterSeries->setColor(pointColor);
                scatterSeries->setMarkerSize(10);
                chart->addSeries(lineSeries);
                chart->addSeries(scatterSeries); 
            }

            chart->createDefaultAxes();
            chart->legend()->hide();
            chart->axes(Qt::Horizontal).first()->setTitleText("X Position");
            chart->axes(Qt::Vertical).first()->setTitleText("Y Position");

            qreal rangeX = node->x_max - node->x_min;
            qreal rangeY = node->y_max - node->y_min;
            qreal largestRange = std::max(rangeX, rangeY);

            chart->axes(Qt::Horizontal).first()->setRange(node->x_min - largestRange / 20, node->x_min + largestRange + largestRange / 20);
            chart->axes(Qt::Vertical).first()->setRange(node->y_min - largestRange / 20, node->y_min + largestRange + largestRange / 20);

            chart->setTitle(data_name.c_str());
            CovarianceChartView* chartView = new CovarianceChartView(chart);
            chartView->setRenderHint(QPainter::Antialiasing);
            
            covariance_window->setCentralWidget(chartView);
            covariance_window->resize(1000, 1000);
            covariance_window->show();
        }
        return app.exec();

    } else {
        rclcpp::shutdown();
        return 0;
    }


}