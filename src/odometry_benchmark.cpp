#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QApplication>
#include <QMainWindow>
#include <QValueAxis>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/msg/odometry.hpp>

QT_CHARTS_USE_NAMESPACE

class OdometryPlot : public rclcpp::Node {
public:

    OdometryPlot() : Node("odometry_plot"){

    }

private:

    rclcpp::Subscription<nav2_msags::msg::Odometry>::SharedPtr 
    
}