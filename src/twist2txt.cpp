#include <waypoint_pkg/utilis.hpp>

class Twist2Txt : public rclcpp::Node
{
    public:

        Twist2Txt() : Node("twist2txt")
        {
            // ROSパラメータの宣言
            this->declare_parameter("way_txt_file", "/home/ubuntu/ubuntu/waypoint.txt");
            this->declare_parameter("point_distance", 4.0);
            this->declare_parameter("deg_thresh", 15.0);
            this->declare_parameter("deg_chord", 1.0);

            // ROSパラメータの取得
            this->get_parameter("way_txt_file", txt_file_param_);
            this->get_parameter("point_distance", point_distance_param_);
            this->get_parameter("deg_thresh", deg_thresh_param_);
            this->get_parameter("deg_chord", deg_chord_param_);

            this->tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            this->tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_, this);

            ofs_.open(txt_file_param_.as_string().c_str(), std::ios::out);
            // ファイルが開けない場合
            if (ofs_.fail())
            {
                RCLCPP_ERROR(this->get_logger(), "The text file to save Waypoints is unavailable.");
                ofs_.close();
                return; 
            } 

            // パブリッシャーの定義
            // テンプレート：送信するデータのタイプは、<visualization_msgs::msg::Marker>
            // 引数1：送信先のトピック
            // 引数2：データ列のサイズ
            this->marker_pub_ = this->create_publisher<Marker>("/waypoint_marker", 100);

            // WallTimerの定義
            // 
            this->timer_ = this->create_wall_timer(10ms, std::bind(&Twist2Txt::marker_pub_callback_, this));
        }

        ~Twist2Txt()
        {

            for(size_t i = 1; i < waypoints_.size(); i++)
            {
                ofs_ << waypoints_[i].pose.position.x << "," 
                    << waypoints_[i].pose.position.y << "," 
                    << waypoints_[i].pose.position.z << ","
                    << waypoints_[i].pose.orientation.x << "," 
                    << waypoints_[i].pose.orientation.y << "," 
                    << waypoints_[i].pose.orientation.z << ","
                    << waypoints_[i].pose.orientation.w << std::endl;
            }

            // ファイルを閉じる
            ofs_.close();
        }

    private:

        // パラメータのインスタンス
        rclcpp::Parameter txt_file_param_;
        rclcpp::Parameter point_distance_param_;
        rclcpp::Parameter deg_thresh_param_;
        rclcpp::Parameter deg_chord_param_;

        // 「ofs_」は、std:ofstreamのインスタンス。ofstreamは、Output File Streamである
        std::ofstream ofs_;    

        // Waypointのインデクスを0で初期化する
        int way_count_ = 0;
        // 「waypoint」は、要素がPoint構造体であるベクトルコンテナのインスタンスを宣言する
        std::vector<Waypoint> waypoints_;

        // ユニークポインタとして定義する
        // 引数1：ROS時間
        // 機能：TF2システムから受信した情報を格納する        
        std::unique_ptr<tf2_ros::Buffer> tf2_buffer_;

        //　共有ポインタとして定義する
        // 引数1：バッファ
        // 引数2：Nodeオブジェクトのポインタ
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        // 「geometry_msgs::msg::StampedTransform」は、座標間の空間関係を表すインスタンス
        TransformStamped transform_;

        // 「rclcpp::Publisher」は、送信するAPIのクラス
        // テンプレート：<visualization_msgs::msg::Marker>
        // 「marker_pub_」は、パブリッシャーのインスタンス
        rclcpp::Publisher<Marker>::SharedPtr marker_pub_;

        // Callback関数に使うTimer
        rclcpp::TimerBase::SharedPtr timer_;

        // Markerを送信するCallback関数
        void marker_pub_callback_()
        {
            // TF2システムから地図とロボットの座標変換関係を取得する
            try
            {
                RCLCPP_DEBUG(get_logger(), "Get Tranfromation between /map and /base_link ... \n");
                // 「tf2_buffer_」のメンバ関数「lookupTransform」を呼び出す
                // 戻り値：二つの座標IDの座標変換結果、すなわち二つの座標の空間関係
				// 引数1：変換先の座標ID
				// 引数2：変換元の座標ID
                // 引数3：望ましい変換先座標の時間スタンプと変換元座標の時間スタンプの時間差。
				// 座標ID「/map」と座標ID「/base_link」の間の座標変換関係情報を「transform_」に保存する
				// ここで、「transform_」の座標IDは「/map」であり、子座標IDは「/base_link」(ロボットの基盤)である
                transform_ = tf2_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            }
            // TF2システムの例外処理
            catch(const tf2::TransformException &ex)
            {
                RCLCPP_WARN(get_logger(), "Exception when listening Transformation of Frames: %s", ex.what());
                rclcpp::sleep_for(1s);
                return;
            }
            
            // Point構造体のインスタンス「wayp_tmp」を新規する
            Waypoint wayp_tmp;
            // 三次元線形座標系のデータを「wayp_tmp」に保存する
            wayp_tmp.pose.position.x = transform_.transform.translation.x;
            wayp_tmp.pose.position.y = transform_.transform.translation.y;
            wayp_tmp.pose.position.z = transform_.transform.translation.z;
            // 四元数座標系のデータを「wayp_tmp」に保存する
            wayp_tmp.pose.orientation = transform_.transform.rotation;

            if(way_count_ == 0)
            {
                // 関数「getRPY()」の引数は、参照渡しを使用した。
                // 参照渡しとは、URL：https://wa3.i-3-i.info/word16070.html
                // geometory_quat -> tf2_quat
                // tf2_quat -> tf2_mat
                // tf2_mat -> RPY
                tf2::Quaternion tf2_quat;
                tf2::fromMsg(wayp_tmp.pose.orientation, tf2_quat);
                tf2::Matrix3x3(tf2_quat).getRPY(
                    wayp_tmp.raw.data,
                    wayp_tmp.pitch.data,
                    wayp_tmp.yaw.data            
                );
                
                // コンテナ「waypoint」の「push_back()」関数を用いて、「wayp_tmp」を格納する
                waypoints_.push_back(wayp_tmp);
                // Waypointのインデクスを加算処理する
                way_count_++;
                // 今回の繰り返し処理をスキップする
                return;
            }

            // 2つのWaypoint間の距離差分を計算する
            double diff_way = hypot(
                wayp_tmp.pose.position.x
                 - waypoints_[way_count_ - 1].pose.position.x,
                wayp_tmp.pose.position.y
                 - waypoints_[way_count_ - 1].pose.position.y
            );

            // geometory_quat -> RPY
            tf2::Quaternion tf2_quat;
            tf2::fromMsg(wayp_tmp.pose.orientation, tf2_quat);
			tf2::Matrix3x3(tf2_quat).getRPY(
				wayp_tmp.raw.data,
				wayp_tmp.pitch.data,
				wayp_tmp.yaw.data            
			);
                            

            // Yaw角の差分の絶対値を計算する
            double diff_yaw = std::abs(
                wayp_tmp.yaw.data            
                 - waypoints_[way_count_ - 1].yaw.data
            );

            // 処理プロセスのログ
            std::stringstream ss;
            ss << "No. " << way_count_ << " " 
                << " yaw_base " << waypoints_[way_count_ - 1].yaw.data * 180.0 / M_PI 
                << " yaw_now " << wayp_tmp.yaw.data * 180.0 / M_PI 
                << " diff_yaw " << diff_yaw * 180.0 / M_PI 
                << " diff_way " << diff_way << std::endl;
            RCLCPP_DEBUG(this->get_logger(), "%s", ss.str().c_str());
            
            // Yaw角の差分は円弧の弧度差分閾値より大きい場合かつ距離差分は円弧の弦差分閾値より大きい場合
            // また、距離差分は直線距離差分より大きい場合
            if((
                diff_yaw * 180.0 / M_PI > deg_thresh_param_.as_double()
                && diff_way > deg_chord_param_.as_double()
            )
                || diff_way > point_distance_param_.as_double())
            {	
                // コンテナ「waypoint」に「wayp_tmp」を入れる
                waypoints_.push_back(wayp_tmp);
                // Waypointのインデクスを加算処理する
                way_count_++;
            }

            // 「Marker」は、ROSメッセージインタフェイスのクラスである
            Marker marker_points;
            // 座標ID
            marker_points.header.frame_id = "map";
            // 時間スタンプ
            marker_points.header.stamp = this->get_clock().get()->now();
            // ネーム
            marker_points.ns = "marker_points";
            // マーカーの色
            marker_points.color.b = 1.0;
            marker_points.color.a = 1.0;
            // マーカーのスケール
            marker_points.scale.x = 0.3;
            marker_points.scale.y = 0.3;
            marker_points.scale.z = 0.3;
            // マーカーの動作
            // 「ADD」は、「Marker」に仕組まれた列挙数の一つであり、マーカーを追加することを意味する
            marker_points.action = Marker::ADD;
            // マーカーの種類
            // 「marker_points」は、「Marker」に仕組まれた列挙数の一つであり、マーカーの種類は点に指定する
            marker_points.type = Marker::POINTS;
            // マーカーの姿勢の四元数の実数部を1.0に指定する
            // 幾何学的な意味は回転しないこと(恒等変換)を指す
            marker_points.pose.orientation.w = 1.0;

            // 送信するマーカーの座標をwaypointから取得する
            // 処理が冗長である。ROS2で改善すべき
            for(size_t i = 1; i < waypoints_.size(); i++)
            {
                // ROSメッセージインスタンスのネームスペース「geometry_msgs」にあるクラス「Point」である
                Point p;
                // ROSのインタフェイスの「p」に構造体の「waypoints_」の座標系データを入れる
                p.x = waypoints_[i].pose.position.x;
                p.y = waypoints_[i].pose.position.y;
                p.z = 0.25;
                // マーカーのインスタンス「points」のメンバ「points」ベクトルに「p」を入れる
                marker_points.points.push_back(p);
            }
            // マーカーを送信する
            this->marker_pub_->publish(marker_points);
            
        }
        
};

int main(int argc, char *argv[])
{
    // ROSの初期化
	// 引数1：端末の引数の数
	// 引数2：端末の引数が入れている配列、型はchar
    rclcpp::init(argc, argv);

    // 処理にスピンさせる
    rclcpp::spin(std::make_shared<Twist2Txt>());

    rclcpp::shutdown();

}