#include <waypoint_pkg/utilis.hpp>

class Txt2Marker : public rclcpp::Node
{
    public:

        // コンストラクタ
        Txt2Marker() : Node("txt2marker")
        {
            // パラメーター定義
            this->declare_parameter("reway_txt_file", "/home/ubuntu/ubuntu/rewaypoint.txt");

            // パラメーター取得
            this->get_parameter("reway_txt_file", txt_file_param_);

            // パブリッシャー定義
            this->marker_array_pub_ = this->create_publisher<MarkerArray>("waypoint_publihser", 100);

            // 実時間計測器の定義
            this->timer_ = this->create_wall_timer(10ms, std::bind(&Txt2Marker::ma_pub_callback_, this));

        }

        bool open_file()
        {
            ifs_.open(txt_file_param_.as_string().c_str());
            // ファイルの読み込みが失敗した場合
            if(ifs_.fail())
            {
                RCLCPP_ERROR(this->get_logger(), "The text file that was attempted to be read is unavailable.");
                ifs_.close();
                return false;
            }

            Waypoint wayp_tmp;
            std::string line_tmp;
            while(std::getline(ifs_, line_tmp))
            {
                std::istringstream iss(line_tmp, std::ios::in);
                std::vector<std::string> tokens;
                std::string token;
                
                while(std::getline(iss, token, ','))
                {
                    tokens.push_back(token);
                }


                if(tokens.size() != 8)
                {
                    RCLCPP_WARN(this->get_logger(), "The element is incomplete in the Waypoint.txt!");

                    // デバッグ
                    // std::stringstream ss;
                    // ss << tokens[0] << " "
                    // << tokens[1] << " "
                    // << tokens[2] << " "
                    // << tokens[3] << " "
                    // << tokens[4] << " "
                    // << tokens[5] << " "
                    // << tokens[6] << " " << std::endl;
                    // RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                    return false;
                }
                else
                {
                    wayp_tmp.pose.position.x = std::stod(tokens[0].c_str());
                    wayp_tmp.pose.position.y = std::stod(tokens[1].c_str());
                    wayp_tmp.pose.position.z = std::stod(tokens[2].c_str());
                    wayp_tmp.pose.orientation.x = std::stod(tokens[3].c_str());
                    wayp_tmp.pose.orientation.y = std::stod(tokens[4].c_str());
                    wayp_tmp.pose.orientation.z = std::stod(tokens[5].c_str());
                    wayp_tmp.pose.orientation.w = std::stod(tokens[6].c_str());
                    wayp_tmp.mode.data = atoi(tokens[7].c_str());             
                }
                waypoints_.push_back(wayp_tmp);
            }
            ifs_.close();
            return true;
        }

        // デストラクタ
        ~Txt2Marker()
        {
            if (ifs_.is_open())ifs_.close();
        }
    
    private:

        // パラメータインスタンス宣言
        rclcpp::Parameter txt_file_param_;

        // パブリッシャー宣言
        rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub_;

        // Waypointコンテナ宣言
        std::vector<Waypoint> waypoints_;

        // 実時間計測器宣言
        rclcpp::TimerBase::SharedPtr timer_;

        // ファイルストリーム宣言
        std::ifstream ifs_;

        // Callback関数定義
        void ma_pub_callback_()
        {
            // MarkerArrayの一時的インスタンス
            // MarkerArrayはMarkerの配列
            MarkerArray array_arrow, array_text;
            // Markerの一時的インスタンス
            Marker arrow, text;
            // 座標ID
            arrow.header.frame_id = text.header.frame_id = "map";
            // 時間スタンプ
            arrow.header.stamp = text.header.stamp = this->get_clock().get()->now();
            // Markerの名前空間
            arrow.ns = "ARROWS";
            text.ns = "TEXT";
            // RGBAを用いた色設定
            arrow.color.b = text.color.b = 1.0;
            text.color.g = 1.0;
            text.color.r = 1.0;
            arrow.color.a = text.color.a = 1.0;     
            // Markerの規模
            arrow.scale.x = 1.0;
            arrow.scale.y = 0.2;
            arrow.scale.z = text.scale.z = 0.4;
            // Markerの動作は「ADD」（追加）
            arrow.action = text.action = Marker::ADD;   
            // Markerの種類は「ARROW」と「TEXT_VIEW_FACING」
            arrow.type = ::Marker::ARROW;
            text.type = ::Marker::TEXT_VIEW_FACING;

            for (size_t i = 0; i < waypoints_.size(); i++)
            {
                Pose pose_tmp;

                // 三次元座標系の座標で定義する
                pose_tmp.position.x = waypoints_[i].pose.position.x;
                pose_tmp.position.y = waypoints_[i].pose.position.y;
                // 2D地図のため、高さは定数で定義する
                pose_tmp.position.z = 0.25;
                // 回転量、四元数座標
                pose_tmp.orientation = waypoints_[i].pose.orientation;
                // テキストマーカーと矢印マーカーの位置を定義する
                arrow.pose=text.pose = pose_tmp;
                // テキストマーカーのZ軸の数値も定数で定義する
                text.pose.position.z = 0.5;
                // テキストマーカーと矢印マーカーの番号はインデクスの数値で定義する
                arrow.id = text.id = i;
                // 矢印マーカーに関する説明は、テキストマーカーで行う
                // そのテキストの内容を下記のように示す
                // 「番号x、モードy」
                std::string str(
                    "No." + std::to_string(i) + 
                    "-mode" + std::to_string(waypoints_[i].mode.data)
                );
                // 上記の内容でテキストマーカーのテキストを定義する
                text.text = str;

                // Waypointのモードは「0」である場合
                // Waypointのモード
                if(waypoints_[i].mode.data == 0){
                    // 矢印マーカーは白色
                    arrow.color.r = 1.0;
                }
                else{
                    // 「0」ではない場合、矢印マーカーは青水色
                    arrow.color.r = 0.0;
                }

                // 矢印マーカーアレーとテキストマーカーアレーのインスタンスのメンバーである「markers」ベクトルに矢印マーカーとテキストマーカーのインスタンスを入れる
                array_arrow.markers.push_back(arrow);
                array_text.markers.push_back(text);
                
            }
            // マーカーを送信する
            this->marker_array_pub_->publish(array_arrow);
            this->marker_array_pub_->publish(array_text);            
            
        }

};

int main(int argc, char *argv[])
{
    // ノード初期化
    rclcpp::init(argc, argv);

    // Nodeインスタンス
    auto txt2marker_node = std::make_shared<Txt2Marker>();
    // ファイル読み込みが失敗した場合
    if (!txt2marker_node->open_file()){
        rclcpp::shutdown();
        return 0;
    }

    // 処理にスピンさせる
    rclcpp::spin(txt2marker_node);

    // 処理に終了させる
    rclcpp::shutdown();

    return 0;
}