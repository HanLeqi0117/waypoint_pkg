#include <waypoint_pkg/utilis.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace interactive_markers;

class WaypointHandler : public rclcpp::Node
{
    public:

        // コンストラクタ
        WaypointHandler();
        
        // 機能：読み込みファイルと書き込むファイルを開く
        // 戻り値：読み取りファイルと書き込みファイルを開くことが成功した場合、Trueを戻す。逆の場合、Falseを戻す。
        bool open_files();

        // 機能：読み込みファイルからWaypoint情報を取得してWaypointのコンテナに格納して、各Callback関数を登録する
        void initialize();

        // デストラクタ
        ~WaypointHandler();

    private:

        // マーカーAPIの「InteractiveMarkerServer」のインスタンスの共有ポインタを宣言する
        std::shared_ptr<InteractiveMarkerServer> marker_server_;
        // Waypointのベクトルコンテナ「wayp」
        std::vector<Waypoint> waypoints_;

        // 引数：ROSメッセージインターフェイス「InteractiveMarkerFeedback」のコンスタントポインタ
        // 「InteractiveMarkerFeedback」の内容について、URL：http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerFeedback.html
        // 機能：異なるフィードバックの状態に基づいて、異なる処理(メニューの変更処理と位置情報の変更処理)を行う   		
        void process_feedback_(InteractiveMarkerFeedback::ConstSharedPtr feedback);
        // Waypointを編集できるようにする機能を与える関数.次のサイトを参考にして作られている
        // <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server>
        // <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls>
        // 引数1：Waypointとマーカーの番号
        // 引数2：「Waypoint」構造体のインスタンス	
        // 機能：「Waypoint」のインスタンス -> 動的マーカー(Waypoint)
        void make_marker_(int name, Waypoint wayp_tmp);

        // 引数：Waypointの番号
        // 機能：Waypointのコンテナに基づいて「start」番号からマーカーを作成する
        void make_marker_all_(int start);

        // 引数：Waypointの番号
        // 機能：「start」番号からすべてのマーカーを削除する
        void erase_marker_(int start);

        // 引数：挿入する場所を示す番号
        // 機能：「name」の後ろに一つWaypointを挿入する
        void insert_marker_(int name);
        
        // 編集した内容をファイルに書き込む
        void save_file_();

        // ファイルストリーム宣言
        std::shared_ptr<std::ifstream> ifs_ptr_;
        std::shared_ptr<std::ofstream> ofs_ptr_;

        // サービス宣言
        // rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_file_service_;
        // rclcpp::Service<WaypointsUpdate>::SharedPtr waypoints_update_service_;

        // パブリッシャ宣言
        rclcpp::Publisher<Marker>::SharedPtr marker_pub_;

        // パラメータ宣言
        rclcpp::Parameter read_file_name_param_;
        rclcpp::Parameter write_file_name_param_;
        rclcpp::Parameter save_service_name_param_;
        rclcpp::Parameter update_service_name_param_;
        rclcpp::Parameter debug_params_;
        
        // The Menu API of Marker
        MenuHandler menu_handler_;

        // Callback関数宣言
        // void waypoints_update_callback_(
        //     const std::shared_ptr<WaypointsUpdate::Request> waypoints_req, 
        //     std::shared_ptr<WaypointsUpdate::Response> waypoints_res
        // );
        // void save_file_callback_(
        //     const std::shared_ptr<std_srvs::srv::Trigger::Request> save_req,
        //     std::shared_ptr<std_srvs::srv::Trigger::Response> save_res
        // );

        // 機能：現にあるすべてのWaypointの位置情報にしたがって、近い2つのWaypointの間で線を引く
        void publish_line_();
        // 引数：Waypointのモードを示す番号(int型)
        // 戻り値：「mode_number」番号と対応するモードの名前
        // 機能：モード番号を渡してそのモード番号と対応する名前を取得する
        std::string get_mode_name_(int mode_number);
    
};