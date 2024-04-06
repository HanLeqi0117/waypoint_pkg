#include <waypoint_pkg/marker_server.hpp>


WaypointHandler::WaypointHandler() : Node("marker_server")
{
    // パラメータ定義
    this->declare_parameter("read_file_name", std::string("/home/ubuntu/ubuntu/waypoint.txt"));
    this->declare_parameter("write_file_name", std::string("/home/ubuntu/ubuntu/rewaypoint.txt"));
    this->declare_parameter("save_service_name", std::string("save_service"));
    this->declare_parameter("update_service_name", std::string("update_service"));
    this->declare_parameter("debug", false);

    // パラメータ取得
    this->get_parameter("read_file_name", this->read_file_name_param_);
    this->get_parameter("write_file_name", this->write_file_name_param_);
    this->get_parameter("save_service_name", this->save_service_name_param_);
    this->get_parameter("update_service_name", this->update_service_name_param_);
    this->get_parameter("debug", this->debug_params_);

    // パブリッシャ定義
    this->marker_pub_ = this->create_publisher<Marker>("marker", 1);

	// マーカーサーバーの定義
	this->marker_server_ = std::make_shared<InteractiveMarkerServer>("interactive_marker_topic", this);

    /*
	サービス定義
    this->save_file_service_ = 
        this->create_service<std_srvs::srv::Trigger>(
            this->save_service_name_param_.as_string(), 
            std::bind(&WaypointHandler::save_file_callback_, this, _1, _2)
    );
    this->waypoints_update_service_ = 
        this->create_service<WaypointsUpdate>(
            this->update_service_name_param_.as_string(), 
            std::bind(&WaypointHandler::waypoints_update_callback_, this, _1, _2)
    );
	*/

    // スリープする
    rclcpp::sleep_for(10ms);

}

WaypointHandler::~WaypointHandler()
{
    save_file_();
    if(this->ifs_ptr_->is_open())ifs_ptr_->close();
    if(this->ofs_ptr_->is_open())ofs_ptr_->close();
}

bool WaypointHandler::open_files()
{
    // ファイルストリーム定義
    this->ifs_ptr_ = std::make_shared<std::ifstream>(
        this->read_file_name_param_.as_string().c_str()
    );
    this->ofs_ptr_ = std::make_shared<std::ofstream>(
        this->write_file_name_param_.as_string().c_str()
    );

    if(ifs_ptr_->fail() || ofs_ptr_->fail()){
        RCLCPP_ERROR(this->get_logger(), "Failed to open the Input file or Output file!");
        ifs_ptr_->close();
        ofs_ptr_->close();
        return false;
    }
    else {
		RCLCPP_INFO(this->get_logger(), "Files are opened.");
        return true;
	}
}

void WaypointHandler::initialize()
{
    // 一時インスタンス
    Waypoint wayp_tmp;
    std::string line_tmp;

    // ファイルの行を読み込む
    while(std::getline(*ifs_ptr_, line_tmp))
    {
        std::istringstream iss(line_tmp, std::ios::in);
        std::vector<std::string> tokens;
        std::string token;
        // デバッグ
		if (this->debug_params_.as_bool()){RCLCPP_INFO(this->get_logger(), "%s", iss.str().c_str());}
        
        // 行を列ごとに読み込む
        while(std::getline(iss, token, ','))
        {
            tokens.push_back(token);
        }

        // 行の内容を一時インスタンスに格納する
        wayp_tmp.pose.position.x = std::stod(tokens[0]);
        wayp_tmp.pose.position.y = std::stod(tokens[1]);
        wayp_tmp.pose.position.z = std::stod(tokens[2]);
        wayp_tmp.pose.orientation.x = std::stod(tokens[3]);
        wayp_tmp.pose.orientation.y = std::stod(tokens[4]);
        wayp_tmp.pose.orientation.z = std::stod(tokens[5]);
        wayp_tmp.pose.orientation.w = std::stod(tokens[6]);
        if(tokens.size() == 7) 
            wayp_tmp.mode.data = 0;
        else 
            wayp_tmp.mode.data = std::stoi(tokens[7]);

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

        waypoints_.push_back(wayp_tmp);
    }

	// MarkerServerの初期化
	// 引数1：メニューオプションのタイトル
	// 引数2：メニューのオプションをクリックした後の処理を行うCallback関数のオブジェクト（bind()関数で関数をオブジェクト化）
    // 機能：MarkerにMenuオプションを追加する。追加したMenuオプションがトリガされた場合、Callback関数で処理を行う
	menu_handler_.insert("Erase this Waypoint", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Add one Waypoint behind this Waypoint", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to NORMAL", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to SEARCH", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to CANCEL", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to DIRECT", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to STOP", std::bind(&WaypointHandler::process_feedback_, this, _1));
	menu_handler_.insert("Change the mode to SIGNAL", std::bind(&WaypointHandler::process_feedback_, this, _1));
	
	for(size_t i = 0; i < waypoints_.size(); i++){
		// マーカーを生成する関数
		make_marker_(i, waypoints_[i]);
	}
	// 変更を有効にする
	marker_server_->applyChanges();
	
	ifs_ptr_->close();
}

void WaypointHandler::save_file_()
{
    for (auto &&waypoint : waypoints_)
    {
        // RPY -> tf2_quat
        // tf2_quat -> geometry_quat
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(
            waypoint.raw.data,
            waypoint.pitch.data,
            waypoint.yaw.data       
        );
        auto geometry_quat = tf2::toMsg(tf2_quat);
        		
        // 書き込む内容の編集
		*ofs_ptr_ << waypoint.pose.position.x << "," 
            << waypoint.pose.position.y << "," 
            << waypoint.pose.position.z << "," 
            << geometry_quat.x << "," 
            << geometry_quat.y << "," 
            << geometry_quat.z << "," 
            << geometry_quat.w << "," 
            << static_cast<int>(waypoint.mode.data) << std::endl;
			
    }

    ofs_ptr_->close();
    
}

void WaypointHandler::make_marker_(int name, Waypoint wayp_tmp)
{
	// InteractiveMarker一時インスタンス
	InteractiveMarker int_marker;

	// 「int_marker」の初期化
	// 座標ID
	int_marker.header.frame_id = "map";
	// スケール
	int_marker.scale = 1;
	// 三次元座標値
	int_marker.pose.position.x = wayp_tmp.pose.position.x;
	int_marker.pose.position.y = wayp_tmp.pose.position.y;
	int_marker.pose.position.z = 0.2;

    // RPY -> tf2_quat
    // tf2_quat -> geometry_quat
    tf2::Quaternion tf2_quat;
    tf2_quat.setRPY(
        wayp_tmp.raw.data,
        wayp_tmp.pitch.data,
        wayp_tmp.yaw.data      
    );

    // 四元数
	int_marker.pose.orientation = tf2::toMsg(tf2_quat);

	std::stringstream ss;
	int_marker.name = std::to_string(name);
	// マーカーの説明文を作成する
	ss << "No." << int_marker.name.c_str() << "-" << "mode." << get_mode_name_(wayp_tmp.mode.data);
	// 説明文を「int_marker」インスタンスに渡す
	int_marker.description = ss.str();

    // 可動マーカーを制御するAPIのインスタンス
	InteractiveMarkerControl control, arrow_control;

	// 常にマーカーを表示
	// Trueの場合、GUIが可動な状態ではない際に、このインスタンス(ここは「control」)に含まれるマーカーインスタンスはいつも表示される状態になる
	control.always_visible = true;
	arrow_control.always_visible = true;
	// VIEW_FACINGモードにおいて, マーカーをカメラの視点に合わせない場合は, これをtrueに設定します. マーカーはINHERITモードと同様に表示されます.
	control.independent_marker_orientation = true;

	// Orientation_mode（オリエンテーション・モード）：方向がどのように変化するかをコントロールする。
	// マーカーに移動させるモードを「y-z平面においてマーカーに移動させる」モードに設定する
	arrow_control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;

	// 「Marker」のクラスのインスタンス「arrow_marker」を宣言する
	Marker arrow_marker;
	// 「arrow_marker」を定義する
	// マーカーの種類は矢印
	arrow_marker.type = Marker::ARROW;
	// マーカーのスケール
	arrow_marker.scale.x = 0.5;
	arrow_marker.scale.y = 0.1;
	arrow_marker.scale.z = 0.1;
	// マーカーの色	RGBA
	arrow_marker.color.r = 0.65;
	arrow_marker.color.g = 0.65;
	arrow_marker.color.b = 0.65;
	arrow_marker.color.a = 1.0;

	// 上記で定義した「arrow_marker」インスタンスを「arrow_control」インスタンスに渡す
	// 矢印マーカー -> 矢印マーカーの制御手法
	arrow_control.markers.push_back(arrow_marker);
	// 上記で定義した「arrow_control」インスタンスを「int_marker」インスタンスに渡す
	// 矢印マーカーの制御手法 -> 可動マーカー
	int_marker.controls.push_back(arrow_control);

	// 制御モードを「x軸に基づいて移動できる」ように設定する
	// この制御の名前は「move_x」で命名する
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	/*
	control.name = "rotate_x";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	*/
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	// 上記で定義した「control」インスタンスを「int_marker」に渡す
	int_marker.controls.push_back(control);

	// 制御モードを「z軸に基づいて回転できる」ように設定する
	// この制御の名前は「rotate_z」で命名する
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	/*
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);
	*/
	control.name = "rotate_z";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);

	// 制御モードを「y軸に基づいて移動できる」ように設定する
	// この制御の名前は「move_y」で命名する
	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	/*
	control.name = "rotate_y";
	control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
	int_marker.controls.push_back(control);
	*/
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	// 引数：可動マーカーのインスタンス
	// 機能：可動マーカーのインスタンスを追加また変更する
	marker_server_->insert(int_marker);

	// 引数1：可動マーカーの名前
	// 引数2：Callback関数のオブジェクト
    // 機能：この可動マーカーに処理Callback関数を登録する
	marker_server_->setCallback(
        int_marker.name, 
        std::bind(&WaypointHandler::process_feedback_, this, _1)
    );
	
    // 引数1：「InteractiveMarkerServer」のインスタンスのポインタ
	// 引数2：マーカーの名前
	// 機能：定義したマーカーの名前及びメニューのCallback関数を「menu_handler」に適用させる
	menu_handler_.apply(*marker_server_, int_marker.name);
}

void WaypointHandler::make_marker_all_(int start){

    for(size_t i = start; i < waypoints_.size(); i++){
        make_marker_(i, waypoints_[i]);
    }
	marker_server_->applyChanges();
}

void WaypointHandler::erase_marker_(int start){
    // 「start」の後の全てのWaypointを削除する
	for(size_t i = start; i < waypoints_.size(); i++){
		// マーカーの名前に基づいてマーカーを削除する
		marker_server_->erase(std::to_string(i));
	}
	// マーカーの変更を有効にしてクライアントに反映する
	marker_server_->applyChanges();
}

void WaypointHandler::insert_marker_(int name){
    erase_marker_(name);
    // 挿入するマーカーを定義する
    // その座標は、二つの現存のWaypointで構成された直線の中点座標である
    auto wayp_tmp = waypoints_[name];
    wayp_tmp.pose.position.x = (
        waypoints_[name].pose.position.x + 
        waypoints_[name + 1].pose.position.x
    ) / 2;
    wayp_tmp.pose.position.y = (
        waypoints_[name].pose.position.y + 
        waypoints_[name + 1].pose.position.y
    ) / 2;
    wayp_tmp.mode.data = 0;
	// Waypointコンテナの「name + 1」の位置に、Waypointを挿入する
	waypoints_.insert(waypoints_.begin() + name + 1, wayp_tmp);
	// マーカーを作成する
	make_marker_all_(name);
	// Waypoint間の線を引く
	publish_line_();
}

void WaypointHandler::process_feedback_(InteractiveMarkerFeedback::ConstSharedPtr feedback)
{

	// 選択条件：「feedback」のメンバ「event_type」
	switch(feedback->event_type)
	{
		// このマーカーの位置は、既に定義された制御手法によって変更された場合
		case InteractiveMarkerFeedback::POSE_UPDATE:
		{
			// 更新されたマーカーの名前
			unsigned int index = std::stoi(feedback->marker_name);
			
			// 「index」の数値と対応するベクトルコンテナ「wayp」に格納されているWaypointの位置情報を「feedback」インスタンスの中のデータで変更する
			// 三次元線形座標系の線形位置情報を変更する
			waypoints_[index].pose.position.x = feedback->pose.position.x;
			waypoints_[index].pose.position.y = feedback->pose.position.y;
			waypoints_[index].pose.position.z = feedback->pose.position.z;

			// 四元数座標系の回転位置情報を変更する
            // geometry_quat -> tf2_quat
            // tf2_quat -> tf2_mat
            // tf2_mat -> RPY
			tf2::Quaternion tf2_quat;
			tf2::fromMsg(feedback->pose.orientation, tf2_quat);
			tf2::Matrix3x3(tf2_quat).getRPY(
				waypoints_[index].raw.data,
				waypoints_[index].pitch.data,
				waypoints_[index].yaw.data            
			);

				// Waypointの間で線を引く
				publish_line_();
				break;
			}

		// このマーカーは、メニューで変更がある場合
		case InteractiveMarkerFeedback::MENU_SELECT:
		{
			// 「Menu 変更が起こったメニューのID selected \n」というログを出力する
			std::stringstream ss;
            ss << "Menu" << feedback->menu_entry_id << " selected" << std::endl;
            RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
			
			// メニューIDが「1」の場合、すなわちMarkerを削除する場合
			if(feedback->menu_entry_id == 1){
				// erase cube
				// このマーカー(Waypoint)を削除する
				unsigned int index = std::stoi(feedback->marker_name);
				std::stringstream ss;
                ss << "No" << index << "erase" << std::endl;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
				// 「index」番のWaypointを削除する
				erase_marker_(index);
				// ベクトルコンテナ「wayp」の「index」番の要素を削除する
				waypoints_.erase(waypoints_.begin() + index);
				// ベクトルコンテナ「wayp」に基づいてすべてのマーカーを作成する
				make_marker_all_(index);
				// Waypointの間で線を引く
				publish_line_();
			}

			// メニューIDが「2」の場合、すなわちMarkerを挿入する場合
			if(feedback->menu_entry_id == 2){
				unsigned int index = std::stoi(feedback->marker_name);
				std::stringstream ss;
                ss << "No" << index << "insert" << std::endl;
                RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
                insert_marker_(index);
            }

			// メニューIDが「3」より大きい場合、すなわちModeが変更されたため、この位置にあるWaypointを作り直す
			if(feedback->menu_entry_id >= 3){
				unsigned index = atoi(feedback->marker_name.c_str());
                // mode_id = menu_entry_id - 3
				waypoints_[index].mode.data = feedback->menu_entry_id - 3;
				// 「index」番のWaypointを削除する
				marker_server_->erase(std::to_string(index));
				// 変更を有効にする
				marker_server_->applyChanges();
				// 「index」番のWaypointを作成する
				make_marker_(index, waypoints_[index]);
			}
			break;
		}
	}
	// 変更を有効にする
	marker_server_->applyChanges();
}

void WaypointHandler::publish_line_(){
	// マーカーの点と線について、URL:<http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines>
	// 「Marker」のインスタンスを宣言する
	Marker line_strip;
	// 「line_strip」を定義する
	// 座標ID
	line_strip.header.frame_id = "map";
	// 時間スタンプ
	line_strip.header.stamp = this->get_clock().get()->now();
	// 名前空間
	line_strip.ns = "lines";
	// 動作は「ADD」追加
	line_strip.action = Marker::ADD;
	// 他の「line_strip.pose」はデフォルトが0。「orientation.w」のみ定義する必要あり
	// 回転が存在することを意味する
	line_strip.pose.orientation.w = 1.0;
	// このオブジェクトのIDは、他のオブジェクトから分別するための存在
	// このオブジェクトIDとネームスペースを合わせて、このオブジェクトの操作と削除などの処理に使える
	line_strip.id = 0;
	// このマーカーの種類はラインストリップ
	line_strip.type = Marker::LINE_STRIP;
	// マーカーのスケール
	line_strip.scale.x = 0.1;
	// マーカーの色
	line_strip.color.b = 1.0;
	line_strip.color.a = 1.0;
	// 「for」文で繰り返し処理を行う
	// 添え字「i」を「0」で初期化する
	// 繰り返し条件：添え字「i」がベクトルコンテナ「wayp」のサイズより小さい場合
	// 添え字の加算処理
	for(size_t i = 0; i < waypoints_.size(); i++){
        Point point_tmp;
		// 添え字と対応するWaypointの位置情報を「p」に渡す
		point_tmp = waypoints_[i].pose.position;
		// z軸の値を「0」で初期化する
		point_tmp.z = 0;
		// マーカーのベクトルコンテナ「points」に位置情報を含む「p」を渡す
		line_strip.points.push_back(point_tmp);
	}

	// マーカーを送信する関数「publish()」を呼び出す
	// 引数：マーカーのインスタンス
	marker_pub_->publish(line_strip);
}

std::string WaypointHandler::get_mode_name_(int mode_number){
	std::string mode_name;
	switch(mode_number){
		case 0:
			mode_name += "NORMAL";
			break;
		case 1:
			mode_name += "SEARCH";
			break;
		case 2:
			mode_name += "CANCEL";
			break;
		case 3:
			mode_name += "DIRECT";
			break;
		case 4:
			mode_name += "STOP";
			break;
		case 5:
			mode_name += "SIGNAL";
	}

	return mode_name;
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto waypoint_handler_node = std::make_shared<WaypointHandler>();
    
    // ファイルを開くことが成功した場合
    if(waypoint_handler_node->open_files()){
		waypoint_handler_node->initialize();
        rclcpp::spin(waypoint_handler_node);
    }
    else{
        rclcpp::shutdown();
        return 0;
    }

    rclcpp::shutdown();
    return 0;
}
