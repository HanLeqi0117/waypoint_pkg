#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <math.h>
#include <tf/LinearMath/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

// 参考文献　
// 可動マーカーについて、URL：https://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started
// ナビゲーションについて、URL：https://wiki.ros.org/turtlebot_navigation/Tutorials/Setup%20the%20Navigation%20Stack%20for%20TurtleBot

// ネームスペースを使用する
// 「visualization_msgs」はROSのメッセージインタフェイスのネームスペース
using namespace visualization_msgs;

// Waypointを保存する構造体
struct text_way{
    // ROSインタフェイス「Point」のインスタンス「pose」
	geometry_msgs::Point pose;
	// Yaw角
	double yaw;
	// モード
	int mode; 
};

// Waypointのクラスを宣言する
class wayp_class{
	public:

		// クラスのコンストラクタ関数を宣言する
		// コンストラクト引数：マーカーAPIの「InteractiveMarkerServer」のインスタンスの共有ポインタ
		// 共有ポインタはスマートポインタの中の一種類である。スマートポインタについて、URL：https://qiita.com/hmito/items/9b35a2438a8b8ee4b5af
		wayp_class(boost::shared_ptr<interactive_markers::InteractiveMarkerServer> serv);

		// デストラクタ関数を宣言する
		~wayp_class();

	private:

		// マーカーAPIの「InteractiveMarkerServer」のインスタンスの共有ポインタを宣言する
		boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

		// Waypointのベクトルコンテナ「wayp」
		std::vector<text_way> wayp;

		// 「processFeedback()」関数を定義する
		// 戻り値：なし
		// 引数：ROSメッセージインターフェイス「InteractiveMarkerFeedback」のコンスタントポインタ
		// 「InteractiveMarkerFeedback」の内容について、URL：http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerFeedback.html
		// 機能：異なるフィードバックの状態に基づいて、異なる処理(メニューの変更処理と位置情報の変更処理)を行う   		
		void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback);

		// Waypointを編集できるようにする機能を与える関数.次のサイトを参考にして作られている
		// <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Writing%20a%20Simple%20Interactive%20Marker%20Server>
		// <http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls>
		// 「text_way」のインスタンス -> 動的マーカー(Waypoint)
		// 引数1：Waypointの番号。int型
		// 引数2：「text_way」構造体のインスタンス		
		void makeCube(int name, text_way posi);

		// ベクトル「wayp」の「start」番後のすべての要素 -> 動的マーカー(Waypoint)
		// 引数：Waypointの番号
		void makeCubeALL(int start);

		// ファイルを読み込んでwaypointのx, y, z, yaw, modeも情報を配列waypに格納する
		// 「read_data()」関数を定義する
		// 引数：ファイルのパス
		// 戻り値：なし
		void read_data(std::string file_name);

		// 引数が代表するWaypointの番号からすべてのマーカーを削除する関数「eraseCube()」
		// 引数：Waypointの番号
		void eraseCube(int start);

		// マーカーを挿入する関数
		// 引数1：挿入する場所を示す番号
		// 引数2：挿入する内容(Waypoint情報)
		// 機能：「num」番にあるマーカーを削除して「data」に基づいて「num」番の場所で新しいマーカーを作成する
		void insertCube(int num, text_way data);

		// 編集した内容をファイルに書き込む
		void save_file();
		// NodeのAPIを宣言する
		ros::NodeHandle nh;

		// サブスクライバ
		ros::Subscriber string_sub;
		ros::Subscriber point_sub;

		// 読み込むファイルのパス
		std::string read_file_name_;
		// 書き込むファイルのパス
		std::string write_file_name_;

		// ROSパラメータの値を格納する変数
		std::string order_string_name_;
		std::string pose_topic_name_;

		// マーカーAPIの「menu_handler」のインスタンス
		interactive_markers::MenuHandler menu_handler;

		// 「string」型のデータを受信したとき処理を行うCallback関数
		// 引数：ROSメッセージインターフェイス「String」のコンスタントポインター
		// 機能：「way_command」の命令(ファイルに保存するかWaypointの番号を変更するか)によって処理を行う
		void stringCallback(const std_msgs::StringConstPtr& msg);

		// 「PoseWithCovarianceStamped」型のデータを受信したとき処理を行うCallback関数
		// 引数：ROSメッセージインターフェイス「PoseWithCovarianceStamped」のコンスタントポインター
		// 機能：指定された座標点の位置情報を受信して、すべてのWaypointの位置情報とその座標点とのユークリッド距離を計算して、
		// 		最短距離を示すWaypointの番号を記録して、その番号と対応するマーカーを削除して、指定された座標点の位置情報に基づいて、
		// 		その番号の場所で新しいWaypointのマーカーを生成する処理を行うCallback関数
		void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
		// マーカーを送信するパブリッシャ
		ros::Publisher marker_pub;

		// 「publish_line()」関数
		// 引数：なし
		// 戻り値：なし
		// 機能：現にあるすべてのWaypointの位置情報にしたがって、近い2つのWaypointの間で線を引く
		void publish_line();
		// get_mode_name()
		// 引数：Waypointのモードを示す番号(int型)
		// 戻り値：「mode_number」番号と対応するモードの名前
		// 機能：モード番号を渡してそのモード番号と対応する名前を取得する
		std::string get_mode_name(int mode_number);
};

// クラスのコンストラクタを定義する
// コンストラクト引数をクラスのメンバで初期化する
wayp_class::wayp_class( boost::shared_ptr<interactive_markers::InteractiveMarkerServer> serv):server(serv){
	
	// 可動マーカーのメニューAPIのインスタンス「menu_handler」を用いて、可動マーカーにメニューを入れる。

	// ここのメニューは、イメージ的にマウスで可動マーカーを右クリックしたら出てくるメニューのことである。
	// このメニューを入れる関数「insert()」について、
	// 引数1：メニューオプションのタイトルの名前
	// 引数2：このオプションを左クリックして処理を行うCallback関数のオブジェクトである
	// 「bind()」関数は、関数をオブジェクトとしてインスタンス化する関数である。「bind()」について、URL：https://qiita.com/hamukun8686/items/49fe85b9cfa14961505e
	// 		「bind()」関数の引数1：関数の参照(アドレス)
	// 		「bind()」関数の引数2：この関数におけるオブジェクトの参照(アドレス)
	// 		「bind()」関数の引数3：「引数1」で渡した関数の引数と対応してプレースホルダ。プレースホルダについて、URL：https://wa3.i-3-i.info/word118.html
	menu_handler.insert("Erase Cube", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode normal", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode serch", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode cancel", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode direct", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode stop", boost::bind(&wayp_class::processFeedback, this, _1));
	menu_handler.insert("change mode signal", boost::bind(&wayp_class::processFeedback, this, _1));

	// スリープ関数
	ros::Duration(0.1).sleep();

	// ROSのNodeのAPI「NodeHandle」のインスタンス「private_nh」を定義する。ネームスペースはローカルである。
	ros::NodeHandle private_nh("~");
	// ROSのパラメータを宣言する
	// 引数1：パラメータのキー
	// 引数2：パラメータの値を一時的保存する変数。ここの変数は全部Nodeオブジェクトのメンバー変数である。
	// 引数3：パラメータの値のデフォルト値
	private_nh.param("read_file_name", read_file_name_, std::string("waypoint.txt"));
	private_nh.param("write_file_name", write_file_name_, std::string("rewaypoint.txt"));
	private_nh.param("pose_topic_name", pose_topic_name_, std::string("rewaypoint.txt"));
	private_nh.param("order_string_name", order_string_name_, std::string("rewaypoint.txt"));
	
	// read_data関数でwaypoint.txtファイルを読み込んでwaypointのx, y, z, yaw, modeをベクトルコンテナwaypに格納する
	read_data(read_file_name_);

	// 「for」文を用いて繰り返し処理
	// 添え字「i」は「0」で初期化する
	// 繰り返し条件：Waypointのコンテナ「wayp」のサイズより小さい場合
	// 添え字の加算処理
	for(int i = 0; i < wayp.size(); i++){
		// マーカーを生成する関数
		makeCube(i, wayp[i]);
	}

	// 変更を有効にする
	server->applyChanges();

	// マーカーを送信するパブリッシャのインスタンスを定義する
	// テンプレート：ROSメッセージインターフェイスのクラス名
	// コンストラクト引数1：トピックの文字列
	// コンストラクト引数2：データ列のサイズ
	marker_pub = nh.advertise<Marker>("marker", 1);

	// 文字列のコマンドを受信するサブスクライバを定義する
	// コンストラクト引数1：トピックの名前
	// コンストラクト引数2：データ列のサイズ
	// コンストラクト引数3：Callback関数
	// コンストラクト引数4：当オブジェクトのポインター
	string_sub = nh.subscribe(order_string_name_, 1, &wayp_class::stringCallback, this);

	// 位置情報のコマンドを受信するサブスクライバを定義する
	// コンストラクト引数1：トピックの名前
	// コンストラクト引数2：データ列のサイズ
	// コンストラクト引数3：Callback関数
	// コンストラクト引数4：当オブジェクトのポインター
	point_sub = nh.subscribe(pose_topic_name_, 1, &wayp_class::poseCallback, this);

	publish_line();
}

void wayp_class::publish_line(){
	// マーカーの点と線について、URL:<http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Points%20and%20Lines>
	// 「Marker」クラスのインスタンス「line_strip」を宣言する
	Marker line_strip;
	// 「line_strip」を定義する
	// 座標ID
	line_strip.header.frame_id = "map";
	// 時間スタンプ
	line_strip.header.stamp = ros::Time::now();
	// ネームスペース
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
	for(int i = 0; i < wayp.size(); i++){
		// ROSメッセージインターフェイス「Point」のインスタンス「p」を宣言する
		geometry_msgs::Point p;
		// 添え字と対応するWaypointの位置情報を「p」に渡す
		p = wayp[i].pose;
		// z軸の値を「0」で初期化する
		p.z = 0;
		// マーカーのベクトルコンテナ「points」に位置情報を含む「p」を渡す
		line_strip.points.push_back(p);
	}

	// マーカーを送信する関数「publish()」を呼び出す
	// 引数：マーカーのインスタンス
	marker_pub.publish(line_strip);
}

wayp_class::~wayp_class(){
	server.reset();
}

void wayp_class::read_data(std::string file_name){
	
	// 「ifstream」クラスのインスタンス「ifs」を定義する。
	// 引数：ファイルのパスのchar型文字
	std::ifstream ifs(file_name.c_str());

	// ファイルが開けない場合
	if(ifs.fail()){
		std::cerr << "file open error.\n";
		return;
	}

	// ファイル内の一行の内容を格納する一時的文字列型の変数
	std::string str;
	// Waypointに関する情報を格納する一時的変数を宣言する
	// 「x, y, z」は、Waypointの三次元線形座標系の座標軸
	// 「qx, qy, qz, qw」は、Waypointの回転量を表す四元数座標系の座標軸
	double x, y, z, qx, qy, qz, qw;
	// 「count」は、ファイルの一行内のカンマの数
	// 「mode」は、このWaypointのモードを表すコード
	int count, mode;
	// ファイルの一行目であるかどうか。Trueの場合、一行目である。Falseの場合、一行目ではない。
	bool first=true;
	// text_wayは構造体のインスタンスを宣言する
	text_way tmp;
	
	//read_file_name_のファイルの中身の1行1行の','の数を数える。初めてこのプログラムを実行するとファイルには、x,y,z,ox,oy,oz,owのみなので、','は6個になる。二回目以降はtmp.mode=0で','が1個増えるから7個になる
	
	// 「while」文で繰り返し処理
	// 判断条件：「getline()」関数の戻り値
	// 「getline()」関数は、ファイルの一行(改行符号「\n」まで)を読み込み、一時変数に格納する
	// 引数1：ファイルのパスの文字列
	// 引数2：ファイルの一行の内容を格納する一時変数
	while(getline(ifs, str)){
		// 最初の行の場合、カンマの数を数える処理だけを行う
		if(first){
			for (char i : str)
			if (i == ',')
			count++;
		}

		// 最初行フラグをfalseにする
		first=false;

		// 一行の中、「カンマ」の数が「6」である場合
		if(count==6){
			// 一行の中の情報を分けて異なる一時的変数に渡す
			// 「sscanf()」関数は、データを特定なフォーマットにしたがって読み取る関数である。
			// 引数1：読み取る予定のデータ。
			// 		「data()」関数は、string型のデータのポインタを戻す関数である
			// 引数2：データを読み取るフォーマット。「%lf」はプレースホルダである。
			// 引数3〜：「引数２」でプレースホルダの場所と対応する一時的変数の参照
			sscanf(str.data(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &qx, &qy, &qz, &qw);
			// Waypointのモードを「0」(ノーマルモード)に設定する
			tmp.mode = 0;
		}
		// 一行の中、「カンマ」の数が「7」である場合
		else{
			sscanf(str.data(), "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &x, &y, &z, &qx, &qy, &qz, &qw, &mode);
			tmp.mode = mode;
		}

		// 上記の処理で取得した一時変数をWaypoin構造体のインスタンス「tmp」に渡す
		tmp.pose.x = x;
		tmp.pose.y = y;
		tmp.pose.z = z;
		// RPY座標系の座標値を格納する一時変数
		double r, p, y;
		// tfライブラリーの「Quaternion」クラスのインスタンス「quat」を宣言する
		// コンストラクト引数１〜４：四元数座標系の座標値
		tf::Quaternion quat(qx, qy, qz, qw);
		// 「Matrix3x3」クラスのインスタンスを生成する
		// コンストラクト引数：四元数インスタンスの「quat」
		// メンバ関数「getRPY()」は、四元数からRPYを算出する関数
		// 引数1〜3：四元数に基づいて算出されたRPYの値を一時的に保存する変数
		tf::Matrix3x3(quat).getRPY(r, p, y);
		// 2次元のWaypointのため、Yaw角だけが変化するため、「tmp」に渡すのは、上記で取得した「y」だけを格納する
		tmp.yaw = y;
		// コンテナ「wayp」の最終列に「tmp」を格納する
		wayp.push_back(tmp);
	}

	// ファイルを閉じる
	ifs.close();
}

void wayp_class::processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
	// 「switch」文を用いて、選択処理
	// 選択条件：「feedback」のメンバ「event_type」
	switch(feedback->event_type)
	{
		// このマーカーの位置は、既に定義された制御手法によって変更された場合
		case InteractiveMarkerFeedback::POSE_UPDATE:
		{
			// 「atoi()」関数
			// 引数：「feedback」ポインタの「marker_name」のchar型
			// 戻り値：int型のデータ
			// 上記のデータで符号なしint型の変数「index」を定義する
			unsigned int index = atoi(feedback->marker_name.c_str());
			
			// 「index」の数値と対応するベクトルコンテナ「wayp」に格納されているWaypointの位置情報を「feedback」インスタンスの中のデータで変更する
			// 三次元線形座標系の線形位置情報を変更する
			wayp[index].pose.x = feedback->pose.position.x;
			wayp[index].pose.y = feedback->pose.position.y;
			wayp[index].pose.z = feedback->pose.position.z;

			// 四元数座標系の回転位置情報を変更する
			tf::Quaternion quat(
				feedback->pose.orientation.x,
				feedback->pose.orientation.y,
				feedback->pose.orientation.z,
				feedback->pose.orientation.w
			);
			double r, p, y;
			tf::Matrix3x3(quat).getRPY(r, p, y);
			wayp[index].yaw = y;

			// Waypointの間で線を引く
			publish_line();
			break;
		}
		// このマーカーは、メニューで変更がある場合
		case InteractiveMarkerFeedback::MENU_SELECT:
		{
			// 「Menu 変更が起こったメニューのID selected \n」というログを出力する
			std::cout << "Menu" << feedback->menu_entry_id << " selected" << std::endl;
			
			// メニューIDが「1」の場合
			if(feedback->menu_entry_id == 1){
				// erase cube
				// このマーカー(Waypoint)を削除する
				unsigned index = atoi(feedback->marker_name.c_str());
				std::cout << "No" << index << "erase" << std::endl;
				// 「index」番のWaypointを削除する
				eraseCube(index);
				// ベクトルコンテナ「wayp」の「index」番の要素を削除する
				wayp.erase(wayp.begin() + index);
				// ベクトルコンテナ「wayp」に基づいてすべてのマーカーを作成する
				makeCubeALL(index);
				// Waypointの間で線を引く
				publish_line();
			}
			// メニューIDが「2」より大きい場合
			if(feedback->menu_entry_id >= 2){
				unsigned index = atoi(feedback->marker_name.c_str());
				// 「index」番のWaypointモード = フィードバックのメニューID - 2
				wayp[index].mode = feedback->menu_entry_id - 2;
				std::stringstream s;
				s << index;
				// 「index」番のWaypointを削除する
				server->erase(s.str());
				// 変更を有効にする
				server->applyChanges();
				// 「index」番のWaypointを作成する
				makeCube(index, wayp[index]);
			}
			break;
		}
	}
	// 変更を有効にする
	server->applyChanges();
}

void wayp_class::makeCube(int name, text_way posi)
{
	// 動的マーカー「InteractiveMarker」クラスのインスタンス「int_marker」を宣言する
	InteractiveMarker int_marker;

	// 「int_marker」の初期化
	// 座標ID
	int_marker.header.frame_id = "map";
	// スケール
	int_marker.scale = 1;
	// 「text_way」に格納された位置情報を「int_marker」に渡す
	int_marker.pose.position.x = posi.pose.x;
	int_marker.pose.position.y = posi.pose.y;
	int_marker.pose.position.z = 0.2;

	// オイラー角からクォータニオンに変換するコード。次のサイトが分かりやすい。<https://myenigma.hatenablog.com/entry/20130719/1374304154>
	// 次のサイトは,オイラー角とクォータニオンについて詳しくまとめている。<https://qiita.com/srs/items/93d7cc671d206a07deae>
	// オイラー角(RPY):Yaw: Z方向の回転、Pitch: Y方向の回転、roll: X方向の3つのパラメーターで回転で表す形式
	// 例えば(10, 20, 30)とあったらZ軸で30度回した後に、新しくできたY軸で20度回して、新しくできたX軸で10度回します。このようにZ-Y-Xの順番で回します。注意ですが回る順番を変えると結果が変わってしまいます。特定の分野では順番が違ったりしますが、ロボットの世界ではこの順番がメジャー。
	tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, posi.yaw);
	// 上記のクォータニオンのデータはtf::Quaternionなので、トピックとしてよく使うgeometry_msgs::Quaternionに変換するする場合は、下記のようにquaternionTFToMsgという関数を使うと簡単
	geometry_msgs::Quaternion geometry_quat;

	// tfライブラリーの関数「quaternionTFToMsg()」を用いて、tf風の四元数インスタンスから「geometry_msgs」風の四元数インスタンスに変換する
	// 引数1：変換元(TF風)
	// 引数2：変換先(geometry_msgs風)
	quaternionTFToMsg(quat, geometry_quat);
	// geometry_msgs風の四元数変数を「int_marker」インスタンスに渡す
	int_marker.pose.orientation = geometry_quat;

	// int型のWaypointの番号情報をstring型に変換する工夫
	// 「std」ライブラリの「stringstream」クラスのインスタンス「s」と「sdes」を宣言する
	std::stringstream s, sdes;
	// Waypointの番号情報を一時的に保存する変数「name」から「s」に渡す
	s << name;
	// string型のWaypoint番号情報を「int_marker」インスタンスに渡す
	int_marker.name = s.str();
	// マーカーの説明文を作成する
	sdes << "No." << name << "-" << "mode." << get_mode_name(posi.mode);
	// 説明文を「int_marker」インスタンスに渡す
	int_marker.description = sdes.str();

	// ROSメッセージインターフェイスのネームスペース「visualization_msgs」の「InteractiveMarkerControl」クラスをインスタンス化する
	// <http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/InteractiveMarkerControl.html>
	// マーカーの回転を制御するインスタンスを作成する
	InteractiveMarkerControl control;
	// マーカーの移動を制御するインスタンスを作成する
	InteractiveMarkerControl arrow_control;
	// visualization_msgs::Markerの仕様<http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html>

	// 常にマーカーを表示
	// Trueの場合、GUIが可動な状態ではない際に、このインスタンス(ここは「control」)に含まれるマーカーインスタンスはいつも表示される状態になる
	control.always_visible = true;
	arrow_control.always_visible = true;
	// VIEW_FACINGモードにおいて, マーカーをカメラの視点に合わせない場合は, これをtrueに設定します. マーカーはINHERITモードと同様に表示されます.
	control.independent_marker_orientation = true;

	// Orientation_mode（オリエンテーション・モード）：方向がどのように変化するかをコントロールする。
	// void rviz::InteractiveMarkerControl::movePlane(Ogre::Ray & mouse_ray), <http://docs.ros.org/en/jade/api/rviz/html/c++/classrviz_1_1InteractiveMarkerControl.html#a63eb6ef9defc103d7a12a87aa9729bcb>
	// マーカーに移動させるモードを「y-z平面においてマーカーに移動させる」モードに設定する
	// 「MOVE_PLANE(4)」は「InteractiveMarkerControl」APIに仕込まれる列挙数の一つである
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

	// 「server」ポインタの「insert()」メンバ関数を呼び出す
	// 引数：可動マーカーのインスタンス
	// 機能：可動マーカーのインスタンスを追加また変更する
	server->insert(int_marker);
	// 「server」ポインタの「setCallback()」メンバ関数を呼び出す
	// 引数1：可動マーカーの名前
	// 引数2：Callback関数のオブジェクト
	server->setCallback(int_marker.name, boost::bind(&wayp_class::processFeedback, this, _1));
	// 定義したマーカーの名前及びメニューのCallback関数を「menu_handler」に適用させる
	// 引数1：「InteractiveMarkerServer」のインスタンス
	// 引数2：マーカーの名前
	menu_handler.apply(*server, int_marker.name);
}

void wayp_class::makeCubeALL(int start){
	for(int i = start; i < wayp.size(); i++){
		makeCube(i, wayp[i]);
	}
	server->applyChanges();
}

void wayp_class::eraseCube(int start){
	// Waypointの番号から最後のWaypointの番号まで「for」文の繰り返し処理
	for(int i = start; i < wayp.size(); i++){
		std::stringstream s;
		s << i;
		// マーカーの名前に基づいてマーカーを削除する
		server->erase(s.str());
	}
	// マーカーの変更を有効にしてクライアントに反映する
	server->applyChanges();
	return;
}

void wayp_class::insertCube(int num, text_way data){
	// 「num」番のマーカーを削除する
	eraseCube(num);
	// ベクトル「wayp」の「num + 1」番に「data」を入れる
	wayp.insert(wayp.begin() + num, data);
	// マーカーを作成する
	makeCubeALL(num);
	// Waypoint間の線を引く
	publish_line();
}


void wayp_class::save_file(){
	// 「ofstream」のインスタンスを作成する
	// 「ofstream」とは、Output File Stream
	std::ofstream ofs;
	// ファイルを開ける
	// 引数1：char型のファイルのパス
	// 引数2：処理モードを「out」にする
	ofs.open(write_file_name_.c_str() , std::ios::out);

	// 処理が失敗した場合
	if (ofs.fail()){
		std::cerr << "失敗" << std::endl;
		return;
	}

	// Waypointの位置情報を一時的に保存する変数
	double x, y, z, yaw;
	// 「for」文で繰り返し処理
	for(unsigned int num = 0 ; num < wayp.size() ; num++){
		tf::Quaternion quat=tf::createQuaternionFromRPY(0, 0, wayp[num].yaw);
		geometry_msgs::Quaternion geometry_quat;
		quaternionTFToMsg(quat, geometry_quat);
		// 書き込む内容の編集
		ofs << wayp[num].pose.x << "," << wayp[num].pose.y << "," << wayp[num].pose.z << "," 
			<< geometry_quat.x << "," << geometry_quat.y << "," << geometry_quat.z << "," 
			<< geometry_quat.w << "," << wayp[num].mode << std::endl;
	}
	// ファイルを閉じる
	ofs.close();
}

void wayp_class::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){
	text_way tmp;
	// 受信した情報を一時インスタンスに保存する
	tmp.pose = msg->pose.pose.position;
	double qx, qy, qz, qw, r, p, y;
	qx = msg->pose.pose.orientation.x;
	qy = msg->pose.pose.orientation.y;
	qz = msg->pose.pose.orientation.z;
	qw = msg->pose.pose.orientation.w;
	tf::Quaternion quat(qx, qy, qz, qw);
	tf::Matrix3x3(quat).getRPY(r, p, y);
	tmp.yaw = y;
	tmp.mode = 0;
	// 最短距離を1000に設定する
	double min_distance = 1000;
	// 最短距離の番号
	int min_num = 0;
	// すべてのWaypointを「for」文で繰り返し処理
	for(int i = 0; i < wayp.size(); i++){
		// 受信したデータが示す位置と「i」番のWaypointが示す位置のユークリッド距離
		double tmp_dis = std::hypot(tmp.pose.x - wayp[i].pose.x, tmp.pose.y - wayp[i].pose.y);
		// ユークリッド距離は最短距離より小さい場合
		if(tmp_dis < min_distance){
			// 最短距離をユークリッド距離で上書きする
			min_distance = tmp_dis;
			// 最短距離の番号を今の添え字「i」で上書きする
			min_num = i;
		}
	}
	// 最短距離を示す番号の場所でWaypointを挿入する
	insertCube(min_num + 1, tmp);
}

void wayp_class::stringCallback(const std_msgs::StringConstPtr& msg){

	// 受信したデータは「save」である場合
	if(msg->data == "save"){
		// ファイルを保存する
		save_file();
		return;
	}

	// ベクトルのインスタンス「v」を作成する
	// テンプレート：int型
	std::vector<int> v;
	// 受信したデータを「data」に一時的に保存する
	// データの構造：「v[0]」は変更元のWaypointの番号。「v[1]」は変更先のWaypointの番号。
	std::string data = msg->data;
	std::string s;
	std::stringstream ss{data};

	// 「while」繰り返し処理
	// 受信したデータをstring型からint型に変換してベクトル「v」に格納する
	while (getline(ss, s, ',')){
		v.push_back(atoi(s.c_str()));
	}

	// 変更元のWaypointの番号のWaypoint情報を一時的に保存する
	text_way backup_data = wayp[v[0]];
	// 変更元の番号が変更先の番号より小さい場合
	if(v[0] < v[1]){
		eraseCube(v[0]);
		// 変更元の番号のWaypointを削除する
		wayp.erase(wayp.begin() + v[0]);
		// 変更先の番号の場所にWaypointを挿入する
		wayp.insert(wayp.begin() + v[1] - 1, backup_data);
		makeCubeALL(v[0]);
	}
	// 大きい場合
	else{
		eraseCube(v[1]);
		wayp.erase(wayp.begin() + v[0]);
		wayp.insert(wayp.begin() + v[1], backup_data);
		makeCubeALL(v[1]);
	}
}

std::string wayp_class::get_mode_name(int mode_number){
	std::string mode_name;
	switch(mode_number){
		case 0:
			mode_name += "normal";
			break;
		case 1:
			mode_name += "serch_mode";
			break;
		case 2:
			mode_name += "cancel";
			break;
		case 3:
			mode_name += "direct";
			break;
		case 4:
			mode_name += "stop";
			break;
		case 5:
			mode_name += "signal";
	}

	return mode_name;
}

int main(int argc, char** argv)
{
	// ROSのNodeを初期化する
	// 引数1：端末の引数の数
	// 引数2：端末の引数を入れている配列
	// 引数3：Nodeの名前
	ros::init(argc, argv, "wayp_edit");
	// 「InteractiveMarkerServer」の共有ポインターを作成する
	// 共有ポインターのコンストラクト引数：「InteractiveMarkerServer」のポインター
	// 「InteractiveMarkerServer」のコンストラクト引数：トピックのネームスペース
	boost::shared_ptr<interactive_markers::InteractiveMarkerServer> sv(new interactive_markers::InteractiveMarkerServer("selection"));
	// Waypointのクラスのインスタンス「ob」
	// コンストラクト引数：「sv」
	wayp_class ob(sv);
	// 「spinner」は、「MultiThreadedSpinner」のインスタンス
	// コンストラクト引数：スレッドの数
	ros::MultiThreadedSpinner spinner(2); 
	// spin()関数でこのスコープで処理に待機させる
	spinner.spin();
}