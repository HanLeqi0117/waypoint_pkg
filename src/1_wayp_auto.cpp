#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <std_msgs/String.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/TransformStamped.h>

#include <visualization_msgs/Marker.h>

// Waypointを構成するPoint構造体
struct Point
{
	// 三次元座標系の座標
	double x;
	double y;
	double z;
	// 回転を表す四元数座標
	// x, y, zは虚数部を表す。wは実数部を表す。
	// 四元数とは、URL：https://ja.wikipedia.org/wiki/%E5%9B%9B%E5%85%83%E6%95%B0
	double ox;
	double oy;
	double oz;
	double ow;
};

int main(int argc, char **argv)
{
	// ROSのNodeの初期化
	// 引数1：端末の引数の数
	// 引数2：端末の引数が入れている配列、型はchar
	// 引数3：当Nodeの名前
	ros::init(argc, argv, "wayp_auto");
	// NodeのAPIインスタンスを宣言する。Pathの領域はグローバルである
	ros::NodeHandle node;
	// NodeのAPIインスタンスを宣言する。Pathの領域はローカルである
	ros::NodeHandle ph("~");

	// ファイルのアドレスを一時的に保存する文字列インスタンスを宣言する
	std::string read_file;
	// 近い2つのWaypoint間の直線距離
	double point_distance;
	// 車体回転における円弧の弧度差分閾値
	double deg_thresh;
	// 車体回転における円弧の弦差分閾値
	double deg_chord;
	// 「ph」というローカルのNodeのインスタンスを用いて、ROSのパラメータを定義する
	ph.param("read_file", read_file, std::string("/home/robo/つくば練習中/way_point/practice_waypoints.txt"));
	ph.param("point_distance", point_distance, 4.0);
	ph.param("deg_thresh", deg_thresh, 15.0);
	ph.param("deg_chord", deg_chord, 1.0);

	// 「ofs」は、std:ofstreamのインスタンス。ofstreamは、Output File Streamである
	std::ofstream ofs;
	// 「ofs」インスタンスのメンバー関数「open()」を用いて、ファイルを開く
	// 引数1：char型のファイルのパス
	// 引数2：処理モードはアウトプット
	ofs.open(read_file.c_str() , std::ios::out);
	// ファイルが開けない場合
	if (ofs.fail())
	{
		std::cerr << "waypointのファイルが見つかりません" << std::endl;
		return -1;
	}

	// Waypointのインデクスを0で初期化する
	int way_count = 0;
	// 「waypoint」は、要素がPoint構造体であるベクトルコンテナのインスタンスを宣言する
	std::vector<Point> waypoint;
	// ロボットのRPY姿勢を表す変数を宣言する
	// ラベル「base」
	double roll_base, pitch_base, yaw_base;
	double roll_now, pitch_now, yaw_now;

	// 「tf::TransformListener」は、ROSのtf座標変換情報を受信するAPIである。
	// 「listener」は、そのAPIのインスタンスである。
	tf::TransformListener listener;
	// 「tf::StampedTransform」は、ROSのtf座標変換情報を保存するインタフェイスである。
	// 「transform」は、そのインタフェイスのインスタンスである。
	tf::StampedTransform transform;

	// 「ros::Publisher」は、ROSに関するデータを送信するAPIである
	// 「maker_pub」は、Publihserのインスタンスである
	// テンプレート：visualization_msgs::Marker
	// 引数1：送信先のトピック
	// 引数2：データ列のサイズ
	ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>("/waypoint_marker", 100);

	// 繰り返し処理の周期の定義
	ros::Rate rate(10);

	// 繰り返し処理：
	// 判断条件：このNodeは生きている場合
	while(ros::ok())
	{
		// 繰り返し処理：
		// 判断条件：このNodeは生きている場合
		while(ros::ok())
		{
			try
			{
				printf("get...\n");
				// 「listener」のメンバ関数である「lookupTransform()」を呼び出す
				// 引数1：変換先の座標ID
				// 引数2：変換元の座標ID
				// 座標ID「/map」と座標ID「/base_link」の間の座標変換関係情報を「transform」に保存する
				// ここで、「transform」の座標IDは「/map」であり、子座標IDは「/base_link」(ロボットの基盤)である
				listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

			}
			// tfの例外処理
			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				// 今回の繰り返し処理をスキップする
				continue;
			}

			break;
		}

		// Point構造体のインスタンス「tmp_point」を新規する
		Point tmp_point;
		// 三次元線形座標系のデータを「tmp_point」に保存する
		tmp_point.x = transform.getOrigin().x();
		tmp_point.y = transform.getOrigin().y();
		tmp_point.z = transform.getOrigin().z();
		// 四元数座標系のデータを「tmp_point」に保存する
		tmp_point.ox = transform.getRotation().x();
		tmp_point.oy = transform.getRotation().y();
		tmp_point.oz = transform.getRotation().z();
		tmp_point.ow = transform.getRotation().w();
		///co.IN_POINT(tmp_point.x,tmp_point.y,tmp_point.z,tmp_point.ox,tmp_point.oy,tmp_point.oz,tmp_point.ow,way_count);

		// Waypointのインデクスはゼロの場合
		// このif文の先は2つのWaypoint間の距離の差分を計算するため、最初Waypointに関する処理が終わったらWhileの繰り返し処理から飛び出す
		if(way_count == 0)
		{
			// 「tf::Quaternion」は、四元数のクラス
			// 「q_base」は、そのインスタンスである。四元数の4つの要素を用いて定義する
			tf::Quaternion q_base(tmp_point.ox, tmp_point.oy, tmp_point.oz, tmp_point.ow);
			// 「tf::Matrix3x3」は、3*3行列式のクラス
			// 「m_base」は、そのインスタンス。四元数のインスタンスで定義する
			tf::Matrix3x3 m_base(q_base);
			// 「m_base」のメンバー関数「getRPY()」を用いてRPYを計算する
			// RPYの3つの姿勢データは、別々に引数1から引数3に保存される
			// 関数「getRPY()」の引数は、参照渡しを使用した。
			// 参照渡しとは、URL：https://wa3.i-3-i.info/word16070.html
			m_base.getRPY(roll_base, pitch_base, yaw_base);

			// コンテナ「waypoint」の「push_back()」関数を用いて、「tmp_point」を格納する
			waypoint.push_back(tmp_point);
			// Waypointのインデクスを加算処理する
			way_count++;
			// 今回の繰り返し処理をスキップする
			continue;
		}

		// 2つのWaypoint間の距離差分を計算する
		double diff_way = std::sqrt(std::pow((tmp_point.x - waypoint[way_count-1].x), 2.0) + std::pow((tmp_point.y - waypoint[way_count-1].y), 2.0));

		// 行150の処理と類似する
		tf::Quaternion q_now(tmp_point.ox, tmp_point.oy, tmp_point.oz, tmp_point.ow);
		tf::Matrix3x3 m_now(q_now);
		m_now.getRPY(roll_now, pitch_now, yaw_now);

		// Yaw角の差分の絶対値を計算する
		double diff_yaw = yaw_now - yaw_base;
		diff_yaw = std::abs(diff_yaw);

		// 処理プロセスのログ
		std::cout << "No. " << way_count << " " << "yaw_base " << yaw_base * 180.0 / M_PI << " yaw_now " << yaw_now * 180.0 / M_PI << " diff_yaw " << diff_yaw * 180.0 / M_PI << " diff_way " << diff_way << std::endl;

		// Yaw角の差分は円弧の弧度差分閾値より大きい場合かつ距離差分は円弧の弦差分閾値より大きい場合
		// また、距離差分は直線距離差分より大きい場合
        if((diff_yaw * 180.0 / M_PI > deg_thresh && diff_way > deg_chord) || diff_way > point_distance)
		{	
			// 基準Yaw角の変数を現在のYaw角の変数で更新する
			yaw_base = yaw_now;
			// コンテナ「waypoint」に「tmp_point」を入れる
			waypoint.push_back(tmp_point);
			// Waypointのインデクスを加算処理する
			way_count++;
		}

		// 「Marker」は、ROSメッセージインタフェイスのクラスである
		// 「points」は、そのクラスのインスタンスである
		visualization_msgs::Marker points;
		// 座標ID
		points.header.frame_id = "/map";
		// 時間スタンプ
		points.header.stamp = ros::Time::now();
		// ネーム
		points.ns = "POINTS";
		// マーカーの色
		points.color.b = 1.0;
		points.color.a = 1.0;
		// マーカーのスケール
		points.scale.x = 0.3;
		points.scale.y = 0.3;
		points.scale.z = 0.3;
		// マーカーの動作
		// 「ADD」は、「Marker」に仕組まれた列挙数の一つであり、マーカーを追加することを意味する
		points.action = visualization_msgs::Marker::ADD;
		// マーカーの種類
		// 「POINTS」は、「Marker」に仕組まれた列挙数の一つであり、マーカーの種類は点に指定する
		points.type = visualization_msgs::Marker::POINTS;
		// マーカーの姿勢の四元数の実数部を1.0に指定する
		// 幾何学的な意味は回転しないこと(恒等変換)を指す
		points.pose.orientation.w = 1.0;

		// 送信するマーカーの座標をwaypointから取得する
		// 処理が冗長である。ROS2で改善すべき
		for(int i = 1; i < waypoint.size(); i++)
		{
			// ここの「Point」は、最初で定義した「Point」構造体と異なる
			// ここの「Point」は、ROSメッセージインスタンスのネームスペース「geometry_msgs」にあるクラス「Point」である
			// 「p」は、そのクラスのインスタンス
			geometry_msgs::Point p;
			// ROSのインタフェイスの「p」に構造体の「waypoint」の座標系データを入れる
			p.x = waypoint[i].x;
			p.y = waypoint[i].y;
			p.z = 0.25;
			// マーカーのインスタンス「points」のメンバ「points」ベクトルに「p」を入れる
			points.points.push_back(p);
		}

		// マーカー「points」を送信する
		marker_pub.publish(points);

		// Nodeの処理はここで一回遅滞する
		ros::spinOnce();
		// スリープ
		rate.sleep();
	}

	// デバッグ
	std::cout << "Finiiiiiiiiiiiiiiiiiiiiiiish!" << std::endl;

	// 上記の処理で取得したすべてのWaypointデータをファイルに書き込む
	for(int i = 1; i < waypoint.size(); i++)
	{
		ofs << waypoint[i].x <<","<< waypoint[i].y << "," << waypoint[i].z << ","
		<< waypoint[i].ox << "," << waypoint[i].oy << "," << waypoint[i].oz << ","
		<< waypoint[i].ow << std::endl;
	}

	// ファイルを閉じる
	ofs.close();

	return 0;
}

