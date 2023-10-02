#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_listener.h>

// Pointの構造体
struct Point
{	
	// 三次元座標系
	double x;
	double y;
	double z;
	// 四元数座標 orientation
	double ox;
	double oy;
	double oz;
	double ow;
	// モード
    int mode;
};
int main(int argc, char **argv)
{
	// ROSのNodeの初期化
	// 引数1：端末の引数の数
	// 引数2：端末の引数が入れている配列、型はchar
	// 引数3：当Nodeの名前
	ros::init(argc, argv, "waypoint_pub");
	// NodeのAPIインスタンスを宣言する。Pathの領域はグローバルである
	ros::NodeHandle node;
	// NodeのAPIインスタンスを宣言する。Pathの領域はローカルである
	ros::NodeHandle ph("~");
	// ファイルのアドレスを一時的に保存する文字列インスタンスを宣言する
	std::string read_file;
	// 「ph」というローカルのNodeのインスタンスを用いて、キーが「read_file」であるROSパラメータを読み込み、一時的変数「read_file」に保存する
	ph.param("read_file", read_file, std::string("/home/useful/way.txt"));


	// 型は構造体Pointであるベクトル(コンテナ)のインスタンスを宣言する。
	std::vector<Point> waypoint;

	// ROSのPublisherのインスタンスを定義する。
	// テンプレート(「<>」符号に入っている内容)は送信するデータの型
	// 引数1：パブリッシャーのトピック
	// 引数2：データ列のサイズ
	ros::Publisher marker_pub = node.advertise<visualization_msgs::MarkerArray>("/waypoint_publisher", 100);

	// 繰り返し処理の周期の定義
	ros::Rate rate(10);

	// fopen()関数を用いてFILE構造体のポインターを定義する
	// 引数1：ファイルのアドレス、型はchar
	// 引数2：処理モードは読み取り(read)
	FILE *rfp = fopen(read_file.c_str(), "r");
	// 空ポインターの場合
	if (rfp == NULL)
	{
		ROS_ERROR("Don't exist file!");
		return -1;
	}

	// Point構造体のインスタンスを宣言する
	Point tmp;
	// 整数変数を宣言する
	int ret;
	// 繰り返し処理：
	// 		判断条件：fscanf()関数の戻り値は、「EOF」(ファイルの末尾)ではない場合
	while (ret = fscanf(rfp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d\n", &tmp.x, &tmp.y, &tmp.z, &tmp.ox, &tmp.oy, &tmp.oz, &tmp.ow, &tmp.mode) != EOF)
	{
		// tmpはファイルの一行で取得したデータ
		// そのデータをpush_back()関数を用いてwaypointベクトルに入れる
		waypoint.push_back(tmp);
    }
	// ファイルを閉じる
	fclose(rfp);

	// 繰り返し処理、判断条件はROSのプロセスは生きているかどうかである
	while(ros::ok())
	{
		// マーカーアレーのインスタンスの宣言
		// 矢印とテキスト二種類がある
		visualization_msgs::MarkerArray array_arrow,array_text;
		// マーカーのインスタンスの宣言
		// 矢印とテキスト二種類がある
        visualization_msgs::Marker arrow,text;
		// 矢印マーカーの座標ID
		arrow.header.frame_id = text.header.frame_id = "/map";
		// 矢印マーカーの時間スタンプ
		arrow.header.stamp = text.header.stamp = ros::Time::now();
		// 矢印マーカーのネームスペース
		arrow.ns = "ARROWS";
		// テキストマーカーのネームスペース
        text.ns = "TEXT";
		// 矢印マーカーとテキストマーカーの色RGB
		arrow.color.b = text.color.b = 1.0;
		text.color.g = 1.0;
		text.color.r = 1.0;
		arrow.color.a = text.color.a = 1.0;
		// 矢印マーカーとテキストマーカーの大きさ
		arrow.scale.x = 1.0;
		arrow.scale.y = 0.2;
		arrow.scale.z =text.scale.z = 0.4;
		// 矢印マーカーとテキストマーカーの動作は「追加」
		arrow.action =text.action = visualization_msgs::Marker::ADD;
		// 矢印マーカーの種類を定義する
		arrow.type = visualization_msgs::Marker::ARROW;
		// テキストマーカーの種類を定義する
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

		// 繰り返し処理、繰り返し回数はベクトルの要素数、iはベクトルのインデクスであり、0で初期化する
		for(int i = 0; i < waypoint.size(); i++)
		{
			// ROSメッセージインタフェイスのインスタンスを宣言する
			// ここのPoseの内容は最初のPoint構造体と対応する
			geometry_msgs::Pose p;
			// 三次元座標系の座標で定義する
			p.position.x = waypoint[i].x;
			p.position.y = waypoint[i].y;
			// 2D地図のため、高さは定数で定義する
			p.position.z = 0.25;
			// 回転量、四元数座標
            p.orientation.x=waypoint[i].ox;
            p.orientation.y=waypoint[i].oy;
            p.orientation.z=waypoint[i].oz;
            p.orientation.w=waypoint[i].ow;
			// テキストマーカーと矢印マーカーの位置を定義する
			arrow.pose=text.pose=p;
			// テキストマーカーのZ軸の数値も定数で定義する
			text.pose.position.z =0.5;
			// テキストマーカーと矢印マーカーの番号はインデクスの数値で定義する
			arrow.id=text.id=i;
            std::string str;
			// 矢印マーカーに関する説明は、テキストマーカーで行う
			// そのテキストの内容を下記のように示す
			// 「番号x、モードy」
            str="No."+std::to_string(i)+"-mode"+std::to_string(waypoint[i].mode);
			// 上記の内容でテキストマーカーのテキストを定義する
            text.text = str;
			// Waypointのモードは「0」である場合
			// Waypointのモード
			if(waypoint[i].mode==0){
				// 矢印マーカーは白色
				arrow.color.r=1.0;
			}else{
				// 「0」ではない場合、矢印マーカーは青水色
				arrow.color.r=0.0;
			}
			// 矢印マーカーアレーとテキストマーカーアレーのインスタンスのメンバーである「markers」ベクトルに矢印マーカーとテキストマーカーのインスタンスを入れる
            array_arrow.markers.push_back(arrow);
            array_text.markers.push_back(text);
		}
		// マーカーを送信する
		marker_pub.publish(array_arrow);
        marker_pub.publish(array_text);
		// 待機
		rate.sleep();
	}

	return 0;
}
