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
#include <cctype>
#include <algorithm>
#include <ros/ros.h>
#include<std_msgs/String.h>

// 戻り値：int型データの場合、Trueを戻す。逆の場合、Falseを戻す
// 引数：文字列
bool check_int(std::string str)
{
    if(std::all_of(str.cbegin(), str.cend(), isdigit))
    {
        std::cout << std::stoi(str) << std::endl;
        return true;
    }
    std::cout << "not int" << std::endl;
    return false;
}

// 戻り値：取得成功の場合、Trueを戻す。逆の場合、Falseを戻す。
// 引数1：1つ目のint型変数の参照
// 引数2：2つ目のint型変数の参照
// 機能：コンソール画面から2つのint型データを求める
bool get_number(int& first, int& second){
	std::string str;
	while(1){
		// 使用方法
		std::cout << "変更したいwaypointの番号を入力せよ" << std::endl;
		std::cout << "キャンセルする場合は c を入力せよ" << std::endl;
		// データを受け取る
		std::cin >> str;
		// int型データの場合
		if(check_int(str)){
			break;
		}
		// キャンセルの場合
		else if(str == "c"){
			std::cout << "キャンセル" << std::endl;
			return false;
		}
	}
	first = std::stoi(str);
	std::string stri;
	// 上記処理と類似する
	while(1){
		std::cout << "変更後のwaypointの番号を入力せよ" << std::endl;
		std::cout << "キャンセルする場合は c を入力せよ" << std::endl;
		std::cin >> stri;
		if(check_int(stri)){
			break;
		}else if(stri == "c"){
			std::cout << "キャンセル" << std::endl;
			return false;
		}
	}
	second = std::stoi(stri);
	return true;
}

int main(int argc, char **argv)
{
	// ROSの初期化
	ros::init(argc, argv, "wayp_command");
	ros::NodeHandle nh;

	// ROSパラメータの宣言
	ros::NodeHandle ph("~");
	std::string topic_name;
	ph.param("topic_name", topic_name, std::string("/commnad"));

	// ROSの送信APIパブリッシャの定義
	// テンプレート：送信するデータのメッセージタイプ
	// コンストラクト引数1：トピックの名前
	// コンストラクト引数2：データ列のサイズ
	ros::Publisher command_pub = nh.advertise<std_msgs::String>(topic_name, 100);

	// 繰り返し処理の待機時間
	ros::Rate rate(10);

	// 繰り返し処理
	// 繰り返し条件：当Nodeは実行されている場合
	while(ros::ok())
	{
		// このコマンドの使用方法をコンソール画面に出力する
		std::cout << "ファイルを保存する場合は save " << std::endl;
		std::cout << "waypointの番号を変更する場合は c "<< std::endl;
		std::cout << "を入力して!" << std::endl;
		std::string s;
		bool get_ok = false;

		while(ros::ok()){
			// コンソール画面から入力を受け取る
			std::cin >> s;
			// 入力が予想内の場合
			if(s == "save" || s == "c") get_ok = true;
			if(get_ok)break;
			// 入力が予想外の場合
			if(!get_ok) std::cout << "コマンドが間違っている" << std::endl;
		}

		std_msgs::String command;
		// 受け取ったコマンドは「save」の場合
		if(s == "save"){
			command.data = "save";
			// 「save」コマンドを「command_pub」インスタンスで送信する
			command_pub.publish(command);
		}
		// 受け取ったコマンドは「c」の場合
		else if(s == "c"){
			int first, secound;
			// int型データの取得に成功した場合
			if(get_number(first, secound)){
				std::ostringstream oss;
				oss << first<< "," <<secound;
				command.data = oss.str();
				command_pub.publish(command);
			}
		}
		rate.sleep();
	}

	return 0;
}